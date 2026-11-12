#!/usr/bin/env python3
"""下水測試（water_*.py）共用的部分，不單獨執行。

- 以 20 Hz 發 /auv_control，控制來源固定是自主程式（SW_AUTO）。
- 讀飛控狀態、深度、航向、8 顆馬達的 PWM，約 5 Hz 記錄到 /logs/<名稱>_<日期_時間>.csv
  （容器的 /logs 掛到 deploy/<機器>/logs/bridge/）。
- 開始前：確認 mavros、adapter 在跑、飛控上鎖，請使用者確認檢查清單，呼叫 /auv_adapter/zero 歸零。
- 結束時（正常結束、Ctrl-C、Ctrl-Z、程式出錯都一樣）：送 MANUAL、全 0、DISARM，確認上鎖。
  上鎖後船靠浮力浮上來。

單位與方向：深度以歸零時為 0，往下為正（m）；航向以歸零時為 0，順時針為正（度）。
"""
import csv
import math
import signal
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import ManualControl, RCOut, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from orca_msgs.msg import AuvControl

MAX_DEPTH = 0.25    # 池子最深 0.25 m（以浮著時為 0），目標深度不能超過
RATE_HZ = 20
MANUAL, DEPTH_HOLD = AuvControl.MODE_MANUAL, AuvControl.MODE_DEPTH_HOLD
MODE_FC = {AuvControl.MODE_MANUAL: 'MANUAL', AuvControl.MODE_STABILIZE: 'STABILIZE',
           AuvControl.MODE_DEPTH_HOLD: 'ALT_HOLD'}

# 從 PWM 反推飛控的垂直出力（µs，正 = 往上）：VECTORED_6DOF 5～8 號的 throttle 係數都是 -1
# （AP_Motors6DOF.cpp），乘上 MOT_n_DIRECTION（+1 -1 -1 +1）。roll、pitch 的係數跟它正交，不會混進來。
# 浮力是正的，定深穩定時應該是負的（往下推），11-06 量到約 -60～-70。
VERT_SIGN = (-1, 1, 1, -1)

CHECKLIST = [
    '船浮在水面、靜止（等一下會以這時的深度和航向當作 0）',
    '繩子綁好，kill switch 在手邊',
    'QGC 的 joystick 關閉',
    'mavproxy 的 log 已出現 MAG0 initial yaw alignment complete',
    'bridge 服務（mavros + auv_adapter）在跑',
]


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def wait_until(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


class Tester(Node):
    def __init__(self, name, log_path):
        super().__init__(name)
        # 要發送的指令（主執行緒改，timer 讀）
        self.sending = False
        self.mode = MANUAL
        self.surge = 0.0
        self.target_depth = 0.0
        self.use_depth = False
        self.target_yaw = 0.0
        self.use_yaw = False
        self.arm_event = AuvControl.ARM_NO_CHANGE
        self.lock = threading.Lock()
        self.step = ''
        self.report = []            # 結束時印出的結果

        # 收到的資料
        self.state = None
        self.pwm = None
        self.fc_depth = None        # 飛控座標，往下為正
        self.yaw_enu = None         # rad，ROS 慣例：逆時針為正
        self.surface = None         # 歸零時的 fc_depth
        self.yaw0 = None            # 歸零時的 yaw_enu
        self.adapter_t = None

        self.pub = self.create_publisher(AuvControl, '/auv_control', 10)
        self.zero_client = self.create_client(Trigger, '/auv_adapter/zero')
        self.create_subscription(State, '/mavros/state',
                                 lambda m: setattr(self, 'state', m), qos_profile_sensor_data)
        self.create_subscription(RCOut, '/mavros/rc/out',
                                 lambda m: setattr(self, 'pwm', list(m.channels[:8])),
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, 'fc_depth', -m.data), qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(ManualControl, '/mavros/manual_control/send',
                                 lambda m: setattr(self, 'adapter_t', time.monotonic()), 10)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.create_timer(0.2, self.on_log)

        self.f = open(log_path, 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['unix_time', 'step', 'cmd_mode', 'surge', 'use_depth_target', 'target_depth',
                           'use_yaw_target', 'target_yaw', 'fc_mode', 'armed', 'depth_m', 'heading_deg',
                           'vertical_out'] + [f'pwm{i}' for i in range(1, 9)])

    # ---------- 發送 ----------

    def on_timer(self):
        if not self.sending:
            return
        with self.lock:
            event, self.arm_event = self.arm_event, AuvControl.ARM_NO_CHANGE
        self.pub.publish(AuvControl(
            control_source=AuvControl.SW_AUTO, mode=self.mode, arm_command=event,
            surge=self.surge, target_depth=self.target_depth, use_depth_target=self.use_depth,
            target_yaw=self.target_yaw, use_yaw_target=self.use_yaw))

    def send_arm(self, command):
        """ARM／DISARM 是事件：只放進下一筆，之後自動回到 NO_CHANGE。"""
        with self.lock:
            self.arm_event = command

    # ---------- 接收與換算 ----------

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_log(self):
        if self.surface is None:
            return
        d, h, v = self.depth(), self.heading(), self.vertical()
        self.log.writerow([
            round(time.time(), 3), self.step, MODE_FC[self.mode], self.surge,
            self.use_depth, self.target_depth, self.use_yaw, self.target_yaw,
            self.state.mode if self.state else '', self.state.armed if self.state else '',
            '' if d is None else round(d, 4), '' if h is None else round(h, 2),
            '' if v is None else round(v, 1)] + (self.pwm or [''] * 8))
        self.f.flush()

    def armed(self):
        return self.state is not None and self.state.armed

    def depth(self):
        if self.fc_depth is None or self.surface is None:
            return None
        return self.fc_depth - self.surface

    def heading(self):
        if self.yaw_enu is None or self.yaw0 is None:
            return None
        return wrap(-math.degrees(self.yaw_enu - self.yaw0))

    def vertical(self):
        if self.pwm is None:
            return None
        return sum(s * (p - 1500) for s, p in zip(VERT_SIGN, self.pwm[4:8])) / 4

    def adapter_alive(self):
        return self.adapter_t is not None and time.monotonic() - self.adapter_t < 0.5

    def status(self):
        d, h, v = self.depth(), self.heading(), self.vertical()
        parts = [f'{self.state.mode if self.state else "?"}', '解鎖' if self.armed() else '上鎖']
        parts.append('深度 ---' if d is None else
                     f'深度 {d:6.3f} m' + (f'（目標 {self.target_depth:.2f}）' if self.use_depth else ''))
        parts.append('航向 ---' if h is None else
                     f'航向 {h:+6.1f}°' + (f'（目標 {self.target_yaw:+.0f}）' if self.use_yaw else ''))
        if self.surge:
            parts.append(f'surge {self.surge:+.2f}')
        parts.append('垂直 ---' if v is None else f'垂直 {v:+4.0f}')
        return '  '.join(parts)

    # ---------- 動作 ----------

    def set_mode(self, code, timeout=5.0):
        self.mode = code
        return wait_until(lambda: self.state is not None and self.state.mode == MODE_FC[code], timeout)

    def arm(self, on, timeout=5.0):
        self.send_arm(AuvControl.ARM if on else AuvControl.DISARM)
        return wait_until(lambda: self.armed() == on, timeout)

    def zero(self):
        if not self.zero_client.wait_for_service(timeout_sec=3.0):
            print('找不到 /auv_adapter/zero，adapter 有在跑嗎？')
            return False
        future = self.zero_client.call_async(Trigger.Request())
        res = future.result() if wait_until(future.done, 5.0) else None
        if res is None or not res.success:
            print('歸零失敗：' + (res.message if res else '沒有回覆'))
            return False
        # 跟 adapter 同時記，之後顯示的深度、航向跟 adapter 用同一個 0
        self.surface, self.yaw0 = self.fc_depth, self.yaw_enu
        print(res.message)
        return True

    def dive(self, depth):
        """切 DEPTH_HOLD、設好目標深度後解鎖（adapter 解鎖後才送目標）。失敗回傳 False。"""
        self.target_depth = depth
        self.use_depth = True
        self.sending = True
        if not self.set_mode(DEPTH_HOLD):
            print(f'沒有切進 ALT_HOLD（目前 {self.state.mode}），深度計正常嗎？')
            return False
        if not self.arm(True):
            print('解鎖失敗')
            return False
        print('已解鎖')
        return True

    def hold(self, seconds, label, until=None):
        """維持目前的指令 seconds 秒（until() 為 True 時提早結束），每 0.5 秒印一次狀態。

        回傳 [(秒數, 深度, 航向)]，秒數從這一段開始算。
        """
        self.step = label
        print(f'--- {label}')
        t0 = time.monotonic()
        samples = []
        next_print = 0.0
        while time.monotonic() - t0 < seconds:
            now = time.monotonic()
            samples.append((now - t0, self.depth(), self.heading()))
            if now >= next_print:
                next_print = now + 0.5
                print(f'  {now - t0:5.1f}s  {self.status()}')
            if until is not None and until():
                break
            if not self.armed():
                raise RuntimeError('飛控在測試中上鎖了（failsafe？），中止')
            time.sleep(0.05)
        return samples

    def finish(self):
        """MANUAL、全 0、DISARM，確認上鎖。"""
        self.step = 'finish'
        self.surge = 0.0
        self.use_depth = False
        self.use_yaw = False
        self.mode = MANUAL
        self.sending = True
        self.send_arm(AuvControl.DISARM)
        ok = wait_until(lambda: not self.armed(), 5.0)
        print('飛控已上鎖，船會浮上來' if ok else '飛控沒有上鎖！用 QGC 或 kill switch 處理')


def step_stats(samples, target, start, tol, angle=False, final_s=10.0):
    """一段目標的統計。samples：[(秒數, 值)]。angle=True 時誤差換算到 ±180。

    回傳 (到達秒數或 None, 過衝, 最後 final_s 秒平均誤差, 最後 final_s 秒波動)。
    到達 = 誤差第一次進入 ±tol；過衝 = 超過目標（沿移動方向）的最大量。
    """
    pts = [(s, v) for s, v in samples if v is not None]
    if not pts:
        return None
    err = (lambda v: wrap(v - target)) if angle else (lambda v: v - target)
    direction = math.copysign(1.0, wrap(target - start) if angle else target - start)
    reach = next((s for s, v in pts if abs(err(v)) <= tol), None)
    overshoot = max(0.0, max(direction * err(v) for _, v in pts))
    end = pts[-1][0]
    last = [err(v) for s, v in pts if s >= end - final_s]
    return reach, overshoot, sum(last) / len(last), max(last) - min(last)


def format_stats(label, stats, unit, scale=1.0, final_s=10.0):
    if stats is None:
        return f'{label}：沒有資料'
    reach, overshoot, mean, p2p = stats
    reach_text = '沒到達' if reach is None else f'{reach:4.1f} s'
    return (f'{label}：到達 {reach_text}，過衝 {overshoot * scale:4.1f} {unit}，'
            f'最後 {final_s:.0f} 秒平均誤差 {mean * scale:+5.1f} {unit}、波動 {p2p * scale:4.1f} {unit}')


def on_ctrl_z(signum, frame):
    # Ctrl-Z 預設會把程式暫停在背景，最後一筆指令會被 adapter 一直沿用。改成跟 Ctrl-C 一樣
    raise KeyboardInterrupt


def run(name, body):
    """共用的開始與收尾。body(t) 是各測試的流程。"""
    signal.signal(signal.SIGTSTP, on_ctrl_z)
    # 不讓 rclpy 處理 Ctrl-C：它預設會先關掉 ROS，結束時的 DISARM 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    log_path = time.strftime(f'/logs/{name}_%Y%m%d_%H%M%S.csv')
    t = Tester(name, log_path)
    executor = SingleThreadedExecutor()
    executor.add_node(t)
    threading.Thread(target=executor.spin, daemon=True).start()
    print(f'記錄到 {log_path}')

    started = False     # 開始送指令後才需要收尾
    try:
        if not wait_until(lambda: t.state is not None and t.state.connected, 5.0):
            print('收不到 /mavros/state 或飛控沒連線，mavros 有在跑嗎？')
            return
        if not wait_until(t.adapter_alive, 3.0):
            print('沒收到 adapter 送的 MANUAL_CONTROL，adapter 有在跑嗎？')
            return
        if t.armed():
            print('飛控是解鎖狀態，先上鎖再跑')
            return
        print('開始前確認：')
        for item in CHECKLIST:
            print(f'  - {item}')
        input('  >>> 都確認了按 Enter（Ctrl-C 取消）')
        if not t.zero():
            return
        started = True
        print('\n測試流程中止' if body(t) is False else '\n測試流程跑完')
    except KeyboardInterrupt:
        print('\nCtrl-C，中止')
    except RuntimeError as e:
        print(f'\n{e}')
    finally:
        if started:
            t.finish()
        if t.report:
            print('\n========== 結果 ==========')
            for line in t.report:
                print('  ' + line)
        print(f'記錄在 {log_path}')
        time.sleep(0.3)     # 讓最後幾筆指令送出去
        executor.shutdown()
        t.f.close()
        rclpy.shutdown()
