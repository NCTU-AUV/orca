#!/usr/bin/env python3
"""mission/ 底下的流程共用的 ROS 部分，不單獨執行。

- 控制：只發 /auv_control（20 Hz，控制來源 SW_AUTO），模式固定 DEPTH_HOLD、用目標深度和目標航向。
  不直接對 mavros 下任何指令。
- 狀態（只讀）：/mavros/state、/mavros/global_position/rel_alt、/mavros/imu/data，
  換算方式跟 auv_adapter.py 相同。零點在呼叫 /auv_adapter/zero 的同時記下，
  所以這裡的深度、航向跟 target_depth、target_yaw 是同一個座標。
- 影像：/detections（orca_perception）配上 camera_info 的內參 → 門的方位角、水平偏移、門框寬度。
- 開始前：確認 mavros、adapter、相機、detector 都在跑、飛控上鎖，請使用者確認檢查清單，歸零。
- 結束時（走完、中止、Ctrl-C、Ctrl-Z、程式出錯都一樣）：送 MANUAL、全 0、DISARM，確認上鎖。
- 約 20 Hz 記錄到 /logs/<名稱>_<日期_時間>.csv（容器的 /logs 掛到 deploy/<機器>/logs/bridge/）。

流程物件要有：first_cmd()、step(now, depth, yaw, gate) → Cmd、phase、result、message、log
（見 gate_pass.py）。

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
from mavros_msgs.msg import ManualControl, State
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray
from orca_msgs.msg import AuvControl

from gate_pass import Gate, wrap

RATE_HZ = 20
STALE_S = 1.0       # 深度、姿態超過這麼久沒更新就當作沒有（遙測約 4 Hz）

CHECKLIST = [
    '船浮在水面、靜止（等一下會以這時的深度和航向當作 0）',
    '門在相機前方、看得到（/detections/image 有框到門）',
    '繩子綁好，kill switch 在手邊',
    'QGC 的 joystick 關閉',
    'mavproxy 的 log 已出現 MAG0 initial yaw alignment complete',
]


def wait_until(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


class Runner(Node):
    def __init__(self, name, classes, min_score, log_path):
        super().__init__(name)
        self.classes = set(classes)
        self.min_score = min_score
        # 要發送的指令（主執行緒改，timer 讀）
        self.cmd = None             # AuvControl；None = 還不發
        self.arm_event = AuvControl.ARM_NO_CHANGE
        self.lock = threading.Lock()

        # 收到的資料
        self.state = None
        self.fc_depth = None        # 飛控座標，往下為正
        self.yaw_enu = None         # rad，ROS 慣例：逆時針為正
        self.depth_t = 0.0          # 最後一次收到的時間（time.monotonic()）
        self.yaw_t = 0.0
        self.surface = None         # 歸零時的 fc_depth
        self.yaw0 = None            # 歸零時的 yaw_enu
        self.adapter_t = None
        self.cam = None             # (畫面寬, fx, cx)
        self.det_t = None           # 最後一次收到 /detections
        self.gate = None            # 最新一筆門的偵測（gate_pass.Gate）

        self.pub = self.create_publisher(AuvControl, '/auv_control', 10)
        self.zero_client = self.create_client(Trigger, '/auv_adapter/zero')
        self.create_subscription(State, '/mavros/state',
                                 lambda m: setattr(self, 'state', m), qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(ManualControl, '/mavros/manual_control/send',
                                 lambda m: setattr(self, 'adapter_t', time.monotonic()), 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections', self.on_detections, 10)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)

        self.f = open(log_path, 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['unix_time', 'phase', 'fc_mode', 'armed', 'depth_m', 'heading_deg',
                           'target_depth', 'target_yaw', 'surge', 'sway', 'gate_age_s',
                           'gate_bearing_deg', 'gate_offset', 'gate_width', 'gate_score', 'gate_class'])

    # ---------- 發送 ----------

    def on_timer(self):
        with self.lock:
            cmd = self.cmd
            if cmd is None:
                return
            cmd.arm_command, self.arm_event = self.arm_event, AuvControl.ARM_NO_CHANGE
        self.pub.publish(cmd)

    def send(self, c):
        """流程的 Cmd → 之後每一筆 /auv_control。"""
        msg = AuvControl(
            control_source=AuvControl.SW_AUTO, mode=AuvControl.MODE_DEPTH_HOLD,
            surge=float(c.surge), sway=float(c.sway),
            target_depth=float(c.target_depth), use_depth_target=True,
            target_yaw=float(c.target_yaw), use_yaw_target=True)
        with self.lock:
            self.cmd = msg

    def send_arm(self, command):
        """ARM／DISARM 是事件：只放進下一筆，之後自動回到 NO_CHANGE。"""
        with self.lock:
            self.arm_event = command

    # ---------- 接收與換算 ----------

    def on_depth(self, msg):
        self.fc_depth = -msg.data       # rel_alt 向上為正
        self.depth_t = time.monotonic()

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw_t = time.monotonic()

    def on_camera_info(self, msg):
        if msg.width and msg.k[0] > 0:
            self.cam = (msg.width, msg.k[0], msg.k[2])

    def on_detections(self, msg):
        now = time.monotonic()
        self.det_t = now
        if self.cam is None:
            return
        best = None     # (分數, 類別, 框)；同一幀有多個門時取分數最高的
        for d in msg.detections:
            for r in d.results:
                h = r.hypothesis
                if (h.class_id in self.classes and h.score >= self.min_score
                        and (best is None or h.score > best[0])):
                    best = (h.score, h.class_id, d.bbox)
        if best is None:
            return
        width, fx, cx = self.cam
        score, label, box = best
        u = box.center.position.x
        seq = self.gate.seq + 1 if self.gate else 1
        self.gate = Gate(seq=seq, t=now, bearing=math.degrees(math.atan2(u - cx, fx)),
                         offset=(u - cx) / (width / 2), width=box.size_x / width,
                         score=score, label=label)

    def armed(self):
        return self.state is not None and self.state.armed

    def depth(self, now):
        if self.surface is None or now - self.depth_t > STALE_S:
            return math.nan
        return self.fc_depth - self.surface

    def heading(self, now):
        if self.yaw0 is None or now - self.yaw_t > STALE_S:
            return math.nan
        return wrap(-math.degrees(self.yaw_enu - self.yaw0))

    def adapter_alive(self):
        return self.adapter_t is not None and time.monotonic() - self.adapter_t < 0.5

    # ---------- 動作 ----------

    def zero(self):
        if not self.zero_client.wait_for_service(timeout_sec=3.0):
            print('找不到 /auv_adapter/zero，adapter 有在跑嗎？')
            return False
        future = self.zero_client.call_async(Trigger.Request())
        res = future.result() if wait_until(future.done, 5.0) else None
        if res is None or not res.success:
            print('歸零失敗：' + (res.message if res else '沒有回覆'))
            return False
        # 跟 adapter 同時記，之後的深度、航向跟 adapter 用同一個 0
        self.surface, self.yaw0 = self.fc_depth, self.yaw_enu
        print(res.message)
        return True

    def start(self, first_cmd):
        """切 DEPTH_HOLD、設好目標後解鎖（adapter 解鎖後才送目標）。失敗回傳 False。"""
        self.send(first_cmd)
        if not wait_until(lambda: self.state.mode == 'ALT_HOLD', 5.0):
            print(f'沒有切進 ALT_HOLD（目前 {self.state.mode}），深度計正常嗎？')
            return False
        self.send_arm(AuvControl.ARM)
        if not wait_until(self.armed, 5.0):
            print('解鎖失敗')
            return False
        print('已解鎖')
        return True

    def finish(self):
        """MANUAL、全 0、DISARM，確認上鎖。"""
        with self.lock:
            self.cmd = AuvControl(control_source=AuvControl.SW_AUTO, mode=AuvControl.MODE_MANUAL)
            self.arm_event = AuvControl.DISARM
        ok = wait_until(lambda: not self.armed(), 5.0)
        print('飛控已上鎖，船會浮上來' if ok else '飛控沒有上鎖！用 QGC 或 kill switch 處理')

    def record(self, phase, now, depth, heading, c):
        g = self.gate
        num = lambda v, n: '' if not math.isfinite(v) else round(v, n)
        self.log.writerow(
            [round(time.time(), 3), phase, self.state.mode, self.state.armed,
             num(depth, 4), num(heading, 2), c.target_depth, round(c.target_yaw, 2),
             round(c.surge, 3), round(c.sway, 3)]
            + ([''] * 6 if g is None else [round(now - g.t, 3), round(g.bearing, 2), round(g.offset, 3),
                                           round(g.width, 3), round(g.score, 2), g.label]))

    def status(self, phase, now, depth, heading, c):
        g = self.gate
        parts = [f'{phase:8s}', f'深度 {depth:5.2f} m', f'航向 {heading:+6.1f}°（目標 {c.target_yaw:+6.1f}）',
                 f'surge {c.surge:+.2f} sway {c.sway:+.2f}']
        if g is None or now - g.t > 1.0:
            parts.append('門 ---')
        else:
            parts.append(f'門 方位 {g.bearing:+5.1f}° 偏移 {g.offset:+.2f} 寬 {g.width:.2f}（{g.label} {g.score:.2f}）')
        return '  '.join(parts)


def ready(t):
    """開始前的檢查，有問題就印出原因並回傳 False。"""
    if not wait_until(lambda: t.state is not None and t.state.connected, 5.0):
        print('收不到 /mavros/state 或飛控沒連線，mavros 有在跑嗎？')
    elif not wait_until(t.adapter_alive, 3.0):
        print('沒收到 adapter 送的 MANUAL_CONTROL，adapter 有在跑嗎？')
    elif t.armed():
        print('飛控是解鎖狀態，先上鎖再跑')
    elif not wait_until(lambda: t.cam is not None, 5.0):
        print('收不到 /camera/camera/color/camera_info，相機（cam_bridge）有在跑嗎？')
    elif not wait_until(lambda: t.det_t is not None, 5.0):
        print('收不到 /detections，detector 有在跑嗎？')
    else:
        return True
    return False


def loop(t, flow):
    """以 RATE_HZ 跑流程直到它結束。回傳 flow.result。"""
    next_print = 0.0
    while flow.result is None:
        now = time.monotonic()
        if not t.armed():
            raise RuntimeError('飛控在流程中上鎖了（failsafe？），中止')
        depth, heading = t.depth(now), t.heading(now)
        c = flow.step(now, depth, heading, t.gate)
        t.send(c)
        t.record(flow.phase, now, depth, heading, c)
        if now >= next_print:
            next_print = now + 0.5
            print('  ' + t.status(flow.phase, now, depth, heading, c))
        time.sleep(1.0 / RATE_HZ)
    print(flow.message)
    return flow.result


def on_ctrl_z(signum, frame):
    # Ctrl-Z 預設會把程式暫停在背景，最後一筆指令會被 adapter 一直沿用。改成跟 Ctrl-C 一樣
    raise KeyboardInterrupt


def run(name, flow, classes, min_score):
    """共用的開始與收尾。"""
    signal.signal(signal.SIGTSTP, on_ctrl_z)
    # 不讓 rclpy 處理 Ctrl-C：它預設會先關掉 ROS，結束時的 DISARM 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    log_path = time.strftime(f'/logs/{name}_%Y%m%d_%H%M%S.csv')
    t = Runner(name, classes, min_score, log_path)
    executor = SingleThreadedExecutor()
    executor.add_node(t)
    threading.Thread(target=executor.spin, daemon=True).start()
    print(f'記錄到 {log_path}')

    started = False     # 開始送指令後才需要收尾
    try:
        if not ready(t):
            return
        print('開始前確認：')
        for item in CHECKLIST:
            print(f'  - {item}')
        input('  >>> 都確認了按 Enter（Ctrl-C 取消）')
        if not t.zero():
            return
        started = True
        if t.start(flow.first_cmd()):
            loop(t, flow)
    except KeyboardInterrupt:
        print('\nCtrl-C，中止')
    except RuntimeError as e:
        print(f'\n{e}')
    finally:
        if started:
            t.finish()
        print(f'記錄在 {log_path}')
        time.sleep(0.3)     # 讓最後幾筆指令送出去
        executor.shutdown()
        t.f.close()
        rclpy.shutdown()
