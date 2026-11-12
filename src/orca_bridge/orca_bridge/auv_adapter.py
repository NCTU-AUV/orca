#!/usr/bin/env python3
"""AuvControl → ArduSub：接收上層的 /auv_control，經 mavros 控制飛控。

- 6 自由度：以 20 Hz 持續送最後收到的那筆（MANUAL_CONTROL）。
  飛控在 MANUAL、或指令的模式是 MANUAL 時，一律送中位，推進器不出力。
- mode：跟飛控目前的模式不同就切換，沒切成功每秒重試。
- arm_command：收到 ARM／DISARM 且跟目前狀態不同時送一次。
- 定深：DEPTH_HOLD、已解鎖、use_depth_target 為 true 時送目標深度（SET_POSITION_TARGET_GLOBAL_INT）。
  use_depth_target 為 true 時 heave 一律當 0。
- 航向：DEPTH_HOLD、已解鎖、use_yaw_target 為 true 時送目標姿態（SET_ATTITUDE_TARGET），
  roll、pitch 保持水平，roll、pitch、yaw 的 effort 送 0。
- 目標深度、目標航向都是改變時立刻送，之後每秒重送（怕 UDP 掉封包；航向目標飛控只保留 5 秒）。
- 零點：呼叫 /auv_adapter/zero（std_srvs/srv/Trigger）時記下當下的深度和航向。
  剛下水、浮在水面時呼叫一次；沒歸零過就不送目標深度和目標航向。

上層停發時沿用最後一筆，不做斷線處理。還沒收到任何指令前只送中位，不動模式、不解鎖。
Ctrl-C 結束時送中位並上鎖。Ctrl-Z 也當成 Ctrl-C 處理。

前提：飛控 JS_GAIN_DEFAULT = 1.0（effort 1.0 才是全力），QGC 的 joystick 關閉。
用法：由 bridge.sh 與 mavros 一起啟動（deploy/*/compose.yaml 的 bridge 服務）
"""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import AttitudeTarget, GlobalPositionTarget, ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from orca_msgs.msg import AuvControl

RATE_HZ = 20          # MANUAL_CONTROL 的發送頻率；停送超過 FS_PILOT_TIMEOUT（3 秒）飛控會上鎖
RETRY_S = 1.0         # 模式沒切成功時，隔多久再送一次
TARGET_RESEND_S = 1.0

MODE_NAMES = {
    AuvControl.MODE_MANUAL: 'MANUAL',
    AuvControl.MODE_STABILIZE: 'STABILIZE',
    AuvControl.MODE_DEPTH_HOLD: 'ALT_HOLD',
}


def effort(v):
    """限制在 [-1, 1]。NaN／inf 當 0，否則 min/max 會把 NaN 變成全力。"""
    if not math.isfinite(v):
        return 0.0
    return max(-1.0, min(1.0, v))


def wrap_deg(v):
    """換算到 [-180, 180)。"""
    return (v + 180.0) % 360.0 - 180.0


class Adapter(Node):
    def __init__(self):
        super().__init__('auv_adapter')
        self.cmd = None             # 最後收到的 AuvControl
        self.state = State()        # 飛控狀態（mode、armed）
        self.depth = None           # 飛控座標的深度，往下為正（零點是飛控開機時）
        self.yaw = None             # 目前航向，ROS 慣例（ENU）：rad，逆時針為正
        self.surface = None         # 歸零時的飛控深度
        self.yaw0 = None            # 歸零時的航向（ENU, rad）
        self.busy = {'mode': False, 'arm': False}   # 服務呼叫還沒回覆
        self.last_mode_try = 0.0
        self.sent = {}              # 'depth'／'yaw' → (上次送的目標, 時間)

        self.pub = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)
        self.depth_pub = self.create_publisher(
            GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)
        self.att_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')

        self.create_subscription(AuvControl, '/auv_control', self.on_cmd, 10)
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        # rel_alt 是 EKF 融合深度計後的估測，向上為正
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_service(Trigger, '~/zero', self.on_zero)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)

    # ---------- 訂閱與服務 ----------

    def on_cmd(self, msg):
        if self.cmd is None or msg.control_source != self.cmd.control_source:
            source = '鍵盤' if msg.control_source == AuvControl.SW_MANUAL else '自主程式'
            self.get_logger().info(f'控制來源：{source}')
        self.cmd = msg
        # arm_command 是事件：只在收到的這一刻處理
        if msg.arm_command in (AuvControl.ARM, AuvControl.DISARM):
            want = msg.arm_command == AuvControl.ARM
            if want != self.state.armed and not self.busy['arm']:
                self.call(self.arming, CommandBool.Request(value=want), 'arm')

    def on_state(self, msg):
        self.state = msg

    def on_depth(self, msg):
        self.depth = -msg.data

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_zero(self, request, response):
        missing = [name for name, v in (('深度（/mavros/global_position/rel_alt）', self.depth),
                                        ('姿態（/mavros/imu/data）', self.yaw)) if v is None]
        if missing:
            response.success = False
            response.message = '還沒收到' + '、'.join(missing)
        else:
            self.surface = self.depth
            self.yaw0 = self.yaw
            self.sent.clear()
            response.success = True
            # 航向印成羅盤慣例：北 = 0°，順時針為正
            heading = wrap_deg(90.0 - math.degrees(self.yaw0)) % 360.0
            response.message = (f'歸零：飛控座標深度 {self.surface:.3f} m、'
                                f'航向 {heading:.1f}° 當作 0')
        self.get_logger().info(response.message)
        return response

    # ---------- 服務呼叫（非同步，等回覆時照樣送搖桿） ----------

    def call(self, client, request, what):
        if not client.service_is_ready():
            self.get_logger().warn(f'{client.srv_name} 還沒準備好，mavros 有在跑嗎？',
                                   throttle_duration_sec=2.0)
            return
        self.busy[what] = True
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.on_done(what, request, f))

    def on_done(self, what, request, future):
        self.busy[what] = False
        result = future.result()
        if what == 'arm':
            ok = result is not None and result.success
            self.get_logger().info(('解鎖' if request.value else '上鎖') + ('成功' if ok else '失敗'))
        else:
            # mode_sent 只代表指令送出去了，實際模式看 /mavros/state
            ok = result is not None and result.mode_sent
            self.get_logger().info(f'切到 {request.custom_mode}：' + ('已送出' if ok else '失敗'))

    # ---------- 主迴圈 ----------

    def on_timer(self):
        now = time.monotonic()
        c = self.cmd
        self.pub.publish(self.manual_control(c))
        if c is None:
            return

        want_mode = MODE_NAMES.get(c.mode)
        if want_mode is None:
            self.get_logger().warn(f'未知的 mode {c.mode}，不切換', throttle_duration_sec=2.0)
        elif (self.state.connected and self.state.mode != want_mode
              and not self.busy['mode'] and now - self.last_mode_try >= RETRY_S):
            self.last_mode_try = now
            self.call(self.set_mode, SetMode.Request(custom_mode=want_mode), 'mode')

        self.update_target('depth', c.use_depth_target, c.target_depth, self.send_depth, now)
        self.update_target('yaw', c.use_yaw_target, wrap_deg(c.target_yaw), self.send_yaw, now)

    def holding(self):
        """目標深度、目標航向只在 ALT_HOLD 且已解鎖時有效：解鎖前 ALT_HOLD 會一直重設目標。"""
        return self.state.mode == 'ALT_HOLD' and self.state.armed

    def yaw_active(self, c):
        return c.use_yaw_target and self.holding() and self.yaw0 is not None

    def manual_control(self, c):
        """AuvControl → MANUAL_CONTROL。MANUAL 時送中位。

        換算（ArduSub 4.7.2 joystick.cpp，JS_GAIN = 1.0）：
          x、y、r：RC = 1500 + 值 × 0.4，±1000 → 1100～1900
          z：0～1000，500 是中位
          s（pitch）、t（roll）：直接加在 1500 上，不乘增益，±400 就到 1100～1900
        """
        msg = ManualControl(x=0.0, y=0.0, z=500.0, r=0.0)
        if (c is None or c.mode == AuvControl.MODE_MANUAL
                or self.state.mode not in ('STABILIZE', 'ALT_HOLD')):
            return msg
        heave = 0.0 if c.use_depth_target else effort(c.heave)
        msg.x = effort(c.surge) * 1000.0
        msg.y = effort(c.sway) * 1000.0
        msg.z = 500.0 + heave * 500.0
        msg.enabled_extensions = 0b11   # bit 0 = s（pitch）、bit 1 = t（roll）有效
        if not self.yaw_active(c):      # 用目標航向時 roll、pitch、yaw 送 0（飛控本來也不看）
            msg.r = effort(c.yaw) * 1000.0
            msg.s = effort(c.pitch) * 400.0
            msg.t = effort(c.roll) * 400.0
        return msg

    def update_target(self, key, use, value, send, now):
        """目標改變時立刻送，之後每 TARGET_RESEND_S 秒重送。"""
        if not (use and self.holding()):
            self.sent.pop(key, None)
            return
        if self.surface is None:
            self.get_logger().warn('還沒歸零（/auv_adapter/zero），不送目標深度和目標航向',
                                   throttle_duration_sec=2.0)
            return
        if not math.isfinite(value):
            self.get_logger().warn(f'目標 {key} 是 {value}，不送', throttle_duration_sec=2.0)
            return
        last = self.sent.get(key)
        if last is not None and last[0] == value and now - last[1] < TARGET_RESEND_S:
            return
        self.sent[key] = (value, now)
        send(value)

    def send_depth(self, depth):
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        # ArduSub 在 ALT_HOLD 下只看 alt（向上為正），其他全部標成忽略
        msg.type_mask = (GlobalPositionTarget.IGNORE_LATITUDE | GlobalPositionTarget.IGNORE_LONGITUDE
                         | GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY
                         | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX
                         | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ
                         | GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE)
        msg.altitude = -(self.surface + depth)
        self.depth_pub.publish(msg)

    def send_yaw(self, yaw_deg):
        """目標航向（相對歸零時，順時針為正）→ 水平姿態 + 航向的四元數。

        mavros 吃 ROS 慣例（ENU，逆時針為正），會自己轉成飛控的 NED，所以這裡用減的。
        ArduSub 在 ALT_HOLD 下只在 type_mask 第 6 位元（IGNORE_THRUST）為 1 時才收；
        thrust 要給 0，mavros 才不會因為沒設 thrust_scaling 把整則訊息丟掉。
        """
        yaw = self.yaw0 - math.radians(yaw_deg)
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE
                         | AttitudeTarget.IGNORE_YAW_RATE | AttitudeTarget.IGNORE_THRUST)
        msg.orientation.w = math.cos(yaw / 2)
        msg.orientation.z = math.sin(yaw / 2)
        msg.thrust = 0.0
        self.att_pub.publish(msg)

    # ---------- 結束 ----------

    def shutdown(self):
        """送中位 0.5 秒後上鎖。"""
        self.cmd = None
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self.pub.publish(ManualControl(x=0.0, y=0.0, z=500.0, r=0.0))
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.arming.wait_for_service(timeout_sec=2.0):
            future = self.arming.call_async(CommandBool.Request(value=False))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            ok = future.result() is not None and future.result().success
            print('上鎖' + ('成功' if ok else '失敗'))


def on_ctrl_z(signum, frame):
    # Ctrl-Z 預設會把程式暫停在背景，最後一筆指令會一直生效到飛控 failsafe。改成跟 Ctrl-C 一樣
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTSTP, on_ctrl_z)
    # 不讓 rclpy 處理 Ctrl-C：它預設會先關掉 ROS，結束時的上鎖就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Adapter()
    node.get_logger().info('auv_adapter 啟動，等 /auv_control')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl-C')
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
