#!/usr/bin/env python3
"""鍵盤操控：pygame 視窗，按住才出力、放開就停，發 /auv_control（控制來源：鍵盤）。

按鍵（視窗要有焦點）：
  W/S surge、A/D sway、R/F heave、Q/E yaw：按住出力，大小是 GAIN
  1 MANUAL、2 STABILIZE、3 DEPTH_HOLD
  Enter 解鎖、空白鍵 上鎖
  Z 歸零（只在上鎖時；以當下深度、航向為 0）
  H 定深開關：開啟時目標 = 目前深度，按住 R/F 以 TARGET_RATE 移動目標（heave 不出力）
  - / = 調整 GAIN、Ctrl+Q 或關閉視窗 結束

安全：
- /auv_control 由主迴圈直接發（20 Hz），不用 ROS timer。畫面卡住發送就停，
  0.5 秒後 adapter 的看門狗改送中位。
- 視窗失去焦點時所有軸歸零；焦點回來後，要先放開所有移動鍵才會再出力。
- 結束時（關視窗、Ctrl+Q、Ctrl-C、SIGTERM、SIGHUP、Ctrl-Z）送 MANUAL、全 0、DISARM，確認上鎖。
- mavros 沒連上或 adapter 沒在跑時不發任何指令；開始發送時的模式沿用飛控目前的模式。

用法（容器內；主機先跑 xhost +SI:localuser:root）：
  docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py [--gain 0.3] [--max-depth 0.25]
"""
import argparse
import math
import os
import signal
import threading
import time

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
import pygame  # noqa: E402
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import ManualControl, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from orca_msgs.msg import AuvControl

RATE_HZ = 20
GAIN_STEP, GAIN_MIN, GAIN_MAX = 0.1, 0.1, 1.0
TARGET_RATE = 0.05      # 定深時按住 R/F，目標深度每秒移動幾公尺
ADAPTER_TIMEOUT = 0.5   # 這麼久沒看到 adapter 送的 MANUAL_CONTROL 就當它沒在跑
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'

M = AuvControl
MODE_FC = {M.MODE_MANUAL: 'MANUAL', M.MODE_STABILIZE: 'STABILIZE', M.MODE_DEPTH_HOLD: 'ALT_HOLD'}
FC_MODE = {v: k for k, v in MODE_FC.items()}
MODE_KEYS = {pygame.K_1: M.MODE_MANUAL, pygame.K_2: M.MODE_STABILIZE, pygame.K_3: M.MODE_DEPTH_HOLD}
# 軸：（名稱, 正方向鍵, 負方向鍵）。正負號照 AuvControl.msg
AXES = [('SURGE', pygame.K_w, pygame.K_s), ('SWAY', pygame.K_d, pygame.K_a),
        ('HEAVE', pygame.K_r, pygame.K_f), ('YAW', pygame.K_e, pygame.K_q)]
MOVE_KEYS = [k for _, pos, neg in AXES for k in (pos, neg)]

# 顏色
BG, FG, DIM = (28, 28, 32), (225, 225, 225), (130, 130, 140)
GREEN, RED, GRAY, YELLOW = (60, 190, 90), (215, 60, 60), (90, 90, 100), (230, 190, 60)


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class Teleop(Node):
    """ROS 那一側：訂閱飛控狀態、發 /auv_control、呼叫歸零。發送的時機由主迴圈決定。"""

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.state = None
        self.fc_depth = None    # 飛控座標，往下為正
        self.yaw_enu = None     # rad，ROS 慣例：逆時針為正
        self.surface = None     # 歸零時的 fc_depth
        self.yaw0 = None        # 歸零時的 yaw_enu
        self.adapter_t = None
        self.message = ''

        self.pub = self.create_publisher(AuvControl, '/auv_control', 10)
        self.zero_client = self.create_client(Trigger, '/auv_adapter/zero')
        self.create_subscription(State, '/mavros/state',
                                 lambda m: setattr(self, 'state', m), qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, 'fc_depth', -m.data), qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(ManualControl, '/mavros/manual_control/send',
                                 lambda m: setattr(self, 'adapter_t', time.monotonic()), 10)

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def say(self, text):
        self.message = text
        self.get_logger().info(text)

    # ---------- 狀態 ----------

    def connected(self):
        return self.state is not None and self.state.connected

    def armed(self):
        return self.state is not None and self.state.armed

    def adapter_alive(self):
        return self.adapter_t is not None and time.monotonic() - self.adapter_t < ADAPTER_TIMEOUT

    def depth(self):
        if self.fc_depth is None or self.surface is None:
            return None
        return self.fc_depth - self.surface

    def heading(self):
        """相對歸零時的航向（度），順時針為正。"""
        if self.yaw_enu is None or self.yaw0 is None:
            return None
        return wrap(-math.degrees(self.yaw_enu - self.yaw0))

    # ---------- 歸零（非同步，不卡畫面） ----------

    def zero(self):
        if self.armed():
            self.say('zero ignored: disarm first')
            return
        if not self.zero_client.service_is_ready():
            self.say('zero failed: /auv_adapter/zero not available')
            return
        self.zero_client.call_async(Trigger.Request()).add_done_callback(self.on_zero)

    def on_zero(self, future):
        res = future.result()
        if res is None or not res.success:
            self.say('zero failed: ' + (res.message if res else 'no response'))
            return
        # 跟 adapter 同時記，畫面上的深度、航向跟 adapter 用同一個 0
        self.surface, self.yaw0 = self.fc_depth, self.yaw_enu
        self.say('zeroed: current depth and heading are now 0')


class Screen:
    """畫面。只負責畫，不改任何狀態。"""

    def __init__(self):
        self.surf = pygame.display.set_mode((640, 430))
        try:
            self.font = pygame.font.Font(FONT, 16)
            self.big = pygame.font.Font(FONT, 20)
        except (FileNotFoundError, OSError):
            self.font = pygame.font.Font(None, 22)
            self.big = pygame.font.Font(None, 28)

    def text(self, s, x, y, color=FG, font=None):
        img = (font or self.font).render(s, True, color)
        self.surf.blit(img, (x, y))
        return x + img.get_width()

    def line(self, y):
        pygame.draw.line(self.surf, GRAY, (12, y), (628, y))

    def bar(self, x, y, v):
        """[-1, 1] 的橫條，中間是 0。"""
        w, h = 220, 14
        pygame.draw.rect(self.surf, GRAY, (x, y, w, h), 1)
        mid = x + w // 2
        n = int(abs(v) * (w // 2))
        if n:
            pygame.draw.rect(self.surf, YELLOW, (mid if v > 0 else mid - n, y + 1, n, h - 2))
        pygame.draw.line(self.surf, FG, (mid, y - 2), (mid, y + h + 1))

    def draw(self, t, ui):
        s = self.surf
        s.fill(BG)
        fc_mode = t.state.mode if t.state else '---'
        pygame.display.set_caption(f'Orca Teleop - {fc_mode} {"ARMED" if t.armed() else "DISARMED"}')

        # 狀態
        y = 12
        x = self.text('FC: ', 16, y)
        x = self.text('CONNECTED' if t.connected() else 'DISCONNECTED',
                      x, y, GREEN if t.connected() else RED)
        x = self.text(f'   MODE: {fc_mode}   ', x, y)
        label = ' ARMED ' if t.armed() else ' DISARMED '
        img = self.font.render(label, True, FG)
        pygame.draw.rect(s, RED if t.armed() else GRAY, (x, y - 2, img.get_width(), img.get_height() + 4))
        s.blit(img, (x, y))
        x = self.text('   ADAPTER: ', x + img.get_width(), y)
        self.text('OK' if t.adapter_alive() else 'NO DATA', x, y, GREEN if t.adapter_alive() else RED)
        y += 24
        self.text('ZERO: ' + ('set' if t.surface is not None else 'not set (press Z while disarmed)'),
                  16, y, FG if t.surface is not None else YELLOW)
        y += 26
        self.line(y)

        # 深度、航向、GAIN
        y += 10
        d, h = t.depth(), t.heading()
        self.text('DEPTH     ' + ('---' if d is None else f'{d:6.3f} m'), 16, y)
        if ui.hold:
            active = fc_mode == 'ALT_HOLD' and t.armed()
            self.text(f'TARGET {ui.target:5.3f} m   DEPTH HOLD: ON' + ('' if active else ' (idle)'),
                      250, y, FG if active else DIM)
        else:
            self.text('DEPTH HOLD: OFF', 250, y, DIM)
        y += 24
        self.text('HEADING   ' + ('---' if h is None else f'{h:+6.1f} deg'), 16, y)
        y += 24
        self.text(f'GAIN      {ui.gain:4.2f}   (- / = to change)', 16, y)
        y += 26
        self.line(y)

        # 各軸
        y += 12
        for (name, pos, neg), v in zip(AXES, ui.efforts):
            keys = f'{pygame.key.name(pos).upper()} / {pygame.key.name(neg).upper()}'
            if name == 'HEAVE' and ui.hold:
                self.text(f'{name:6s}  hold', 16, y)
                self.bar(140, y + 2, 0.0)
                self.text(f'{keys} moves target', 380, y, DIM)
            else:
                self.text(f'{name:6s} {v:+5.2f}', 16, y)
                self.bar(140, y + 2, v)
                self.text(keys, 380, y, DIM)
            y += 24
        y += 2
        self.line(y)

        # 按鍵說明
        y += 10
        for row in ('1 MANUAL   2 STABILIZE   3 DEPTH_HOLD',
                    'ENTER arm   SPACE disarm   Z zero (disarmed only)',
                    'H depth hold on/off   - / = gain   Ctrl+Q quit'):
            self.text(row, 16, y, DIM)
            y += 22
        y += 4
        self.line(y)
        y += 10
        if not ui.started:
            self.text('> waiting for mavros and adapter ... (nothing is sent)', 16, y, YELLOW)
        else:
            self.text('> ' + t.message, 16, y)

        # 失去焦點
        if not ui.focused or ui.blocked:
            band = pygame.Rect(0, 190, 640, 60)
            pygame.draw.rect(s, RED, band)
            msg = ('FOCUS LOST - all axes zeroed, click to resume' if not ui.focused
                   else 'release all movement keys to resume')
            img = self.big.render(msg, True, FG)
            s.blit(img, img.get_rect(center=band.center))
        pygame.display.flip()


class UI:
    """操作狀態：要發出去的指令，以及主迴圈的旗標。"""

    def __init__(self, gain):
        self.gain = gain
        self.mode = M.MODE_MANUAL
        self.arm = M.ARM_NO_CHANGE
        self.hold = False           # use_depth_target
        self.target = 0.0           # target_depth
        self.efforts = [0.0] * len(AXES)
        self.started = False        # 開始發送了沒（mavros、adapter 都上線後）
        self.focused = True
        self.blocked = False        # 焦點回來後，等所有移動鍵放開


def on_signal(signum, frame):
    raise KeyboardInterrupt


def handle_key(e, t, ui, max_depth):
    """單次動作（KEYDOWN）。回傳 False 表示要結束。"""
    k = e.key
    if k == pygame.K_q and e.mod & pygame.KMOD_CTRL:
        return False
    if k in MODE_KEYS:
        ui.mode = MODE_KEYS[k]
        t.say(f'mode -> {MODE_FC[ui.mode]}')
    elif k in (pygame.K_RETURN, pygame.K_KP_ENTER):
        ui.arm = M.ARM
        t.say('arm requested')
    elif k == pygame.K_SPACE:
        ui.arm = M.DISARM
        t.say('disarm requested')
    elif k == pygame.K_z:
        if t.armed():
            t.say('zero ignored: disarm first')
            return True
        if ui.hold:
            ui.hold = False
            t.say('depth hold OFF (re-zeroing)')
        t.zero()
    elif k == pygame.K_h:
        if ui.hold:
            ui.hold = False
            t.say('depth hold OFF')
        elif t.depth() is None:
            t.say('depth hold needs zero first (Z while disarmed)')
        else:
            ui.hold = True
            ui.target = min(max(t.depth(), 0.0), max_depth)
            t.say(f'depth hold ON, target {ui.target:.3f} m')
    elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
        ui.gain = round(max(GAIN_MIN, ui.gain - GAIN_STEP), 2)
    elif k in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
        ui.gain = round(min(GAIN_MAX, ui.gain + GAIN_STEP), 2)
    return True


def step(t, ui, max_depth):
    """主迴圈的一格：讀按鍵、算 effort、發一筆。回傳 False 表示要結束。"""
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            return False
        if e.type == pygame.KEYDOWN and not handle_key(e, t, ui, max_depth):
            return False

    keys = pygame.key.get_pressed()
    ui.focused = pygame.key.get_focused()
    if not ui.focused:
        ui.blocked = True
    elif ui.blocked and not any(keys[k] for k in MOVE_KEYS):
        ui.blocked = False
    moving = ui.focused and not ui.blocked
    ui.efforts = [ui.gain * (keys[pos] - keys[neg]) if moving else 0.0 for _, pos, neg in AXES]
    if ui.hold:
        # R/F 改成移動目標深度：R（上）讓目標變淺
        ui.target -= ui.efforts[2] / ui.gain * TARGET_RATE / RATE_HZ
        ui.target = min(max(ui.target, 0.0), max_depth)
        ui.efforts[2] = 0.0

    if not ui.started:
        if not (t.connected() and t.adapter_alive()):
            return True
        # 沿用飛控目前的模式，啟動 teleop 本身不切模式
        ui.mode = FC_MODE.get(t.state.mode, M.MODE_MANUAL)
        ui.started = True
        t.say(f'connected, mode {MODE_FC[ui.mode]}')

    surge, sway, heave, yaw = ui.efforts
    t.pub.publish(AuvControl(
        control_source=M.SW_MANUAL, mode=ui.mode, arm_command=ui.arm,
        surge=surge, sway=sway, heave=heave, yaw=yaw,
        target_depth=ui.target, use_depth_target=ui.hold))
    ui.arm = M.ARM_NO_CHANGE    # ARM／DISARM 是事件，只放進這一筆
    return True


def finish(t):
    """MANUAL、全 0、DISARM，等飛控確認上鎖。"""
    print('結束：送 MANUAL、全 0、DISARM')
    end = time.monotonic() + 5.0
    first = True
    while time.monotonic() < end:
        t.pub.publish(AuvControl(control_source=M.SW_MANUAL, mode=M.MODE_MANUAL,
                                 arm_command=M.DISARM if first else M.ARM_NO_CHANGE))
        first = False
        if not t.armed() and time.monotonic() > end - 4.5:   # 至少送 0.5 秒
            print('飛控已上鎖')
            return
        time.sleep(1.0 / RATE_HZ)
    print('飛控沒有上鎖！用 QGC 或 kill switch 處理')


def main():
    ap = argparse.ArgumentParser(description='Orca keyboard teleop')
    ap.add_argument('--gain', type=float, default=0.3, help='initial thrust gain (0.1 ~ 1.0)')
    ap.add_argument('--max-depth', type=float, default=0.25,
                    help='upper limit of depth-hold target [m] (test pool is 0.25 m)')
    args = ap.parse_args()
    gain = round(min(max(args.gain, GAIN_MIN), GAIN_MAX), 2)

    # Ctrl-Z 會把程式停在背景，最後一筆指令被沿用到看門狗介入；關終端機（SIGHUP）、docker stop（SIGTERM）
    # 都跟 Ctrl-C 一樣走收尾
    for sig in (signal.SIGTSTP, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    # 不讓 rclpy 處理 Ctrl-C：它預設會先關掉 ROS，結束時的 DISARM 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    t = Teleop()
    executor = SingleThreadedExecutor()
    executor.add_node(t)
    threading.Thread(target=executor.spin, daemon=True).start()

    # 只初始化畫面和字型：pygame.init() 會去開音效裝置，容器裡沒有
    pygame.display.init()
    pygame.font.init()
    try:
        screen = Screen()
    except pygame.error as e:
        print(f'開不了視窗：{e}\n主機先跑 xhost +SI:localuser:root，並確認容器有 DISPLAY')
        rclpy.shutdown()
        return

    ui = UI(gain)
    clock = pygame.time.Clock()
    try:
        while step(t, ui, args.max_depth):
            screen.draw(t, ui)
            clock.tick(RATE_HZ)
    except KeyboardInterrupt:
        print('\n中斷')
    finally:
        pygame.display.quit()
        if ui.started:
            finish(t)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
