"""載具狀態（只讀）與送給 /auv_control 的指令。這個檔案不 import ROS，可以單獨測。

單位與方向（跟 orca_msgs/AuvControl、mission/runner.py 相同）：
  深度：歸零時為 0，往下為正 [m]
  航向：歸零時為 0，順時針為正 [deg]，範圍 [-180, 180)

狀態來源是 mavros 的 topic（/mavros/state、/mavros/global_position/rel_alt、/mavros/imu/data），
跟 auv_adapter 讀的一樣。adapter 不對外發布零點，所以在呼叫 /auv_adapter/zero 的同一刻
自己記一份（zero_here()），之後算出來的深度、航向跟 target_depth、target_yaw 是同一個座標。

遙測經 mavproxy 約 4 Hz：heading_at() 用航向歷史內插出「某個時間點」的航向，
給影像換算絕對方位用；轉向中誤差會變大（見 perception.py）。
"""
import math
from collections import deque
from dataclasses import dataclass, replace

STALE_S = 1.0           # 深度、航向超過這麼久沒更新就當作沒有


def wrap(deg):
    """換算到 [-180, 180)。"""
    return (deg + 180.0) % 360.0 - 180.0


def clamp(v, limit):
    if not math.isfinite(v):
        return 0.0
    return max(-limit, min(limit, v))


@dataclass(frozen=True)
class Command:
    """執行器的輸出。模式固定 DEPTH_HOLD，use_depth_target、use_yaw_target 全程 true：
    不切回 yaw 轉速（切回去的 5 秒內飛控對 yaw 沒反應，見 AuvControl.msg）。"""
    surge: float = 0.0
    sway: float = 0.0
    target_depth: float = 0.0
    target_yaw: float = 0.0

    def stopped(self):
        """同樣的目標深度、航向，推力歸 0。"""
        return replace(self, surge=0.0, sway=0.0)


@dataclass(frozen=True)
class Limits:
    """所有指令最後都經過這裡（Shaper）。"""
    max_surge: float = 0.5
    max_sway: float = 0.5
    max_depth: float = 3.0          # 目標深度上限 [m]
    yaw_rate: float = 30.0          # 目標航向每秒最多變多少度（ArduSub 上限 60°/s）


class Shaper:
    """限幅，並限制目標航向的變化速度。

    目標航向每一步最多走 yaw_rate × dt，方向取最短的一邊。大角度轉向因此會變成一連串
    小步，不會出現「相鄰兩筆目標差 180°、飛控不知道往哪轉」的情形。
    """

    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.last_yaw = None

    def reset(self, yaw):
        self.last_yaw = wrap(yaw)

    def __call__(self, cmd, dt):
        lim = self.limits
        want = wrap(cmd.target_yaw) if math.isfinite(cmd.target_yaw) else self.last_yaw
        if want is None:
            want = 0.0
        if self.last_yaw is None:
            yaw = want
        else:
            step = lim.yaw_rate * max(dt, 0.0)
            yaw = wrap(self.last_yaw + clamp(wrap(want - self.last_yaw), step))
        self.last_yaw = yaw
        depth = cmd.target_depth if math.isfinite(cmd.target_depth) else 0.0
        return Command(surge=clamp(cmd.surge, lim.max_surge),
                       sway=clamp(cmd.sway, lim.max_sway),
                       target_depth=max(0.0, min(lim.max_depth, depth)),
                       target_yaw=yaw)


class VehicleState:
    """把 mavros 的資料換算成跟 AuvControl 同一個座標。時間一律用呼叫端給的 t（秒）。"""

    def __init__(self, history_s=5.0):
        self.connected = False
        self.armed = False
        self.mode = ''
        self.fc_depth = None        # 飛控座標，往下為正
        self.yaw_enu = None         # rad，ROS 慣例：逆時針為正
        self.depth_t = -math.inf
        self.yaw_t = -math.inf
        self.state_t = -math.inf
        self.adapter_t = -math.inf  # 最後一次看到 adapter 送 MANUAL_CONTROL
        self.surface = None         # 歸零時的 fc_depth
        self.yaw0 = None            # 歸零時的 yaw_enu
        self.history = deque()      # (t, 航向 deg)，歸零後才有
        self.history_s = history_s

    # ---------- 輸入（ROS callback 呼叫） ----------

    def on_state(self, t, connected, armed, mode):
        self.connected, self.armed, self.mode, self.state_t = connected, armed, mode, t

    def on_rel_alt(self, t, rel_alt):
        self.fc_depth = -rel_alt        # rel_alt 向上為正
        self.depth_t = t

    def on_orientation(self, t, x, y, z, w):
        self.yaw_enu = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.yaw_t = t
        if self.yaw0 is not None:
            self.history.append((t, self._heading()))
            while self.history and t - self.history[0][0] > self.history_s:
                self.history.popleft()

    def on_adapter(self, t):
        self.adapter_t = t

    def zero_here(self):
        """跟 adapter 同時歸零（呼叫 /auv_adapter/zero 成功之後馬上呼叫）。"""
        self.surface, self.yaw0 = self.fc_depth, self.yaw_enu
        self.history.clear()
        if self.yaw0 is not None:
            self.history.append((self.yaw_t, 0.0))

    # ---------- 查詢 ----------

    @property
    def zeroed(self):
        return self.surface is not None and self.yaw0 is not None

    def _heading(self):
        return wrap(-math.degrees(self.yaw_enu - self.yaw0))

    def depth(self, now):
        if self.surface is None or self.fc_depth is None or now - self.depth_t > STALE_S:
            return math.nan
        return self.fc_depth - self.surface

    def heading(self, now):
        if self.yaw0 is None or self.yaw_enu is None or now - self.yaw_t > STALE_S:
            return math.nan
        return self._heading()

    def heading_at(self, t):
        """t 時刻的航向，用歷史內插。比最新一筆新（遙測還沒到）就用最新一筆，比最舊的還舊回傳 NaN。"""
        h = self.history
        if not h or t < h[0][0]:
            return math.nan
        if t >= h[-1][0]:
            return h[-1][1] if t - h[-1][0] <= STALE_S else math.nan
        for (t0, y0), (t1, y1) in zip(h, list(h)[1:]):
            if t0 <= t <= t1:
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return wrap(y0 + a * wrap(y1 - y0))
        return math.nan

    def adapter_alive(self, now):
        return now - self.adapter_t < 0.5


class SurfaceWatch:
    """收尾時判斷船浮上水面了沒：深度（相對歸零時的水面）≤ depth 持續 hold_s 秒。
    深度不明（NaN）或又沉下去就重新計時。"""

    def __init__(self, depth=0.05, hold_s=1.0):
        self.depth = depth
        self.hold_s = hold_s
        self.since = None

    def update(self, now, depth):
        if not (math.isfinite(depth) and depth <= self.depth):
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold_s
