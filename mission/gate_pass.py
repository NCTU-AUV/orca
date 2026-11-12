#!/usr/bin/env python3
"""過門流程（控制來源：自主程式）。

流程：歸零（浮在水面）→ 切 DEPTH_HOLD、解鎖
  1. DIVE      下潛到 depth（預設 0.30 m），航向鎖在歸零時的 0°，等深度穩定
  2. ALIGN     原地轉 yaw，讓門框中心落在畫面水平中央
  3. APPROACH  把對準當下的 yaw 鎖成目標航向，往前走；
               之後不再轉向，門的左右偏移只用 sway 修正
  4. PASS      門框佔畫面夠寬，或近距離跟丟（門超出視野）→ 鎖 yaw 直走 pass_s 秒
  5. STOP      停 stop_s 秒 → 上鎖（浮上來）
任何一步逾時、或遠距離跟丟等不回來：中止，上鎖。

用法（bridge 容器內，cam_bridge、detector、bridge 都要在跑）：
    python3 /orca/mission/gate_pass.py [--depth 0.30] [--surge 0.2] [--pass-s 8] [--classes gate,qualification_gate]
  其餘參數改下面的 Params。任何時候 Ctrl-C：上鎖，船浮上來。

介面（見 runner.py）：控制只發 /auv_control；深度、航向跟 auv_adapter 讀同樣的 mavros topic
（只讀），零點跟 adapter 同時記，所以讀到的 yaw 可以直接當 target_yaw 送回去；
門的位置來自 /detections。這個檔案只有流程本身，不碰 ROS，可以單獨測（test_gate_pass.py）。

要在水裡調的：
- pass_s：水平方向沒有定位，「過了沒」只能用時間估。先量 surge 推力下的速度，
  讓 pass_s 夠走完「開始盲走時到門的距離 + 船身 + 餘裕」。
- near_width、lost_near_width：偵測在多近會開始掉，看 CSV 的 gate_width 欄決定。
- depth：水池測試腳本的上限是 0.25 m（那個測試池的深度）；這裡預設 0.30 m，池子要夠深。
"""
import argparse
import math
from dataclasses import dataclass


@dataclass
class Params:
    # ----- DIVE -----
    depth: float = 0.30             # 過門深度 [m]，往下為正，0 = 歸零時（浮著）
    depth_tol: float = 0.03         # 深度誤差在這個範圍內……
    depth_stable_s: float = 2.0     # ……持續這麼久算到達
    dive_timeout: float = 40.0      # 定深約 10～15 秒收斂；超過這麼久沒到就中止

    # ----- ALIGN -----
    yaw_gain: float = 0.5           # 每筆新偵測：目標航向 = 目前航向 + yaw_gain × 門的方位角
    yaw_step_max: float = 15.0      # 目標航向最多領先目前航向幾度（限制轉速）
    align_tol: float = 3.0          # 方位角在 ± 這麼多度內……
    align_stable_s: float = 1.5     # ……持續這麼久算對準
    find_timeout: float = 20.0      # 這麼久沒看到門就中止（不會自己轉頭找）
    align_timeout: float = 40.0     # 超過這麼久還沒對準：看得到門就照樣往下走，看不到就中止

    # ----- APPROACH -----
    surge: float = 0.2              # 前進推力（-1 ~ 1）
    sway_gain: float = 0.8          # sway = sway_gain × 門中心的水平偏移（畫面中央 0、右緣 +1）
    sway_max: float = 0.3           # sway 上限
    sway_deadband: float = 0.03     # 偏移小於這個不修
    forward_offset_max: float = 0.25    # 偏移超過這個先停止前進，只平移
    near_width: float = 0.7         # 門框寬 / 畫面寬超過這個 → PASS
    lost_s: float = 1.0             # 這麼久沒看到門算跟丟
    lost_near_width: float = 0.4    # 跟丟前門框寬度已到這個 → 當作門超出視野，PASS；否則停下來等
    reacquire_s: float = 5.0        # 遠距離跟丟後停下來等這麼久，還是沒有就中止

    # ----- PASS、STOP -----
    pass_s: float = 8.0             # 盲走幾秒
    stop_s: float = 2.0             # 過門後停幾秒再上鎖

    # ----- 共用 -----
    det_fresh_s: float = 0.5        # 最後一筆門的偵測超過這麼久，就當作現在沒看到
    state_timeout: float = 2.0      # 深度或航向是 NaN（飛控沒送資料來）超過這麼久就中止
    mission_timeout: float = 180.0  # 整個流程的上限


@dataclass
class Gate:
    """一筆門的偵測（runner.py 從 /detections 和 camera_info 換算）。"""
    seq: int        # 每筆新的偵測 +1
    t: float        # 收到的時間，跟 step() 的 now 同一個時鐘
    bearing: float  # 門中心的方位角 [deg]，在右邊為正。用相機在空氣中的內參換算，
                    # 平面視窗在水下會把角度放大約 1.33 倍；閉迴路會收斂，只是等效增益變小
    offset: float   # 門中心的水平偏移：畫面中央 0、右緣 +1、左緣 -1
    width: float    # 門框寬 / 畫面寬
    score: float = 0.0
    label: str = ''


@dataclass
class Cmd:
    """流程的輸出。模式固定 DEPTH_HOLD，use_depth_target、use_yaw_target 都是 true：
    全程用目標航向，不切回 yaw 轉速（切回去的 5 秒內飛控對 yaw 沒反應，見 AuvControl.msg）。"""
    surge: float
    sway: float
    target_depth: float
    target_yaw: float


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def clamp(v, limit):
    return max(-limit, min(limit, v))


class GatePass:
    def __init__(self, params=None, log=print):
        self.p = params or Params()
        self.log = log
        self.phase = 'DIVE'
        self.result = None          # None = 進行中，True = 走完，False = 中止
        self.message = ''
        self.target_yaw = 0.0
        self.t0 = None              # 流程開始
        self.phase_t0 = None        # 目前這一步開始
        self.since = None           # 目前這一步的條件（深度到達、已對準）從什麼時候開始成立
        self.bad_since = None       # 深度或航向從什麼時候開始是 NaN
        self.last_seq = None        # 已經用來轉向的那筆偵測
        self.seen_t = None          # 最後一次看到門
        self.seen_width = 0.0       # 那時的門框寬度

    def first_cmd(self):
        """解鎖前就開始送的指令：目標深度、航向 0。"""
        return Cmd(0.0, 0.0, self.p.depth, 0.0)

    # ---------- 每一輪 ----------

    def step(self, now, depth, yaw, gate):
        """depth [m]、yaw [deg] 相對零點（沒資料時是 NaN）；gate 是最新的 Gate 或 None。"""
        p = self.p
        if self.t0 is None:
            self.t0 = now
            self.enter('DIVE', now, f'下潛到 {p.depth:.2f} m、鎖航向 0°')
        stop = Cmd(0.0, 0.0, p.depth, self.target_yaw)
        if self.result is not None:
            return stop
        if now - self.t0 > p.mission_timeout:
            return self.fail(f'整個流程超過 {p.mission_timeout:.0f} 秒', stop)
        if not (math.isfinite(depth) and math.isfinite(yaw)):
            self.bad_since = now if self.bad_since is None else self.bad_since
            if now - self.bad_since > p.state_timeout:
                return self.fail('收不到飛控的深度或航向', stop)
            return stop
        self.bad_since = None

        visible = gate is not None and now - gate.t <= p.det_fresh_s
        if visible:
            self.seen_t, self.seen_width = gate.t, gate.width
        return getattr(self, 'do_' + self.phase.lower())(now, depth, yaw, gate if visible else None, stop)

    def do_dive(self, now, depth, yaw, gate, stop):
        p = self.p
        if self.held(abs(depth - p.depth) <= p.depth_tol, now, p.depth_stable_s):
            self.seen_t = now       # find_timeout 從這裡起算
            self.enter('ALIGN', now, '轉 yaw 對準門')
        elif now - self.phase_t0 > p.dive_timeout:
            return self.fail(f'{p.dive_timeout:.0f} 秒內沒到 {p.depth:.2f} m（目前 {depth:.2f} m）', stop)
        return stop

    def do_align(self, now, depth, yaw, gate, stop):
        p = self.p
        timed_out = now - self.phase_t0 > p.align_timeout
        if gate is None:
            self.since = None
            if timed_out or now - self.seen_t > p.find_timeout:
                return self.fail('沒看到門', stop)
            return stop
        # 每筆偵測只用一次：同一筆配上更新後的航向重算，目標會越推越遠
        if gate.seq != self.last_seq:
            self.last_seq = gate.seq
            self.target_yaw = wrap(yaw + clamp(p.yaw_gain * gate.bearing, p.yaw_step_max))
        aligned = self.held(abs(gate.bearing) <= p.align_tol, now, p.align_stable_s)
        if aligned or timed_out:
            if not aligned:
                self.log(f'{p.align_timeout:.0f} 秒內沒對準（方位角 {gate.bearing:+.1f}°），照樣往前')
            self.target_yaw = yaw       # 鎖住現在的航向，之後不再改
            self.enter('APPROACH', now, f'鎖航向 {yaw:+.1f}°，前進，左右只用 sway')
        return Cmd(0.0, 0.0, p.depth, self.target_yaw)

    def do_approach(self, now, depth, yaw, gate, stop):
        p = self.p
        if gate is None:
            lost = now - max(self.seen_t, self.phase_t0)
            if lost < p.lost_s:                     # 偵測偶爾掉幾幀：直走，不平移
                return Cmd(p.surge, 0.0, p.depth, self.target_yaw)
            if self.seen_width >= p.lost_near_width:
                self.enter('PASS', now, f'近距離跟丟（最後門框寬 {self.seen_width:.2f}），'
                                        f'直走 {p.pass_s:.0f} 秒')
                return Cmd(p.surge, 0.0, p.depth, self.target_yaw)
            if lost > p.lost_s + p.reacquire_s:
                return self.fail(f'遠距離跟丟（最後門框寬 {self.seen_width:.2f}）', stop)
            return stop                             # 停下來等門再出現
        if gate.width >= p.near_width:
            self.enter('PASS', now, f'門框寬 {gate.width:.2f}，直走 {p.pass_s:.0f} 秒')
            return Cmd(p.surge, 0.0, p.depth, self.target_yaw)
        sway = 0.0 if abs(gate.offset) < p.sway_deadband else clamp(p.sway_gain * gate.offset, p.sway_max)
        surge = p.surge if abs(gate.offset) <= p.forward_offset_max else 0.0
        return Cmd(surge, sway, p.depth, self.target_yaw)

    def do_pass(self, now, depth, yaw, gate, stop):
        if now - self.phase_t0 >= self.p.pass_s:
            self.enter('STOP', now, '停下')
            return stop
        return Cmd(self.p.surge, 0.0, self.p.depth, self.target_yaw)

    def do_stop(self, now, depth, yaw, gate, stop):
        if now - self.phase_t0 >= self.p.stop_s:
            self.result, self.message = True, '流程走完'
        return stop

    # ---------- 小工具 ----------

    def held(self, ok, now, seconds):
        """條件 ok 連續成立滿 seconds 秒才回傳 True。"""
        if not ok:
            self.since = None
            return False
        self.since = now if self.since is None else self.since
        return now - self.since >= seconds

    def enter(self, phase, now, text):
        self.phase, self.phase_t0, self.since = phase, now, None
        self.log(f'--- {phase}：{text}')

    def fail(self, message, stop):
        self.result, self.message = False, '中止：' + message
        return stop


def main():
    d = Params()
    ap = argparse.ArgumentParser(description='過門流程')
    ap.add_argument('--depth', type=float, default=d.depth, help='過門深度 [m]')
    ap.add_argument('--surge', type=float, default=d.surge, help='前進推力（0 ~ 1）')
    ap.add_argument('--pass-s', type=float, default=d.pass_s, help='盲走幾秒')
    ap.add_argument('--classes', default='gate,qualification_gate',
                    help='當作門的偵測類別，逗號分隔；同一幀有多個時取分數最高的')
    ap.add_argument('--min-score', type=float, default=0.5, help='低於這個分數的偵測不用')
    args = ap.parse_args()
    if not (0.0 < args.depth and 0.0 < args.surge <= 1.0 and args.pass_s >= 0.0):
        ap.error('depth 要大於 0，surge 要在 0 ~ 1，pass-s 不能是負的')

    from runner import run      # 這裡才 import：上面的流程不需要 ROS
    flow = GatePass(Params(depth=args.depth, surge=args.surge, pass_s=args.pass_s))
    run('gate_pass', flow, classes=args.classes.split(','), min_score=args.min_score)


if __name__ == '__main__':
    main()
