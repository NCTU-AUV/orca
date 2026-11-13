"""過門用的執行器：轉向對準（AlignTo）、平移對準（CenterGate）、接近（ApproachGate）。

AlignTo、ApproachGate 的邏輯跟 mission/gate_pass.py 的 ALIGN、APPROACH 相同，參數名稱與預設值也一樣。
行為樹（trees/gate_pass.yaml）目前的流程不轉向，航向從解鎖起就鎖住，對準只用 sway：
  Dive → CenterGate → ApproachGate → Cruise（盲走過門）→ Hold（停）
"""
import dataclasses
import math

from ..vehicle import Command, clamp, wrap
from .base import Task


def _labels(v):
    return [v] if isinstance(v, str) else list(v)


class AlignTo(Task):
    """原地轉 yaw，讓目標中心落在畫面水平中央。

    每一筆新的偵測：目標航向 = 拍攝當下的航向 + yaw_gain × 方位角（最多 yaw_step_max 度）。
    方位角在 ± tol 內持續 stable_s 秒算對準，把目前的航向鎖成目標航向後結束。
    find_timeout 秒都沒看到：失敗（不會自己轉頭找）。
    give_up_s 秒還沒對準：看得到就照樣算成功（往下走），看不到就失敗。
    """

    @dataclasses.dataclass
    class Params:
        labels: list = dataclasses.field(default_factory=lambda: ['gate', 'qualification_gate'])
        camera: str = 'front'
        min_score: float = 0.5
        max_age: float = 0.5            # 偵測超過這麼久就當作現在沒看到
        yaw_gain: float = 0.5
        yaw_step_max: float = 15.0
        tol: float = 3.0
        stable_s: float = 1.5
        find_timeout: float = 20.0
        give_up_s: float = 40.0

    def on_start(self, snap):
        self.target = self.hold.target_yaw
        self.last_frame = None
        self.seen_t = snap.now

    def step(self, snap):
        p = self.p
        s = snap.best(_labels(p.labels), p.camera, p.max_age, p.min_score)
        timed_out = self.elapsed(snap) > p.give_up_s
        if s is None:
            self.held(False, snap.now, 0)
            self.feedback = f'找 {"/".join(_labels(p.labels))}（{snap.now - self.seen_t:.1f} s 沒看到）'
            if timed_out or snap.now - self.seen_t > p.find_timeout:
                return self.fail('沒看到目標')
            return Command(0.0, 0.0, self.hold.target_depth, self.target)
        self.seen_t = snap.now
        # 每筆偵測只用一次：同一筆配上更新後的航向重算，目標會越推越遠
        if s.frame != self.last_frame:
            self.last_frame = s.frame
            base = s.heading if math.isfinite(s.heading) else snap.heading
            self.target = wrap(base + clamp(p.yaw_gain * s.bearing, p.yaw_step_max))
        self.feedback = f'{s.label} 方位 {s.bearing:+.1f}°（{s.score:.2f}）'
        aligned = self.held(abs(s.bearing) <= p.tol, snap.now, p.stable_s)
        if aligned or timed_out:
            lock = Command(0.0, 0.0, self.hold.target_depth, snap.heading)
            if aligned:
                return self.succeed(f'對準，鎖航向 {snap.heading:+.1f}°', lock)
            return self.succeed(f'{p.give_up_s:.0f} 秒內沒對準（方位 {s.bearing:+.1f}°），照樣往前', lock)
        return Command(0.0, 0.0, self.hold.target_depth, self.target)


class CenterGate(Task):
    """航向鎖住不動，只用 sway 左右平移，讓門的中心落在船頭正前方。

    對準的判斷用「門中心離畫面中央幾個門寬」：lateral = offset_x / (2 × 門框寬)。
    這個比例等於「船偏離門中線的距離 / 門寬」，跟距離、相機內參都無關
    （例：門寬 1.5 m、tol 0.1 → 船偏離門中線 ±15 cm 以內）。
    |lateral| ≤ tol 持續 stable_s 秒算對準，成功（下一步 ApproachGate 開始往前）。
    find_timeout 秒都沒看到：失敗。
    give_up_s 秒還沒對準：看得到就照樣算成功（往前時 ApproachGate 會繼續用 sway 修），看不到就失敗。
    """

    @dataclasses.dataclass
    class Params:
        labels: list = dataclasses.field(default_factory=lambda: ['gate', 'qualification_gate'])
        camera: str = 'front'
        min_score: float = 0.5
        max_age: float = 0.5
        sway_gain: float = 0.8          # sway = sway_gain × offset_x（跟 ApproachGate 相同）
        sway_max: float = 0.3
        tol: float = 0.1                # 對準門檻：|偏離| ≤ 門寬 × tol
        stable_s: float = 1.5
        find_timeout: float = 20.0
        give_up_s: float = 40.0

    def on_start(self, snap):
        self.yaw = self.hold.target_yaw
        self.seen_t = snap.now

    def step(self, snap):
        p = self.p
        depth = self.hold.target_depth
        stop = Command(0.0, 0.0, depth, self.yaw)
        s = snap.best(_labels(p.labels), p.camera, p.max_age, p.min_score)
        timed_out = self.elapsed(snap) > p.give_up_s
        if s is None:
            self.held(False, snap.now, 0)
            self.feedback = f'找 {"/".join(_labels(p.labels))}（{snap.now - self.seen_t:.1f} s 沒看到）'
            if timed_out or snap.now - self.seen_t > p.find_timeout:
                return self.fail('沒看到目標')
            return stop                                     # 看不到就停住，不亂平移
        self.seen_t = snap.now
        lateral = s.offset_x / (2.0 * s.width) if s.width > 0 else math.inf
        self.feedback = f'{s.label} 偏離 {lateral:+.2f} 門寬（offset {s.offset_x:+.2f} 寬 {s.width:.2f}）'
        if self.held(abs(lateral) <= p.tol, snap.now, p.stable_s):
            return self.succeed(f'對準，偏離 {lateral:+.2f} 門寬', stop)
        if timed_out:
            return self.succeed(f'{p.give_up_s:.0f} 秒內沒對準（偏離 {lateral:+.2f} 門寬），照樣往前', stop)
        sway = 0.0 if abs(lateral) <= p.tol else clamp(p.sway_gain * s.offset_x, p.sway_max)
        return Command(0.0, sway, depth, self.yaw)


class ApproachGate(Task):
    """鎖住目前的目標航向往前走，門的左右偏移只用 sway 修正。

    門框寬 / 畫面寬 ≥ near_width：成功（接下來盲走）。
    跟丟：lost_s 內照樣直走；跟丟前門框已經 ≥ lost_near_width 當作門超出視野，成功；
    否則停下來等，reacquire_s 秒還沒看到就失敗。
    """

    @dataclasses.dataclass
    class Params:
        labels: list = dataclasses.field(default_factory=lambda: ['gate', 'qualification_gate'])
        camera: str = 'front'
        min_score: float = 0.5
        max_age: float = 0.5
        surge: float = 0.2
        sway_gain: float = 0.8
        sway_max: float = 0.3
        sway_deadband: float = 0.03
        forward_offset_max: float = 0.25
        near_width: float = 0.7
        lost_s: float = 1.0
        lost_near_width: float = 0.4
        reacquire_s: float = 5.0

    def on_start(self, snap):
        self.yaw = self.hold.target_yaw
        self.seen_t = snap.now
        self.seen_width = 0.0

    def step(self, snap):
        p = self.p
        depth = self.hold.target_depth
        stop = Command(0.0, 0.0, depth, self.yaw)
        ahead = Command(p.surge, 0.0, depth, self.yaw)
        s = snap.best(_labels(p.labels), p.camera, p.max_age, p.min_score)
        if s is None:
            lost = snap.now - self.seen_t
            self.feedback = f'跟丟 {lost:.1f} s（最後門框寬 {self.seen_width:.2f}）'
            if lost < p.lost_s:                 # 偵測偶爾掉幾幀：直走，不平移
                return ahead
            if self.seen_width >= p.lost_near_width:
                return self.succeed(f'近距離跟丟（最後門框寬 {self.seen_width:.2f}）', ahead)
            if lost > p.lost_s + p.reacquire_s:
                return self.fail(f'遠距離跟丟（最後門框寬 {self.seen_width:.2f}）', stop)
            return stop                         # 停下來等門再出現
        self.seen_t, self.seen_width = snap.now, s.width
        self.feedback = f'{s.label} 偏移 {s.offset_x:+.2f} 寬 {s.width:.2f}（{s.score:.2f}）'
        if s.width >= p.near_width:
            return self.succeed(f'門框寬 {s.width:.2f}', ahead)
        sway = 0.0 if abs(s.offset_x) < p.sway_deadband else clamp(p.sway_gain * s.offset_x, p.sway_max)
        surge = p.surge if abs(s.offset_x) <= p.forward_offset_max else 0.0
        return Command(surge, sway, depth, self.yaw)
