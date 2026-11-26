"""投球用的執行器：用前視相機接近 drum（ApproachTarget）、用底部相機停在 drum 正上方（CenterOver）。

行為樹（trees/drop_ball.yaml）：
  Dive → AlignTo（前視，轉向對準 drum）→ ApproachTarget → CenterOver → DropBall（任務層節點）→ Hold

底部相機的影像方向（config/orca.yaml）：上方 = 船頭、右方 = 右舷。所以
  目標在畫面上方（offset_y < 0）→ 在船的前方 → surge 往前；目標在畫面右邊 → sway 往右。
"""
import dataclasses
import math

from ..vehicle import Command, clamp
from .base import Task


def _labels(v):
    return [v] if isinstance(v, str) else list(v)


class ApproachTarget(Task):
    """鎖住目前的目標航向往前走，前視相機裡目標的左右偏移用 sway 修，直到底部相機看到目標。

    靠近時目標會從前視畫面的下緣出去、還沒進底部相機的視野（中間有一段兩台都看不到），
    所以前視看不到之後照樣直走，最多 blind_s 秒；這段時間底部相機看到就成功，沒看到就失敗。
    一開始前視就沒看到：停住等，find_timeout 秒都沒看到就失敗。
    """

    @dataclasses.dataclass
    class Params:
        labels: list = dataclasses.field(default_factory=lambda: ['blue_drum'])
        camera: str = 'front'           # 接近時看的相機
        stop_camera: str = 'bottom'     # 這台看到目標就成功
        min_score: float = 0.5
        stop_min_score: float = 0.35    # stop_camera（俯視）的門檻，分數比前視低
        max_age: float = 0.5
        surge: float = 0.15
        sway_gain: float = 0.8
        sway_max: float = 0.3
        sway_deadband: float = 0.03
        forward_offset_max: float = 0.25    # 左右偏太多（|offset_x| 超過）先停下來平移，不往前
        blind_s: float = 10.0
        find_timeout: float = 10.0

    def on_start(self, snap):
        self.yaw = self.hold.target_yaw
        self.seen_t = None              # 前視最後一次看到

    def step(self, snap):
        p = self.p
        depth = self.hold.target_depth
        stop = Command(0.0, 0.0, depth, self.yaw)
        ahead = Command(p.surge, 0.0, depth, self.yaw)
        labels = _labels(p.labels)
        below = snap.best(labels, p.stop_camera, p.max_age, p.stop_min_score)
        if below is not None:
            return self.succeed(f'{p.stop_camera} 看到 {below.label}（offset {below.offset_x:+.2f}, '
                                f'{below.offset_y:+.2f}）', stop)
        s = snap.best(labels, p.camera, p.max_age, p.min_score)
        if s is None:
            if self.seen_t is None:
                self.feedback = f'找 {"/".join(labels)}（{self.elapsed(snap):.1f} s 沒看到）'
                if self.elapsed(snap) > p.find_timeout:
                    return self.fail('前視沒看到目標')
                return stop
            blind = snap.now - self.seen_t
            self.feedback = f'前視看不到 {blind:.1f} s，直走等 {p.stop_camera} 看到'
            if blind > p.blind_s:
                return self.fail(f'前視看不到之後走了 {p.blind_s:g} 秒，{p.stop_camera} 還是沒看到', stop)
            return ahead
        self.seen_t = snap.now
        self.feedback = f'{s.label} 偏移 {s.offset_x:+.2f}、{s.offset_y:+.2f}（{s.score:.2f}）'
        sway = 0.0 if abs(s.offset_x) < p.sway_deadband else clamp(p.sway_gain * s.offset_x, p.sway_max)
        surge = p.surge if abs(s.offset_x) <= p.forward_offset_max else 0.0
        return Command(surge, sway, depth, self.yaw)


class CenterOver(Task):
    """航向、深度鎖住，用 surge、sway 平移，讓目標停在底部相機畫面的 (aim_x, aim_y)。

    aim_x、aim_y：電磁鐵正下方在底部相機畫面裡的位置（offset 的單位：畫面中央 0、右緣／下緣 +1）。
    電磁鐵跟相機不在同一點時，用它補偏移；裝好之後在岸上量（球垂下去的位置在畫面哪裡）。
    誤差（到 aim 的距離）≤ tol 持續 stable_s 秒算對準，成功。
    看不到目標時往最後看到的方向慢慢移（seek_effort）：畫面邊緣的偵測分數比較低、常常掉，停在原地
    會一直卡在邊緣（2026-11-26 模擬實測：邊緣 0.4～0.7，正下方 0.9 以上）。
    lost_timeout 秒都沒看到失敗。give_up_s 秒還沒對準也失敗（不在沒對準時放球）。
    """

    @dataclasses.dataclass
    class Params:
        labels: list = dataclasses.field(default_factory=lambda: ['blue_drum'])
        camera: str = 'bottom'
        min_score: float = 0.35         # 俯視的 drum 分數比前視低
        max_age: float = 0.5
        aim_x: float = 0.0
        aim_y: float = 0.0
        gain: float = 0.5               # surge／sway = gain × 誤差
        max_effort: float = 0.15
        deadband: float = 0.02
        tol: float = 0.1
        stable_s: float = 2.0
        seek_effort: float = 0.08       # 看不到時往最後看到的方向移的推力
        lost_timeout: float = 8.0
        give_up_s: float = 60.0

    def on_start(self, snap):
        self.yaw = self.hold.target_yaw
        self.seen_t = snap.now
        # 最後看到時的 (offset_x, offset_y)。開始時用前一步（ApproachTarget）剛看到的那筆：
        # 往往只有邊緣的一兩幀，CenterOver 自己不一定看得到
        recent = [s for s in (snap.last_seen.get(lbl) for lbl in _labels(self.p.labels))
                  if s is not None and s.camera == self.p.camera and snap.now - s.t < 2.0]
        s = max(recent, key=lambda s: s.t, default=None)
        self.last_err = (s.offset_x, s.offset_y) if s else None

    def drive(self, ex, ey, limit):
        surge = 0.0 if abs(ey) < self.p.deadband else clamp(-self.p.gain * ey, limit)
        sway = 0.0 if abs(ex) < self.p.deadband else clamp(self.p.gain * ex, limit)
        return surge, sway

    def step(self, snap):
        p = self.p
        depth = self.hold.target_depth
        stop = Command(0.0, 0.0, depth, self.yaw)
        s = snap.best(_labels(p.labels), p.camera, p.max_age, p.min_score)
        if s is None:
            self.held(False, snap.now, 0)
            lost = snap.now - self.seen_t
            self.feedback = f'{p.camera} 看不到 {"/".join(_labels(p.labels))}（{lost:.1f} s）'
            if lost > p.lost_timeout:
                return self.fail(f'{p.camera} 跟丟目標')
            if self.last_err is None:
                return stop
            # 往最後看到的方向（不是 aim 的方向：目標在畫面邊緣掉的，往它那邊移才會回到畫面裡）
            lx, ly = self.last_err
            n = math.hypot(lx, ly)
            if n < p.tol:
                return stop
            self.feedback += '，往最後看到的方向移'
            return Command(-p.seek_effort * ly / n, p.seek_effort * lx / n, depth, self.yaw)
        self.seen_t = snap.now
        ex, ey = s.offset_x - p.aim_x, s.offset_y - p.aim_y
        self.last_err = (s.offset_x, s.offset_y)
        err = math.hypot(ex, ey)
        self.feedback = f'{s.label} 誤差 {err:.2f}（x {ex:+.2f}、y {ey:+.2f}）'
        if self.held(err <= p.tol, snap.now, p.stable_s):
            return self.succeed(f'對準 {s.label}，誤差 {err:.2f}', stop)
        if self.elapsed(snap) > p.give_up_s:
            return self.fail(f'{p.give_up_s:g} 秒內沒對準（誤差 {err:.2f}）', stop)
        if err <= p.tol:
            return stop
        surge, sway = self.drive(ex, ey, p.max_effort)
        return Command(surge, sway, depth, self.yaw)
