"""投球用的執行器：用前視相機接近 drum（ApproachTarget）、用底部相機停在 drum 正上方（CenterOver）。

行為樹（trees/drop_ball.yaml）：
  Dive → AlignTo（前視，轉向對準 drum）→ ApproachTarget → CenterOver → DropBall（任務層節點）→ Hold

底部相機的影像方向（config/orca.yaml）：上方 = 船頭、右方 = 右舷。所以
  目標在畫面上方（offset_y < 0）→ 在船的前方 → surge 往前；目標在畫面右邊 → sway 往右。
"""
import dataclasses
import math

from ..vehicle import Command, clamp, wrap
from .base import Task


def _labels(v):
    return [v] if isinstance(v, str) else list(v)


class ApproachTarget(Task):
    """鎖住目前的目標航向往前走，前視相機裡目標的左右偏移用 sway 修，直到底部相機看到目標。

    靠近時目標會從前視畫面的下緣出去、還沒進底部相機的視野（中間有一段兩台都看不到），
    所以前視看不到之後照樣直走，最多 blind_s 秒；這段時間底部相機看到就成功，沒看到就失敗。
    一開始前視就沒看到：停住等，find_timeout 秒都沒看到就失敗。

    左右修正看目標的「絕對方位相對鎖住的航向」（abs_bearing − 目標航向），不看畫面上的 offset_x：
    船頭被推歪時目標在畫面上會跑掉，但絕對方位不變，sway 不會跟著亂修。2026-11-26 模擬：
    surge 從 0 跳到 0.15 的瞬間船頭被推偏約 3.4°（飛控拉回來很慢，小誤差落在推進器死區），
    用 offset_x 時 sway 把這當成左右偏移、往反方向修，看起來就是往前的瞬間左右晃一下。
    surge 也改成 surge_ramp_s 秒內慢慢加上去，減少這一下的偏航。
    sway 至少 sway_min（約 2 N），小於這個 T200 不出力。
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
        surge_ramp_s: float = 2.0       # surge 從 0 加到 surge 花幾秒
        sway_gain: float = 0.06         # sway = sway_gain × 方位差 [deg]（5° → 0.3）
        sway_min: float = 0.1
        sway_max: float = 0.3
        sway_deadband: float = 1.0      # 方位差 [deg] 小於這個不修
        sway_hold_s: float = 0.5        # 方位差超過 deadband 持續這麼久才修（遙測的航向比影像晚到，船頭剛被推歪時
                                        # 絕對方位會短暫算錯）
        forward_bearing_max: float = 10.0   # 方位差超過這個 [deg] 先停下來平移，不往前
        blind_s: float = 10.0
        find_timeout: float = 10.0

    def on_start(self, snap):
        self.yaw = self.hold.target_yaw
        self.seen_t = None              # 前視最後一次看到

    def step(self, snap):
        p = self.p
        depth = self.hold.target_depth
        stop = Command(0.0, 0.0, depth, self.yaw)
        surge = p.surge * min(1.0, self.elapsed(snap) / p.surge_ramp_s) if p.surge_ramp_s > 0 else p.surge
        ahead = Command(surge, 0.0, depth, self.yaw)
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
        # 目標相對鎖住航向的方位：右邊為正。拍攝當下的航向不知道時退回只用畫面上的方位角
        rel = wrap(s.abs_bearing - self.yaw) if math.isfinite(s.abs_bearing) else s.bearing
        self.feedback = f'{s.label} 方位差 {rel:+.1f}°（{s.score:.2f}）'
        sway = 0.0
        if self.held(abs(rel) >= p.sway_deadband, snap.now, p.sway_hold_s):
            sway = math.copysign(min(max(p.sway_gain * abs(rel), p.sway_min), p.sway_max), rel)
        if abs(rel) > p.forward_bearing_max:
            surge = 0.0
        return Command(surge, sway, depth, self.yaw)


class CenterOver(Task):
    """航向、深度鎖住，用 surge、sway 平移，讓目標停在底部相機畫面的 (aim_x, aim_y)。

    aim_x、aim_y：電磁鐵正下方在底部相機畫面裡的位置（offset 的單位：畫面中央 0、右緣／下緣 +1）。
    電磁鐵跟相機不在同一點時，用它補偏移；裝好之後在岸上量（球垂下去的位置在畫面哪裡）。
    誤差 = drum 中心到 aim 的距離，用「幾個 drum 直徑」算（bbox 的寬、高就是一個直徑），
    跟相機視角、深度都無關（底部相機 120° 時，畫面比例 0.1 在 1.2 m 高約 21 cm，不能直接當門檻）。
    誤差 ≤ tol 持續 stable_s 秒算對準，成功。例：tol 0.08、drum 直徑 60 cm → 中心偏 5 cm 以內。
    推力照畫面上的偏移算（gain × offset），但不小於 min_effort：小推力落在 T200 的死區裡，
    船推不動（docs/sim/notes.md「CAD 船體、推力死區」；2026-11-26 模擬：surge 0.03 時停在 7 cm 不動）。
    哪一軸的誤差已經 ≤ tol / 2 就不推那一軸。
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
        min_effort: float = 0.1         # 要推就至少這麼多（約 2 N），低於這個 T200 不出力
        deadband: float = 0.02          # 看不到時的移動：最後位置離畫面中央這麼近就不移
        tol: float = 0.08               # 對準門檻 [drum 直徑]
        target_size: float = 0.6        # drum 直徑 [m]，只用來把誤差換成公分給人看
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

    def effort(self, offset, err_d):
        """一軸的推力：offset 是畫面上的偏移（決定方向、大小），err_d 是這軸的誤差 [drum 直徑]。"""
        p = self.p
        if abs(err_d) <= p.tol / 2:
            return 0.0
        return math.copysign(min(max(p.gain * abs(offset), p.min_effort), p.max_effort), offset)

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
            if n < p.deadband:
                return stop
            self.feedback += '，往最後看到的方向移'
            return Command(-p.seek_effort * ly / n, p.seek_effort * lx / n, depth, self.yaw)
        self.seen_t = snap.now
        ex, ey = s.offset_x - p.aim_x, s.offset_y - p.aim_y
        self.last_err = (s.offset_x, s.offset_y)
        # 畫面上一個 drum 直徑 = offset 單位裡的 2 × width（x）、2 × height（y）
        dx = ex / (2.0 * s.width) if s.width > 0 else math.inf
        dy = ey / (2.0 * s.height) if s.height > 0 else math.inf
        err = math.hypot(dx, dy)
        cm = err * p.target_size * 100
        self.feedback = f'{s.label} 誤差 {err:.3f} 直徑（約 {cm:.0f} cm；x {dx:+.3f}、y {dy:+.3f}）'
        if self.held(err <= p.tol, snap.now, p.stable_s):
            return self.succeed(f'對準 {s.label}，誤差 {err:.3f} 直徑（約 {cm:.0f} cm）', stop)
        if self.elapsed(snap) > p.give_up_s:
            return self.fail(f'{p.give_up_s:g} 秒內沒對準（誤差約 {cm:.0f} cm）', stop)
        if err <= p.tol:
            return stop
        return Command(-self.effort(ey, dy), self.effort(ex, dx), depth, self.yaw)
