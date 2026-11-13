"""基本動作（測試與組合用的 placeholder 執行器）。

都只靠飛控的定深、定向閉迴路：送目標深度、目標航向，加上開迴路的 surge／sway。
水平方向沒有定位（沒有 DVL、GPS），所以「走多遠」只能用時間估。
"""
import dataclasses
import math

from ..vehicle import Command, wrap
from .base import Task


class Hold(Task):
    """原地保持目前的深度與航向 seconds 秒。"""

    @dataclasses.dataclass
    class Params:
        seconds: float = 2.0

    def step(self, snap):
        cmd = self.hold.stopped()
        left = self.p.seconds - self.elapsed(snap)
        self.feedback = f'保持 {max(left, 0):.1f} s'
        if left <= 0:
            return self.succeed('時間到', cmd)
        return cmd


class Dive(Task):
    """改變目標深度，等深度誤差在 tol 內持續 stable_s 秒。航向維持目前的目標。"""

    @dataclasses.dataclass
    class Params:
        depth: float                # 目標深度 [m]，往下為正
        tol: float = 0.03
        stable_s: float = 2.0

    def step(self, snap):
        cmd = Command(0.0, 0.0, self.p.depth, self.hold.target_yaw)
        err = snap.depth - self.p.depth
        self.feedback = f'深度 {snap.depth:.2f} / {self.p.depth:.2f} m'
        if self.held(abs(err) <= self.p.tol, snap.now, self.p.stable_s):
            return self.succeed(f'到達 {self.p.depth:.2f} m', cmd)
        return cmd


class TurnTo(Task):
    """轉到目標航向，等航向誤差在 tol 內持續 stable_s 秒。深度維持目前的目標。

    yaw：relative 為 false 時是相對歸零時的航向；為 true 時是相對「目前的目標航向」。
    轉速由 Shaper 限制（Limits.yaw_rate），大角度會自動拆成小步。
    """

    @dataclasses.dataclass
    class Params:
        yaw: float
        relative: bool = False
        tol: float = 3.0
        stable_s: float = 1.0

    def on_start(self, snap):
        self.target = wrap(self.hold.target_yaw + self.p.yaw if self.p.relative else self.p.yaw)

    def step(self, snap):
        cmd = Command(0.0, 0.0, self.hold.target_depth, self.target)
        err = wrap(snap.heading - self.target) if math.isfinite(snap.heading) else math.nan
        self.feedback = f'航向 {snap.heading:+.1f} → {self.target:+.1f}°'
        if self.held(abs(err) <= self.p.tol, snap.now, self.p.stable_s):
            return self.succeed(f'航向 {self.target:+.1f}°', cmd)
        return cmd


class Cruise(Task):
    """以固定的 surge／sway 走 seconds 秒（開迴路），航向鎖住。

    yaw 不給時鎖在目前的目標航向。"""

    @dataclasses.dataclass
    class Params:
        seconds: float
        surge: float = 0.2
        sway: float = 0.0
        yaw: float = math.nan

    def step(self, snap):
        yaw = self.p.yaw if math.isfinite(self.p.yaw) else self.hold.target_yaw
        cmd = Command(self.p.surge, self.p.sway, self.hold.target_depth, yaw)
        left = self.p.seconds - self.elapsed(snap)
        self.feedback = f'surge {self.p.surge:+.2f} sway {self.p.sway:+.2f}，剩 {max(left, 0):.1f} s'
        if left <= 0:
            return self.succeed(f'走了 {self.p.seconds:.1f} s', cmd)
        return cmd
