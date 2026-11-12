#!/usr/bin/env python3
"""gate_pass.py 的流程測試：用一個很粗的平面模型代替船、飛控和相機，不需要 ROS。

    python3 mission/test_gate_pass.py        # 或 pytest mission/

模型只用來確認流程的邏輯和正負號（門在右邊 → 右轉／右移），不代表實機的動態：
- 航向：追目標航向，上限 60°/s；深度、前進、平移都是一階延遲
- 遙測：深度、航向 4 Hz（跟實機經 mavproxy 拿到的差不多）
- 相機：640 px 寬、fx 462（HFOV 約 69°），10 Hz、延遲 0.15 秒；太近（min_range）就偵測不到
座標：x 是歸零時的正前方，y 是右邊，航向順時針為正。
"""
import math

from gate_pass import Gate, GatePass, Params, wrap

DT = 0.05
W, FX, CX = 640, 462.0, 320.0
GATE_W = 1.5
SPEED = 1.5         # 推力 1.0 時的速度 [m/s]（surge 0.2 → 0.3 m/s）


class World:
    def __init__(self, gate_xy=(6.0, 0.0), yaw=0.0, min_range=1.0, max_range=20.0, has_gate=True):
        self.x = self.y = self.u = self.v = self.depth = 0.0
        self.yaw = yaw
        self.gate_xy = gate_xy
        self.min_range, self.max_range, self.has_gate = min_range, max_range, has_gate
        self.t = 0.0
        self.meas = (0.0, yaw)      # 4 Hz 的 (深度, 航向)
        self.frames = []            # 還在路上的偵測：(送達時間, Gate 或 None)
        self.gate = None
        self.seq = 0
        self.cross_y = None         # 通過門的平面時，相對門中心的橫向位置
        self.surged_blind = False   # 還沒看過門就前進過

    def see(self):
        """目前位置看到的門框 → Gate（還沒填 seq、t），看不到回傳 None。"""
        if not self.has_gate:
            return None
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        px = []
        for side in (-0.5, 0.5):
            dx, dy = self.gate_xy[0] - self.x, self.gate_xy[1] + side * GATE_W - self.y
            fwd, right = dx * c + dy * s, -dx * s + dy * c
            if not self.min_range <= fwd <= self.max_range:
                return None
            px.append(CX + FX * right / fwd)
        left, right = max(0.0, min(px)), min(float(W), max(px))
        if right - left < 8:
            return None
        u = (left + right) / 2
        return Gate(0, 0.0, math.degrees(math.atan2(u - CX, FX)), (u - CX) / (W / 2), (right - left) / W)

    def step(self, cmd):
        n = round(self.t / DT)
        if n % 5 == 0:              # 4 Hz 遙測
            self.meas = (self.depth, self.yaw)
        if n % 2 == 0:              # 10 Hz 相機
            self.frames.append((self.t + 0.15, self.see()))
        while self.frames and self.frames[0][0] <= self.t:
            g = self.frames.pop(0)[1]
            if g is not None:
                self.seq += 1
                g.seq, g.t = self.seq, self.t
                self.gate = g
        if cmd.surge > 0 and self.gate is None:
            self.surged_blind = True

        rate = max(-60.0, min(60.0, 2.0 * wrap(cmd.target_yaw - self.yaw)))
        self.yaw = wrap(self.yaw + rate * DT)
        self.depth += (cmd.target_depth - self.depth) * DT / 3.0
        self.u += (SPEED * cmd.surge - self.u) * DT / 1.5
        self.v += (SPEED * cmd.sway - self.v) * DT / 1.5
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        x0 = self.x
        self.x += (self.u * c - self.v * s) * DT
        self.y += (self.u * s + self.v * c) * DT
        if x0 < self.gate_xy[0] <= self.x:
            self.cross_y = self.y - self.gate_xy[1]
        self.t += DT


def run(world, params=None, max_s=300.0):
    logs = []
    flow = GatePass(params or Params(), log=logs.append)
    cmd = flow.first_cmd()
    phases = []
    while flow.result is None and world.t < max_s:
        cmd = flow.step(world.t, world.meas[0], world.meas[1], world.gate)
        phases.append((world.t, flow.phase, cmd))
        world.step(cmd)
    return flow, phases, logs


def phase_cmds(phases, name):
    return [c for _, p, c in phases if p == name]


def assert_passed(world, flow, phases):
    assert flow.result is True, flow.message
    assert world.cross_y is not None, '沒有通過門的平面'
    assert abs(world.cross_y) < GATE_W / 2 - 0.3, f'通過時偏離門中心 {world.cross_y:+.2f} m'
    assert not world.surged_blind
    assert [p for i, (_, p, _) in enumerate(phases) if i == 0 or phases[i - 1][1] != p] == \
        ['DIVE', 'ALIGN', 'APPROACH', 'PASS', 'STOP']
    assert abs(world.depth - flow.p.depth) < 0.05
    # 鎖航向之後目標航向不再變，轉向只發生在 ALIGN
    assert len({c.target_yaw for ph in ('APPROACH', 'PASS', 'STOP') for c in phase_cmds(phases, ph)}) == 1
    assert all(c.surge == 0 and c.sway == 0 for ph in ('DIVE', 'ALIGN', 'STOP') for c in phase_cmds(phases, ph))
    assert all(c.sway == 0 for c in phase_cmds(phases, 'PASS'))


def test_straight_ahead():
    world = World(gate_xy=(6.0, 0.0))
    flow, phases, _ = run(world)
    assert_passed(world, flow, phases)


def test_gate_on_the_right_turns_right():
    world = World(gate_xy=(6.0, 2.0))
    flow, phases, _ = run(world)
    assert_passed(world, flow, phases)
    locked = phase_cmds(phases, 'APPROACH')[0].target_yaw
    assert abs(locked - math.degrees(math.atan2(2.0, 6.0))) < 4.0, locked


def test_gate_on_the_left_and_heading_off():
    world = World(gate_xy=(7.0, -1.5), yaw=15.0)
    flow, phases, _ = run(world)
    assert_passed(world, flow, phases)
    assert phase_cmds(phases, 'APPROACH')[0].target_yaw < 0


def test_sway_corrects_sideways_drift():
    """對準後被推到旁邊（這裡用很鬆的對準容許值製造偏移）：靠 sway 拉回來，航向不動。"""
    world = World(gate_xy=(8.0, 1.2))
    flow, phases, _ = run(world, Params(align_tol=12.0, yaw_gain=0.0))    # 完全不轉向
    assert_passed(world, flow, phases)
    assert phase_cmds(phases, 'APPROACH')[0].target_yaw == 0.0
    assert max(c.sway for c in phase_cmds(phases, 'APPROACH')) > 0          # 門在右邊 → 往右移


def test_lost_when_close_goes_blind():
    """還沒到 near_width 偵測就掉了（門超出視野）：最後門框夠寬，照樣盲走過門。"""
    world = World(gate_xy=(6.0, 0.5), min_range=2.2)
    flow, phases, logs = run(world)
    assert_passed(world, flow, phases)
    assert any('近距離跟丟' in line for line in logs), logs


def test_no_gate_never_moves():
    world = World(has_gate=False)
    flow, phases, _ = run(world)
    assert flow.result is False and '沒看到門' in flow.message
    assert all(c.surge == 0 and c.sway == 0 for _, _, c in phases)
    assert abs(world.x) < 1e-6


def test_lost_far_away_stops_and_aborts():
    world = World(gate_xy=(9.0, 0.0), min_range=6.0)    # 6 m 就看不到，門框還很小
    flow, phases, _ = run(world)
    assert flow.result is False and '遠距離跟丟' in flow.message
    assert phases[-1][2].surge == 0
    assert world.x < 9.0 - 3.0, world.x                 # 沒有盲衝過去


def test_dive_timeout():
    world = World()
    logs = []
    flow = GatePass(log=logs.append)
    t = 0.0
    while flow.result is None and t < 100:
        cmd = flow.step(t, 0.10, 0.0, None)             # 深度卡在 0.10 m（例如池子太淺）
        t += DT
    assert flow.result is False and '沒到' in flow.message
    assert flow.phase == 'DIVE' and cmd.surge == 0


def test_state_lost_aborts():
    flow = GatePass(log=lambda s: None)
    t = 0.0
    while flow.result is None and t < 10:
        cmd = flow.step(t, math.nan, math.nan, None)
        t += DT
    assert flow.result is False and '深度或航向' in flow.message
    assert cmd.surge == 0 and cmd.sway == 0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
