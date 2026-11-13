"""過門的行為樹（trees/gate_pass.yaml）在平面模型裡跑：情境跟 mission/test_gate_pass.py 一樣。

    cd src/orca_mission && python3 -m pytest test/
"""
import copy
import math
import os

import pytest
from py_trees.common import Status

from harness import GATE_W, Harness, World
from orca_mission.bt.loader import read
from orca_mission.task_runner import Mode

TREES = os.path.join(os.path.dirname(__file__), '..', 'trees')
GATE_PASS = os.path.join(TREES, 'gate_pass.yaml')
ORDER = ['Dive', 'AlignTo', 'ApproachGate', 'Cruise', 'Hold']


def spec_with(**task_params):
    """gate_pass.yaml，把某些執行器的參數換掉：spec_with(AlignTo={'tol': 12})。"""
    spec = read(GATE_PASS)

    def walk(node):
        node.update(task_params.get(node['type'], {}))
        for c in node.get('children', []):
            walk(c)
        if 'child' in node:
            walk(node['child'])
    walk(spec['root'])
    return read(copy.deepcopy(spec))


def assert_passed(h):
    w = h.world
    assert h.status == Status.SUCCESS, h.logs
    assert w.cross_y is not None, '沒有通過門的平面'
    assert abs(w.cross_y) < GATE_W / 2 - 0.3, f'通過時偏離門中心 {w.cross_y:+.2f} m'
    assert not w.surged_blind
    assert h.task_order() == ORDER, h.task_order()
    # 鎖航向之後目標航向不再變，轉向只發生在 AlignTo
    assert len({round(c.target_yaw, 6) for t in ('ApproachGate', 'Cruise', 'Hold') for c in h.cmds(t)}) == 1
    assert all(c.surge == 0 and c.sway == 0 for t in ('Dive', 'AlignTo', 'Hold') for c in h.cmds(t))
    assert all(c.sway == 0 for c in h.cmds('Cruise'))
    assert all(abs(c.target_depth - 0.30) < 1e-9 for t in ORDER for c in h.cmds(t))
    # 結束：Finish 節點上鎖
    assert h.runner.mode == Mode.FINISH and not h.armed


def test_straight_ahead():
    h = Harness(World(gate_xy=(6.0, 0.0)), GATE_PASS)
    h.run()
    assert_passed(h)


def test_gate_on_the_right_turns_right():
    h = Harness(World(gate_xy=(6.0, 2.0)), GATE_PASS)
    h.run()
    assert_passed(h)
    locked = h.cmds('ApproachGate')[0].target_yaw
    assert abs(locked - math.degrees(math.atan2(2.0, 6.0))) < 4.0, locked


def test_gate_on_the_left_and_heading_off():
    h = Harness(World(gate_xy=(7.0, -1.5), yaw=15.0), GATE_PASS)
    h.run()
    assert_passed(h)
    assert h.cmds('ApproachGate')[0].target_yaw < 0      # 相對歸零時（船頭偏右 15°）的航向


def test_sway_corrects_sideways_drift():
    """對準容許值很鬆、完全不轉向：靠 sway 拉回來，航向不動。"""
    h = Harness(World(gate_xy=(8.0, 1.2)), spec_with(AlignTo={'tol': 12.0, 'yaw_gain': 0.0}))
    h.run()
    assert_passed(h)
    assert abs(h.cmds('ApproachGate')[0].target_yaw) < 1.0
    assert max(c.sway for c in h.cmds('ApproachGate')) > 0      # 門在右邊 → 往右移


def test_lost_when_close_goes_blind():
    # 2.2 m 外就跟丟，模型速度 0.3 m/s：盲走要比預設的 5 秒長才走得到門（同 mission/test_gate_pass.py）
    h = Harness(World(gate_xy=(6.0, 0.5), min_range=2.2), read(GATE_PASS, {'pass_s': 8.0}))
    h.run()
    assert_passed(h)
    assert any('近距離跟丟' in line for line in h.logs), h.logs


def test_no_gate_never_moves():
    h = Harness(World(has_gate=False), GATE_PASS)
    h.run()
    assert h.status == Status.FAILURE
    assert any('沒看到目標' in line for line in h.logs), h.logs
    assert all(c.surge == 0 and c.sway == 0 for _, _, c in h.trace)
    assert abs(h.world.x) < 1e-6
    assert h.runner.mode == Mode.FINISH and not h.armed      # 失敗也一定上鎖


def test_lost_far_away_stops_and_aborts():
    h = Harness(World(gate_xy=(9.0, 0.0), min_range=6.0), GATE_PASS)
    h.run()
    assert h.status == Status.FAILURE
    assert any('遠距離跟丟' in line for line in h.logs), h.logs
    assert h.world.x < 9.0 - 3.0, h.world.x                     # 沒有盲衝過去


def test_dive_timeout():
    """池子只有 0.10 m 深：下潛 40 秒逾時（行為樹的 Timeout），中止。"""
    h = Harness(World(max_depth=0.10), GATE_PASS)
    h.run()
    assert h.status == Status.FAILURE
    assert any('下潛 逾時' in line for line in h.logs), h.logs
    assert h.task_order() == ['Dive']
    assert 40.0 <= h.t < 50.0


def test_state_lost_aborts():
    """解鎖後飛控不再回報深度、航向：執行器 2 秒後失敗。"""
    h = Harness(World(), GATE_PASS)
    while h.vehicle.yaw0 is None or not h.armed:
        h.run(max_s=h.t + 0.5)
    h.telemetry = False
    h.run(max_s=h.t + 20)
    assert h.status == Status.FAILURE
    assert any('收不到飛控的深度或航向' in line for line in h.logs), h.logs
    assert not h.armed


def test_override_vars():
    spec = read(GATE_PASS, {'depth': 0.5, 'pass_s': 3})
    h = Harness(World(), spec)
    h.run()
    assert h.status == Status.SUCCESS, h.logs
    assert all(abs(c.target_depth - 0.5) < 1e-9 for c in h.cmds('Cruise'))
    cruise_t = [t for t, name, _ in h.trace if name == 'Cruise']
    assert cruise_t[-1] - cruise_t[0] == pytest.approx(3.0, abs=0.15)


def test_smoke_tree_without_gate():
    """smoke_test.yaml：沒有門 → Selector 走「往前」那一支；轉向 90° 再轉回來。"""
    h = Harness(World(has_gate=False), os.path.join(TREES, 'smoke_test.yaml'))
    h.run()
    assert h.status == Status.SUCCESS, h.logs
    assert h.task_order() == ['Dive', 'TurnTo', 'Cruise', 'TurnTo', 'Dive']
    assert h.world.x == pytest.approx(0.0, abs=0.3) and h.world.y > 0.5     # 往右舷走了一段
    yaws = [c.target_yaw for c in h.cmds('TurnTo')]
    steps = [abs(b - a) for a, b in zip(yaws, yaws[1:])]
    assert max(steps) <= 30.0 * 0.05 + 1e-6                     # Shaper 限制轉速 30°/s
