"""先轉向對準再鎖航向的過門（trees/gate_pass_yaw.yaml）在平面模型裡跑。

流程：下潛 → 原地轉 yaw 對準門（AlignTo）→ 鎖住當下的航向 → sway 平移對準（CenterGate）→ 接近 → 盲走 → 停。
AlignTo 之後的流程跟 gate_pass.yaml 一樣（那些情境在 test_gate_flow.py）。

    cd src/orca_mission && python3 -m pytest test/
"""
import math
import os

import pytest
from py_trees.common import Status

from harness import GATE_W, Harness, World
from orca_mission.bt.loader import read
from orca_mission.task_runner import Mode

TREE = os.path.join(os.path.dirname(__file__), '..', 'trees', 'gate_pass_yaw.yaml')
ORDER = ['Dive', 'AlignTo', 'CenterGate', 'ApproachGate', 'Cruise', 'Hold']
AFTER_ALIGN = ORDER[2:]
ALIGN_TOL = 3.0         # gate_pass_yaw.yaml 的 align_tol


def bearing_from_start(world):
    """起點看門中心的方位角，相對歸零時的船頭 [deg]（下潛、轉向時船不會平移）。跑之前呼叫。"""
    gx, gy = world.gate_xy
    return math.degrees(math.atan2(gy, gx)) - world.yaw


def locked_yaw(h):
    """AlignTo 之後所有執行器送的目標航向，必須只有一個值（鎖住），回傳它。"""
    yaws = {round(c.target_yaw, 6) for t in AFTER_ALIGN for c in h.cmds(t)}
    assert len(yaws) == 1, f'對準之後航向還在變：{sorted(yaws)}'
    return yaws.pop()


def run(world, spec=TREE):
    h = Harness(world, spec)
    h.yaw0 = world.yaw          # 歸零時的世界航向
    h.bearing0 = bearing_from_start(world)
    h.yaw_at = {}               # 每一步的世界航向
    step = world.step

    def logged(cmd):
        h.yaw_at[round(h.t, 3)] = world.yaw
        step(cmd)
    world.step = logged
    h.run()
    return h


def assert_passed(h):
    w = h.world
    assert h.status == Status.SUCCESS, h.logs
    assert h.task_order() == ORDER, h.task_order()
    assert w.cross_y is not None, '沒有通過門的平面'
    assert abs(w.cross_y) < GATE_W / 2 - 0.3, f'通過時偏離門中心 {w.cross_y:+.2f} m'
    assert not w.surged_blind
    # 下潛時航向還鎖在歸零時的 0°；轉向對準時原地轉，不平移、不前進
    assert {round(c.target_yaw, 6) for c in h.cmds('Dive')} == {0.0}
    assert all(c.surge == 0 and c.sway == 0 for t in ('Dive', 'AlignTo', 'Hold') for c in h.cmds(t))
    assert all(c.surge == 0 for c in h.cmds('CenterGate'))      # 對準完才往前
    assert all(c.sway == 0 for c in h.cmds('Cruise'))
    assert all(abs(c.target_depth - 0.30) < 1e-9 for t in ORDER for c in h.cmds(t))
    # 對準後鎖住的航向：船頭朝向門中心（誤差在 align_tol 內），船也真的照這個航向走
    yaw = locked_yaw(h)
    assert abs(yaw - h.bearing0) <= ALIGN_TOL + 0.5, (yaw, h.bearing0)
    assert abs(w.yaw - (h.yaw0 + yaw)) < 1.0
    assert any(f'鎖航向 {yaw:+.1f}°' in line for line in h.logs), h.logs
    assert h.runner.mode == Mode.FINISH and not h.armed
    return yaw


def test_straight_ahead_no_turn():
    h = run(World(gate_xy=(6.0, 0.0)))
    assert assert_passed(h) == 0.0
    assert {round(c.target_yaw, 6) for c in h.cmds('AlignTo')} == {0.0}
    assert all(c.sway == 0 for c in h.cmds('CenterGate'))


@pytest.mark.parametrize('world', [
    dict(gate_xy=(6.0, 2.0)),                   # 門在右前方 18°
    dict(gate_xy=(7.0, -1.5)),                  # 門在左前方 12°
    dict(gate_xy=(5.0, 3.0)),                   # 門在右前方 31°（一開始門框被畫面邊緣切掉一部分）
    dict(gate_xy=(6.0, 0.0), yaw=10.0),         # 歸零時船頭偏右 10°：轉回來
    dict(gate_xy=(6.0, 1.0), yaw=-15.0),        # 歸零時船頭偏左 15°、門也在右邊
], ids=['right', 'left', 'far-right', 'heading-right', 'heading-left'])
def test_turns_to_gate_then_locks(world):
    h = run(World(**world))
    yaw = assert_passed(h)
    assert yaw * h.bearing0 > 0 and abs(yaw) > ALIGN_TOL      # 往門的方向轉了
    # 鎖住之後就不再轉：世界航向從 CenterGate 開始到結束都停在鎖住的航向
    after = [h.yaw_at[round(t, 3)] for t, name, _ in h.trace if name in AFTER_ALIGN]
    assert max(abs(y - (h.yaw0 + yaw)) for y in after) < 1.0


def test_align_tol_from_vars():
    spec = read(TREE, {'align_tol': 6.0, 'center_tol': 0.2})
    found = {}

    def walk(node):
        found[node['type']] = node
        for c in node.get('children', []):
            walk(c)
    walk(spec['root'])
    assert found['AlignTo']['tol'] == 6.0 and found['CenterGate']['tol'] == 0.2
    h = run(World(gate_xy=(6.0, 2.0)), spec)
    assert h.status == Status.SUCCESS, h.logs
    assert h.task_order() == ORDER
    locked_yaw(h)


def test_no_gate_never_moves():
    """看不到門：AlignTo 不會自己轉頭找，find_timeout 後失敗；全程不轉、不前進、不平移。"""
    h = run(World(has_gate=False))
    assert h.status == Status.FAILURE
    assert any('沒看到目標' in line for line in h.logs), h.logs
    assert h.task_order() == ['Dive', 'AlignTo']
    assert all(c.surge == 0 and c.sway == 0 and c.target_yaw == 0 for _, _, c in h.trace)
    assert abs(h.world.x) < 1e-6 and abs(h.world.yaw) < 1e-6
    assert h.runner.mode == Mode.FINISH and not h.armed      # 失敗也一定上鎖
