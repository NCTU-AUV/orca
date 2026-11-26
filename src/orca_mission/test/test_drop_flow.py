"""投球的行為樹（trees/drop_ball.yaml、drop_test.yaml）在平面模型裡跑。

drop_ball：下潛 → 前視轉向對準 drum → 接近到底部相機看到 → 停在正上方 → 放球 → 停。
其餘情境：沒對準不放球、放球機構沒就緒、放球失敗。

    cd src/orca_mission && python3 -m pytest test/
"""
import copy
import math
import os

import pytest
from py_trees.common import Status

from harness import BOTTOM, Done, DrumWorld, Harness, World
from orca_mission.bt.loader import read
from orca_mission.task_runner import Mode

TREES = os.path.join(os.path.dirname(__file__), '..', 'trees')
DROP_BALL = os.path.join(TREES, 'drop_ball.yaml')
DROP_TEST = os.path.join(TREES, 'drop_test.yaml')


def run(world, spec=DROP_BALL, **over):
    h = Harness(world, read(spec, over) if isinstance(spec, str) else spec)
    h.run()
    return h


def test_drop_ball_over_drum():
    """drum 在右前方：轉向、接近、平移到正上方才放，放的位置離 drum 中心 10 cm 內。"""
    w = DrumWorld(drum_xy=(6.0, 1.0))
    h = run(w)
    assert h.status == Status.SUCCESS, h.logs
    assert h.task_order() == ['Dive', 'AlignTo', 'ApproachTarget', 'CenterOver', 'Hold'], h.task_order()
    assert h.drop_xy is not None and not h.dropper_holding
    miss = math.dist(h.drop_xy, w.drum_xy)
    assert miss < 0.1, f'放球位置離 drum 中心 {miss:.2f} m'
    # 對準完才放：放球那段（沒有執行器在跑）推力都是 0，深度維持
    assert all(c.surge == 0 and c.sway == 0 for t, name, c in h.trace if name == '' and h.armed)
    assert all(abs(c.target_depth - 0.5) < 1e-9 for t in ('AlignTo', 'ApproachTarget', 'CenterOver')
               for c in h.cmds(t))
    assert h.runner.mode == Mode.FINISH and not h.armed


def test_aim_offset_moves_drop_point():
    """aim_y = -0.3（電磁鐵在相機前方）：drum 停在畫面偏上的位置，船停得比 drum 後面一點。"""
    w = DrumWorld(drum_xy=(6.0, 0.0))
    h = run(w, aim_y=-0.3)
    assert h.status == Status.SUCCESS, h.logs
    h_cam = w.floor - 0.3 - 0.5                     # 相機到 drum 口（投球深度 0.5 m）
    expect_ahead = 0.3 * 240 / 462 * h_cam          # drum 在船前方這麼遠
    assert w.drum_xy[0] - h.drop_xy[0] == pytest.approx(expect_ahead, abs=0.06)


class NoBottomWorld(DrumWorld):
    """底部相機看不到 drum（例如偵測在俯視畫面認不出來）。"""

    def detect(self):
        out = super().detect()
        out[BOTTOM.frame_id] = []
        return out


def test_no_bottom_detection_does_not_drop():
    h = run(NoBottomWorld(drum_xy=(6.0, 0.0)))
    assert h.status == Status.FAILURE
    assert h.drop_xy is None and h.dropper_holding, '沒對準不能放球'
    assert 'CenterOver' not in h.task_order()
    assert any('還是沒看到' in line for line in h.logs), h.logs
    assert h.runner.mode == Mode.FINISH and not h.armed


def test_lost_in_bottom_camera_does_not_drop():
    """對準到一半底部相機跟丟：CenterOver 失敗，不放球。"""
    w = DrumWorld(drum_xy=(6.0, 0.0))
    h = Harness(w, read(DROP_BALL))
    detect = w.detect

    def flaky():
        out = detect()
        if h.runner.running() is not None and h.runner.running().name == 'CenterOver':
            out[BOTTOM.frame_id] = []
        return out
    w.detect = flaky
    h.run()
    assert h.status == Status.FAILURE
    assert h.drop_xy is None and h.dropper_holding
    assert any('跟丟目標' in line for line in h.logs), h.logs


def test_waits_for_dropper_ready():
    """沒收到 /ball_dropper/holding（None）：等就緒逾時，不解鎖。"""
    h = Harness(World(has_gate=False), read(DROP_TEST, {'ready_timeout': 3}))
    h.dropper_holding = None
    h.run(max_s=20)
    assert h.status == Status.FAILURE
    assert not h.armed and h.runner.mode == Mode.IDLE
    assert any('放球機構沒回報吸住' in line for line in h.logs), h.logs


def test_drop_test_tree():
    h = run(World(has_gate=False), DROP_TEST)
    assert h.status == Status.SUCCESS, h.logs
    assert h.task_order() == ['Dive', 'Hold'], h.task_order()     # 兩個 Hold 中間是 DropBall（沒有執行器）
    assert not h.dropper_holding and h.drop_xy is not None
    assert h.world.depth > 0.4                                     # 在水裡放的


def test_drop_failure_fails_tree():
    """放球服務回覆失敗：DropBall 失敗，樹失敗、上鎖。"""
    h = Harness(World(has_gate=False), read(DROP_TEST))
    calls = []

    def refuse(hold):
        calls.append(hold)
        return Done((False, 'GPIO 寫不進去'))
    # Context 在 Harness 建構時就綁好 dropper；直接換掉 DropBall 節點的 ctx.dropper
    for node in h.root.iterate():
        if type(node).__name__ == 'DropBall':
            node.ctx.dropper = refuse
    h.run()
    assert calls == [False]
    assert h.status == Status.FAILURE
    assert any('放球失敗：GPIO 寫不進去' in line for line in h.logs), h.logs
    assert h.runner.mode == Mode.FINISH and not h.armed


def test_trees_load_with_overrides():
    spec = read(DROP_BALL, {'labels': ['red_drum'], 'depth': 1.2})
    found = []

    def walk(node):
        if node['type'] in ('AlignTo', 'ApproachTarget', 'CenterOver'):
            found.append(node['labels'])
        for c in node.get('children', []):
            walk(c)
        if 'child' in node:
            walk(node['child'])
    walk(copy.deepcopy(spec)['root'])
    assert found == [['red_drum']] * 3


class EdgeBlindWorld(DrumWorld):
    """底部相機在畫面邊緣只偵測到第一幀，之後要到畫面中央附近（|offset| < 0.5）才又看得到。

    2026-11-26 模擬實測的情形：俯視的 drum 在邊緣分數 0.4～0.7、常常掉，正下方 0.9 以上。
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self.edge_seen = False

    def detect(self):
        out = super().detect()
        kept = []
        for d in out[BOTTOM.frame_id]:
            r = max(abs(d[2] - 320) / 320, abs(d[3] - 240) / 240)
            if r < 0.5:
                kept.append(d)
            elif not self.edge_seen:
                kept.append(d)
                self.edge_seen = True
                self.u = self.v = 0.0       # 模擬裡船很快就停住（這個平面模型的慣性比較大）
        out[BOTTOM.frame_id] = kept
        return out


def test_edge_detection_then_lost_still_centers():
    """邊緣看到一下就掉：CenterOver 往最後看到的方向移，回到看得到的範圍再對準（不會停在原地逾時）。"""
    w = EdgeBlindWorld(drum_xy=(6.0, 0.0))
    h = run(w)
    assert h.status == Status.SUCCESS, h.logs
    assert math.dist(h.drop_xy, w.drum_xy) < 0.1
    assert any(c.surge > 0 for c in h.cmds('CenterOver')), '看不到時要往前移'
