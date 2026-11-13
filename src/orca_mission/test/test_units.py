"""vehicle、perception、TaskRunner、YAML 載入與流程圖的單元測試。"""
import math
import os

import pytest
from py_trees.common import Status as BtStatus

from orca_mission.bt.behaviours import Context
from orca_mission.bt.loader import TreeError, build, find_tree, read
from orca_mission.bt.render import main as render_main
from orca_mission.bt.render import mermaid
from orca_mission.perception import CameraConfig, Perception
from orca_mission.snapshot import Snapshot
from orca_mission.task_runner import Mode, TaskRunner
from orca_mission.tasks import Status
from orca_mission.tasks.basic import Hold
from orca_mission.vehicle import Command, Limits, Shaper, VehicleState

TREES = os.path.join(os.path.dirname(__file__), '..', 'trees')


# ---------- vehicle ----------

def test_shaper_limits_and_slew():
    s = Shaper(Limits(max_surge=0.5, max_sway=0.3, max_depth=2.0, yaw_rate=30.0))
    s.reset(0.0)
    c = s(Command(surge=2.0, sway=-1.0, target_depth=5.0, target_yaw=90.0), 0.1)
    assert (c.surge, c.sway, c.target_depth) == (0.5, -0.3, 2.0)
    assert c.target_yaw == pytest.approx(3.0)
    c = s(Command(surge=math.nan, target_depth=-1.0, target_yaw=90.0), 0.1)
    assert c.surge == 0.0 and c.target_depth == 0.0 and c.target_yaw == pytest.approx(6.0)


def test_shaper_takes_short_way_across_180():
    s = Shaper(Limits(yaw_rate=30.0))
    s.reset(170.0)
    c = s(Command(target_yaw=-170.0), 0.2)          # 最短是 +20°：170 → 176
    assert c.target_yaw == pytest.approx(176.0)
    for _ in range(5):
        c = s(Command(target_yaw=-170.0), 0.2)
    assert c.target_yaw == pytest.approx(-170.0)


def quat(yaw_enu):
    return 0.0, 0.0, math.sin(yaw_enu / 2), math.cos(yaw_enu / 2)


def test_vehicle_zero_and_heading_history():
    v = VehicleState()
    v.on_rel_alt(0.0, -0.2)
    v.on_orientation(0.0, *quat(math.radians(30)))
    v.zero_here()
    assert v.depth(0.0) == pytest.approx(0.0) and v.heading(0.0) == pytest.approx(0.0)
    v.on_rel_alt(1.0, -0.7)                          # 往下 0.5 m
    v.on_orientation(1.0, *quat(math.radians(10)))  # ENU 逆時針少了 20° → 往右轉 20°
    assert v.depth(1.0) == pytest.approx(0.5)
    assert v.heading(1.0) == pytest.approx(20.0)
    assert v.heading_at(0.5) == pytest.approx(10.0)
    assert v.heading_at(1.3) == pytest.approx(20.0)  # 比最新一筆新：用最新的
    assert math.isnan(v.heading_at(-1.0))
    assert math.isnan(v.depth(2.5))                  # 超過 1 秒沒更新


# ---------- perception ----------

def test_perception_bearing_and_cameras():
    front = CameraConfig('front', 'f', '/f/info')
    bottom = CameraConfig('bottom', 'b', '/b/info', mount_pitch=-90)
    p = Perception([front, bottom])
    p.on_camera_info('front', 640, 480, [400, 0, 320, 0, 400, 240, 0, 0, 1])
    p.on_camera_info('bottom', 640, 480, [400, 0, 320, 0, 400, 240, 0, 0, 1])
    out = p.on_detections(10.0, 'f', [('gate', 0.8, 720, 240, 320, 200), ('gate', 0.9, 320, 40, 64, 48)],
                          lambda t: 5.0)
    assert [s.score for s in out] == [0.9, 0.8]     # 分數高的在前
    right = out[1]
    assert right.bearing == pytest.approx(45.0) and right.offset_x == pytest.approx(1.25)
    assert right.abs_bearing == pytest.approx(50.0) and right.width == pytest.approx(0.5)
    assert out[0].elevation == pytest.approx(math.degrees(math.atan2(200, 400)))
    down = p.on_detections(10.1, 'b', [('red_drum', 0.7, 320, 120, 50, 50)], lambda t: 5.0)
    assert math.isnan(down[0].abs_bearing) and down[0].offset_y == pytest.approx(-0.5)
    assert p.on_detections(10.2, 'unknown', [], lambda t: 0.0) is None
    snap = Snapshot(now=10.3, frames=p.snapshot())
    assert snap.best('gate').score == 0.9
    assert snap.best('red_drum', camera='front') is None
    assert snap.best('red_drum').camera == 'bottom'
    assert Snapshot(now=11.0, frames=p.snapshot()).best('gate') is None     # 太舊
    assert p.last_seen['red_drum'].camera == 'bottom'


# ---------- TaskRunner ----------

def test_runner_idle_active_finish():
    r = TaskRunner(log=lambda s: None)
    snap = Snapshot(now=0.0, depth=0.0, heading=0.0)
    assert r.step(snap) is None
    with pytest.raises(RuntimeError):
        r.start(Hold(seconds=1), snap)
    r.activate(Command(target_depth=0.3, target_yaw=10.0))
    r.arm()
    out = r.step(snap)
    assert out.mode == 'DEPTH_HOLD' and out.arm == 'ARM' and out.cmd.target_depth == 0.3
    assert r.step(snap).arm == ''                    # ARM 是事件，只送一次
    h = r.start(Hold(seconds=1.0), snap)
    assert r.status(h)[0] == Status.RUNNING
    t = 0.0
    while r.status(h)[0] == Status.RUNNING:
        t += 0.05
        r.step(Snapshot(now=t, depth=0.3, heading=10.0))
    assert r.status(h)[0] == Status.SUCCEEDED and t == pytest.approx(1.0, abs=0.06)
    h2 = r.start(Hold(seconds=5.0), snap)
    r.cancel(h2)
    assert r.status(h2)[0] == Status.CANCELED
    r.finish()
    out = r.step(snap)
    assert (out.mode, out.arm, out.cmd) == ('MANUAL', 'DISARM', Command())
    assert r.step(snap).arm == '' and r.mode == Mode.FINISH


# ---------- YAML ----------

def test_trees_load():
    for name in ('gate_pass', 'smoke_test'):
        spec = read(os.path.join(TREES, name + '.yaml'))
        root = build(spec, Context())
        assert root.name and spec['name'] == name


def bad(root, **kw):
    return {'root': root, **kw}


@pytest.mark.parametrize('spec, message', [
    (bad({'type': 'Dvie', 'depth': 1}), '沒有 Dvie'),
    (bad({'type': 'Dive'}), '缺少參數 depth'),
    (bad({'type': 'Dive', 'depth': 1, 'speed': 2}), '不認得的參數 speed'),
    (bad({'type': 'Dive', 'depth': 'deep'}), 'depth 要是數字'),
    (bad({'type': 'Sequence', 'children': []}), 'children 要是非空的列表'),
    (bad({'type': 'Sequence', 'children': [{'type': 'Hold', 'timeout': -1}]}), 'root.children[0]'),
    (bad({'type': 'Dive', 'depth': '$d'}), '沒有變數 $d'),
    (bad({'type': 'Retry', 'child': {'type': 'Hold'}}), 'times 要是正數'),
])
def test_tree_errors(spec, message):
    with pytest.raises(TreeError, match=message.replace('[', r'\[').replace('$', r'\$')):
        read(spec)


def test_override_unknown_var():
    with pytest.raises(TreeError, match='沒有這個變數'):
        read(os.path.join(TREES, 'gate_pass.yaml'), {'dept': 1})


def test_find_tree():
    assert find_tree('a/b.yaml') == 'a/b.yaml'
    assert os.path.isfile(find_tree('gate_pass'))


def test_mermaid_and_generated_docs_up_to_date():
    text = mermaid(read(os.path.join(TREES, 'gate_pass.yaml')))
    assert text.startswith('flowchart TD') and '對準門' in text and '⏱ 180 s' in text
    assert render_main(['--check']) == 0, '跑 ros2 run orca_mission render_tree 重新產生 trees/*.md'


def test_timeout_uses_snapshot_clock():
    """Timeout 用 Snapshot.now，不是牆上時間。"""
    now = [0.0]
    r = TaskRunner(log=lambda s: None)
    r.activate(Command())
    ctx = Context(runner=r, snapshot=lambda: Snapshot(now=now[0], depth=0.0, heading=0.0), log=lambda s: None)
    root = build(read(bad({'type': 'Hold', 'seconds': 100, 'timeout': 2})), ctx)
    root.tick_once()
    assert root.status == BtStatus.RUNNING
    now[0] = 2.5
    root.tick_once()
    assert root.status == BtStatus.FAILURE and r.running() is None     # 逾時 → 執行器被取消
