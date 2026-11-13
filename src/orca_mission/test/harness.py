"""離線測試用的平面模型：代替船、飛控、adapter、相機與偵測，不需要 ROS 和 Gazebo。

跟 mission/test_gate_pass.py 的模型相同（數字也一樣），差別是這裡跑的是整套新架構：
行為樹（YAML）→ TaskRunner → Shaper → 「飛控」，影像經 Perception 換算方位。

- 航向：追目標航向，上限 60°/s；深度、前進、平移都是一階延遲
- 遙測：深度、航向、模式、解鎖狀態 4 Hz（跟實機經 mavproxy 拿到的差不多）
- 相機：640 px 寬、fx 462（HFOV 約 69°），10 Hz、延遲 0.15 秒；太近（min_range）就偵測不到
- 「adapter」：DEPTH_HOLD 時馬上切 ALT_HOLD；ALT_HOLD 才能解鎖；沒解鎖時推進器不出力；
  目標航向相對歸零時的航向（加回歸零時的世界航向再給「飛控」）
座標：x 是起點的正前方，y 是右邊，航向順時針為正。
"""
import dataclasses
import math

import py_trees

from orca_mission.bt.behaviours import Context
from orca_mission.bt.loader import build, read
from orca_mission.perception import CameraConfig, Perception
from orca_mission.snapshot import Snapshot
from orca_mission.task_runner import Mode, TaskRunner
from orca_mission.vehicle import Command, Limits, Shaper, VehicleState, wrap

DT = 0.05
W, H, FX, CX, CY = 640, 480, 462.0, 320.0, 240.0
GATE_W = 1.5
SPEED = 1.5         # 推力 1.0 時的速度 [m/s]（surge 0.2 → 0.3 m/s）
FRONT = CameraConfig('front', 'front_frame', '/front/camera_info')
BOTTOM = CameraConfig('bottom', 'bottom_frame', '/bottom/camera_info', mount_pitch=-90.0)


class World:
    def __init__(self, gate_xy=(6.0, 0.0), yaw=0.0, min_range=1.0, max_range=20.0, has_gate=True,
                 max_depth=10.0):
        self.x = self.y = self.u = self.v = self.depth = 0.0
        self.yaw = yaw
        self.gate_xy = gate_xy
        self.min_range, self.max_range, self.has_gate = min_range, max_range, has_gate
        self.max_depth = max_depth  # 池子的深度（測試「到不了目標深度」用）
        self.cross_y = None         # 通過門的平面時，相對門中心的橫向位置
        self.surged_blind = False   # 還沒看過門就前進過

    def see(self):
        """目前位置看到的門框 → (中心 u, 寬 px)，看不到回傳 None。"""
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
        return (left + right) / 2, right - left

    def step(self, cmd):
        rate = max(-60.0, min(60.0, 2.0 * wrap(cmd.target_yaw - self.yaw)))
        self.yaw = wrap(self.yaw + rate * DT)
        self.depth += (min(cmd.target_depth, self.max_depth) - self.depth) * DT / 3.0
        self.u += (SPEED * cmd.surge - self.u) * DT / 1.5
        self.v += (SPEED * cmd.sway - self.v) * DT / 1.5
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        x0 = self.x
        self.x += (self.u * c - self.v * s) * DT
        self.y += (self.u * s + self.v * c) * DT
        if x0 < self.gate_xy[0] <= self.x:
            self.cross_y = self.y - self.gate_xy[1]


class Done:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


class Harness:
    """把 World 接到行為樹與 TaskRunner。spec：read() 的結果或 YAML 路徑。"""

    def __init__(self, world, spec, telemetry=True, limits=None):
        self.world = world
        self.t = 0.0
        self.telemetry = telemetry      # False：飛控停止回報深度、航向（測試資料中斷）
        self.vehicle = VehicleState()
        self.perception = Perception([FRONT, BOTTOM])
        self.perception.on_camera_info('front', W, H, [FX, 0, CX, 0, FX, CY, 0, 0, 1])
        self.logs = []
        self.runner = TaskRunner(Shaper(limits or Limits()), log=self.logs.append)
        self.mode, self.armed = 'MANUAL', False
        self.frames = []                # 還在路上的偵測：(送達時間, 拍攝時間, 偵測)
        self.trace = []                 # (t, 正在跑的執行器名稱, 送出的 Command)
        self.snap = self.snapshot()
        ctx = Context(runner=self.runner, snapshot=lambda: self.snap,
                      zero=lambda: Done((True, '歸零')), zero_here=self.vehicle.zero_here,
                      interactive=False, log=self.logs.append)
        self.root = build(spec if isinstance(spec, dict) else read(spec), ctx)
        self.status = None

    def snapshot(self):
        v, now = self.vehicle, self.t
        return Snapshot(now=now, connected=True, armed=v.armed, mode=v.mode, zeroed=v.zeroed,
                        depth=v.depth(now), heading=v.heading(now), adapter_alive=True,
                        cameras_ready=frozenset(self.perception.intrinsics), detections_t=now,
                        frames=self.perception.snapshot(), last_seen=dict(self.perception.last_seen))

    def sense(self, n):
        w = self.world
        if n % 5 == 0 and self.telemetry:                 # 4 Hz 遙測
            yaw_enu = -math.radians(w.yaw)
            self.vehicle.on_rel_alt(self.t, -w.depth)
            self.vehicle.on_orientation(self.t, 0.0, 0.0, math.sin(yaw_enu / 2), math.cos(yaw_enu / 2))
        if n % 5 == 0:
            self.vehicle.on_state(self.t, True, self.armed, self.mode)
        if n % 2 == 0:                                    # 10 Hz 相機
            seen = w.see()
            dets = [] if seen is None else [('gate', 0.9, seen[0], CY, seen[1], seen[1] * 0.6)]
            self.frames.append((self.t + 0.15, self.t, dets))
        while self.frames and self.frames[0][0] <= self.t:
            _, stamp, dets = self.frames.pop(0)
            self.perception.on_detections(stamp, FRONT.frame_id, dets, self.vehicle.heading_at)

    def run(self, max_s=300.0):
        n = 0
        while self.t < max_s and (self.status is None or self.runner.mode == Mode.FINISH and self.armed):
            self.sense(n)
            self.snap = self.snapshot()
            if n % 2 == 0 and self.status is None:        # 行為樹 10 Hz
                self.root.tick_once()
                if self.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                    self.status = self.root.status
                    if self.runner.mode != Mode.IDLE:
                        self.runner.finish()
            out = self.runner.step(self.snap)              # TaskRunner 20 Hz
            cmd = Command(target_depth=0.0, target_yaw=self.world.yaw)
            if out is not None:
                self.mode = 'ALT_HOLD' if out.mode == 'DEPTH_HOLD' else 'MANUAL'
                if out.arm == 'ARM' and self.mode == 'ALT_HOLD':
                    self.armed = True
                elif out.arm == 'DISARM':
                    self.armed = False
                task = self.runner.running()
                self.trace.append((self.t, task.name if task else '', out.cmd))
                if self.armed and self.mode == 'ALT_HOLD':
                    # 目標航向是相對歸零時的航向；「飛控」跟 adapter 一樣加回歸零時的世界航向
                    yaw_zero = -math.degrees(self.vehicle.yaw0) if self.vehicle.yaw0 is not None else 0.0
                    cmd = dataclasses.replace(out.cmd, target_yaw=wrap(out.cmd.target_yaw + yaw_zero))
            if cmd.surge > 0 and not self.perception.last_seen:
                self.world.surged_blind = True
            self.world.step(cmd)
            self.t += DT
            n += 1
        return self.status

    # ---------- 給斷言用 ----------

    def task_order(self):
        names = [name for _, name, _ in self.trace if name]
        return [name for i, name in enumerate(names) if i == 0 or names[i - 1] != name]

    def cmds(self, task):
        return [c for _, name, c in self.trace if name == task]
