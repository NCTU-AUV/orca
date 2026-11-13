#!/usr/bin/env python3
"""mission：載入一棵行為樹（trees/*.yaml）並執行。

    ros2 run orca_mission mission gate_pass                      # 實機：開始前列檢查清單、等 Enter
    ros2 run orca_mission mission gate_pass --no-checklist       # 模擬
    ros2 run orca_mission mission gate_pass -p depth=0.5 -p pass_s=6

一個節點、一條執行緒（SingleThreadedExecutor），資料流：

  /mavros/state、rel_alt、imu/data ─> VehicleState ─┐
  /detections ＋ 各相機 camera_info ─> Perception ──┴─> Snapshot（每次 tick 一份，唯讀）
                                                          │
           行為樹 10 Hz（任務層：順序、重試、逾時、就緒檢查、歸零、解鎖）
                                                          │ start／status／cancel
           TaskRunner 20 Hz（執行器、限幅、限制轉速）──> /auv_control（唯一發布者）

- 控制只經 /auv_control；mavros 的 topic 只讀。adapter（orca_bridge）不改。
- 收尾一定要做：樹結束（成功或失敗）、飛控在流程中上鎖、程式出錯、Ctrl-C、Ctrl-Z、SIGTERM
  → MANUAL、全 0、DISARM，等飛控上鎖。adapter 的看門狗在 0.5 秒沒收到指令時只會送中位，
  不切模式、不上鎖，船會停在水裡維持深度。
- 狀態：/mission/status（std_msgs/String，JSON，2 Hz）；py_trees_ros 的樹快照
  （py-trees-tree-watcher 可以即時看）；CSV 記錄到 /logs/<樹>_<日期_時間>.csv（20 Hz）。
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time
import traceback

import py_trees
import py_trees_ros
import rclpy
import yaml
from mavros_msgs.msg import ManualControl, State
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from orca_msgs.msg import AuvControl

from .bt.behaviours import Context
from .bt.loader import TreeError, build, find_tree, read
from .perception import CameraConfig, Perception
from .snapshot import Snapshot
from .task_runner import Mode, TaskRunner
from .vehicle import Limits, Shaper, VehicleState

CONTROL_HZ = 20
TICK_HZ = 10
MODES = {'MANUAL': AuvControl.MODE_MANUAL, 'DEPTH_HOLD': AuvControl.MODE_DEPTH_HOLD}
ARMS = {'': AuvControl.ARM_NO_CHANGE, 'ARM': AuvControl.ARM, 'DISARM': AuvControl.DISARM}


class ZeroCall:
    """非同步呼叫 /auv_adapter/zero，給行為樹的 Zero 節點輪詢。"""

    def __init__(self, client):
        self.client, self.future, self.error = client, None, None
        self.t0 = time.monotonic()

    def done(self):
        if self.error:
            return True
        if self.future is None:
            if self.client.service_is_ready():
                self.future = self.client.call_async(Trigger.Request())
            elif time.monotonic() - self.t0 > 3.0:
                self.error = '找不到 /auv_adapter/zero，fc_adapter 有在跑嗎？'
                return True
            return False
        if not self.future.done() and time.monotonic() - self.t0 > 8.0:
            self.error = '/auv_adapter/zero 沒有回覆'
            return True
        return self.future.done()

    def result(self):
        if self.error:
            return False, self.error
        res = self.future.result()
        return (res.success, res.message) if res else (False, '沒有回覆')


def load_config(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    cams = [CameraConfig(name=name, **c) for name, c in (cfg.get('cameras') or {}).items()]
    return cams, Limits(**(cfg.get('limits') or {}))


def find_config(path):
    if path:
        return path
    for d in (os.environ.get('ORCA_MISSION_CONFIG_DIR', ''),
              os.path.join(os.path.dirname(__file__), '..', 'config')):
        p = os.path.join(d, 'orca.yaml')
        if d and os.path.isfile(p):
            return p
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('orca_mission'), 'config', 'orca.yaml')


class MissionNode(Node):
    def __init__(self, spec, cameras, limits, interactive, log_path):
        super().__init__('mission')
        self.tree_name = spec.get('name', 'mission')
        self.vehicle = VehicleState()
        self.perception = Perception(cameras)
        self.runner = TaskRunner(Shaper(limits), log=self.say)
        self.detections_t = -math.inf
        self.snap = self.snapshot()
        self.was_armed = False
        self.done = None                # (成功與否, 訊息)，樹結束或中止後才有
        self.last_tip = None

        self.pub = self.create_publisher(AuvControl, '/auv_control', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.zero_client = self.create_client(Trigger, '/auv_adapter/zero')
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: self.vehicle.on_rel_alt(self.now(), m.data),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(ManualControl, '/mavros/manual_control/send',
                                 lambda m: self.vehicle.on_adapter(self.now()), 10)
        self.create_subscription(Detection2DArray, '/detections', self.on_detections, 10)
        for cam in cameras:
            self.create_subscription(
                CameraInfo, cam.info_topic,
                lambda m, name=cam.name: self.perception.on_camera_info(name, m.width, m.height, m.k),
                qos_profile_sensor_data)

        ctx = Context(runner=self.runner, snapshot=lambda: self.snap,
                      zero=lambda: ZeroCall(self.zero_client), zero_here=self.vehicle.zero_here,
                      interactive=interactive, log=self.say)
        self.tree = py_trees_ros.trees.BehaviourTree(build(spec, ctx))
        self.tree.setup(node=self, timeout=5.0)

        self.f = open(log_path, 'w', newline='')
        self.csv = csv.writer(self.f)
        self.csv.writerow(['unix_time', 'node', 'feedback', 'fc_mode', 'armed', 'depth_m', 'heading_deg',
                           'target_depth', 'target_yaw', 'surge', 'sway', 'sightings'])
        self.create_timer(1.0 / CONTROL_HZ, self.on_control)
        self.tick_timer = self.create_timer(1.0 / TICK_HZ, self.on_tick)
        self.create_timer(0.5, self.on_status)

    # ---------- 輸入 ----------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        self.vehicle.on_state(self.now(), m.connected, m.armed, m.mode)

    def on_imu(self, m):
        q = m.orientation
        self.vehicle.on_orientation(self.now(), q.x, q.y, q.z, q.w)

    def on_detections(self, msg):
        self.detections_t = self.now()
        dets = []
        for d in msg.detections:
            for r in d.results[:1]:
                dets.append((r.hypothesis.class_id, r.hypothesis.score, d.bbox.center.position.x,
                             d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.perception.on_detections(stamp, msg.header.frame_id, dets, self.vehicle.heading_at) is None:
            self.get_logger().warn(f'/detections 的 frame_id「{msg.header.frame_id}」不在 config 的相機裡',
                                   throttle_duration_sec=10.0)

    def snapshot(self):
        now, v = self.now(), self.vehicle
        return Snapshot(
            now=now, connected=v.connected, armed=v.armed, mode=v.mode, zeroed=v.zeroed,
            depth=v.depth(now), heading=v.heading(now), adapter_alive=v.adapter_alive(now),
            cameras_ready=frozenset(self.perception.intrinsics), detections_t=self.detections_t,
            frames=self.perception.snapshot(), last_seen=dict(self.perception.last_seen))

    # ---------- 行為樹 10 Hz ----------

    def on_tick(self):
        if self.done is not None:
            return
        self.snap = self.snapshot()
        if self.runner.mode == Mode.ACTIVE:
            if self.snap.armed:
                self.was_armed = True
            elif self.was_armed:
                return self.end(False, '飛控在流程中上鎖了（failsafe？）')
        self.tree.tick()
        status = self.tree.root.status
        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            tip = self.tree.root.tip()
            why = '' if status == py_trees.common.Status.SUCCESS else \
                f'（{tip.name}：{tip.feedback_message}）' if tip else ''
            self.end(status == py_trees.common.Status.SUCCESS, f'行為樹 {status.value}{why}')

    def end(self, ok, message):
        """收尾：已經開始送指令就 MANUAL、全 0、DISARM。"""
        if self.done is not None:
            return
        self.done = (ok, message)
        self.tick_timer.cancel()
        if self.tree.root.status == py_trees.common.Status.RUNNING:
            self.tree.root.stop(py_trees.common.Status.INVALID)     # 正在跑的節點收到 terminate(INVALID)
        if self.runner.mode != Mode.IDLE:
            self.runner.finish()
        self.say(('完成：' if ok else '中止：') + message)

    # ---------- /auv_control 20 Hz ----------

    def on_control(self):
        snap = self.snapshot()
        out = self.runner.step(snap)
        if out is None:
            return
        c = out.cmd
        self.pub.publish(AuvControl(
            control_source=AuvControl.SW_AUTO, mode=MODES[out.mode], arm_command=ARMS[out.arm],
            surge=float(c.surge), sway=float(c.sway),
            target_depth=float(c.target_depth), use_depth_target=out.mode == 'DEPTH_HOLD',
            target_yaw=float(c.target_yaw), use_yaw_target=out.mode == 'DEPTH_HOLD'))
        tip = self.tree.root.tip()
        num = lambda v, n: '' if not math.isfinite(v) else round(v, n)
        seen = ' '.join(f'{s.camera}:{s.label}:{s.score:.2f}:{s.offset_x:+.2f}:{s.width:.2f}'
                        for _, (_, t, ss) in snap.frames.items() if snap.now - t < 0.5 for s in ss)
        self.csv.writerow([round(time.time(), 3), tip.name if tip else '', tip.feedback_message if tip else '',
                           snap.mode, snap.armed, num(snap.depth, 4), num(snap.heading, 2),
                           round(c.target_depth, 3), round(c.target_yaw, 2), round(c.surge, 3),
                           round(c.sway, 3), seen])

    # ---------- 狀態 ----------

    def on_status(self):
        snap, c = self.snap, self.runner.cmd
        tip = self.tree.root.tip()
        status = {
            'tree': self.tree_name, 'status': self.tree.root.status.value,
            'node': tip.name if tip else '', 'feedback': tip.feedback_message if tip else '',
            'fc_mode': snap.mode, 'armed': snap.armed, 'runner': self.runner.mode.value,
            'depth': snap.depth if math.isfinite(snap.depth) else None,
            'heading': snap.heading if math.isfinite(snap.heading) else None,
            'cmd': {'surge': c.surge, 'sway': c.sway, 'target_depth': c.target_depth, 'target_yaw': c.target_yaw},
            'seen': {lbl: {'camera': s.camera, 'age': round(snap.now - s.t, 2), 'abs_bearing': s.abs_bearing
                           if math.isfinite(s.abs_bearing) else None} for lbl, s in snap.last_seen.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if self.done is None and self.runner.mode != Mode.IDLE:
            print(f'  {status["node"][:14]:14s} 深度 {snap.depth:5.2f} m  航向 {snap.heading:+6.1f}°'
                  f'（目標 {c.target_yaw:+6.1f}）  surge {c.surge:+.2f} sway {c.sway:+.2f}  '
                  f'{status["feedback"]}', flush=True)
        elif self.done is None and tip is not None and tip.name != self.last_tip:
            self.last_tip = tip.name
            print(f'  {tip.name}：{tip.feedback_message}', flush=True)

    def say(self, text):
        print(text, flush=True)


def parse_overrides(items):
    out = {}
    for item in items:
        k, sep, v = item.partition('=')
        if not sep:
            raise TreeError(f'-p 的格式是 名稱=值，拿到 {item!r}')
        out[k.strip()] = yaml.safe_load(v)
    return out


def on_signal(signum, frame):
    # Ctrl-Z 預設會把程式暫停在背景、SIGTERM（docker stop）預設直接結束，
    # 兩者都不會上鎖（adapter 的看門狗只送中位）。改成跟 Ctrl-C 一樣走收尾
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description='執行一棵行為樹（trees/*.yaml）')
    ap.add_argument('tree', help='樹的名稱（trees/ 底下，例如 gate_pass）或 YAML 路徑')
    ap.add_argument('-p', '--param', action='append', default=[], metavar='名稱=值',
                    help='覆寫樹的 vars，可以重複')
    ap.add_argument('--no-checklist', action='store_true', help='不列檢查清單、不等 Enter（只在模擬用）')
    ap.add_argument('--config', help='相機與限幅設定（預設 config/orca.yaml）')
    ap.add_argument('--log-dir', default=os.environ.get('ORCA_LOG_DIR', '/logs'))
    ap.add_argument('--print-tree', action='store_true', help='只印出樹的結構，不執行')
    args = ap.parse_args(argv if argv is not None else rclpy.utilities.remove_ros_args(sys.argv)[1:])

    try:
        path = find_tree(args.tree)
        spec = read(path, parse_overrides(args.param))
        cameras, limits = load_config(find_config(args.config))
    except (TreeError, OSError) as e:
        print(f'載入失敗：{e}', file=sys.stderr)
        return 2
    if args.print_tree:
        print(py_trees.display.unicode_tree(build(spec, None)))
        return 0

    signal.signal(signal.SIGTSTP, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    # 不讓 rclpy 處理 Ctrl-C：它預設會先關掉 ROS，收尾的 DISARM 就送不出去
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(args.log_dir, time.strftime(f'{spec.get("name", "mission")}_%Y%m%d_%H%M%S.csv'))
    node = MissionNode(spec, cameras, limits, not args.no_checklist, log_path)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f'行為樹 {path}\n{py_trees.display.unicode_tree(node.tree.root)}\n記錄到 {log_path}', flush=True)

    try:
        while node.done is None:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        print()
        node.end(False, 'Ctrl-C')
    except Exception as e:
        traceback.print_exc()
        node.end(False, f'程式出錯：{type(e).__name__}: {e}')
    finally:
        # 收尾：TaskRunner 繼續送 MANUAL＋DISARM，等飛控上鎖（最多 5 秒）
        if node.done is None:
            node.end(False, '結束')
        finishing = node.runner.mode == Mode.FINISH
        deadline = time.monotonic() + 5.0
        settle = time.monotonic() + 0.5
        while finishing and time.monotonic() < deadline:
            try:
                executor.spin_once(timeout_sec=0.05)
            except KeyboardInterrupt:
                break
            except Exception:
                traceback.print_exc()
            if time.monotonic() > settle and not node.vehicle.armed:
                break
        if finishing:
            print('飛控已上鎖，船會浮上來' if not node.vehicle.armed
                  else '飛控沒有上鎖！用 QGC 或 kill switch 處理', flush=True)
        print(f'記錄在 {log_path}', flush=True)
        ok = node.done[0]
        node.f.close()
        node.tree.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
