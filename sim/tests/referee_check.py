#!/usr/bin/env python3
"""驗收：每一種規則事件都「做得出來、也讀得到」。

對著正在跑的模擬（sauvc_final.sdf 或 sauvc_qual.sdf）把船瞬移到道具旁邊、用外力推它去撞，
再檢查 sim/referee.py 有沒有在 ROS /sim/referee/events 發出對應的事件與分數，
以及該擋住的有擋住（門、起始區的框）、該撞得動的撞得動（flare 被撞歪、橘色 flare 自己回正）。
不經過飛控：ArduSub 在跑但沒解鎖，推進器不出力，船完全由這支程式擺布，所以結果可以重現。
推船用世界檔裡的 ApplyLinkWrench（/world/<名稱>/wrench/persistent），瞬移用 /world/<名稱>/set_pose。

用 deploy/sim/referee_check.sh 跑（它會把兩個世界各起一次；SEED=7 時決賽用隨機佈局）。flare 的順序、
drum 的種類從世界檔讀，所以隨機佈局也能檢查。單獨跑：

    cd deploy/sim && WORLD=sauvc_final.sdf GUI=0 docker compose up -d simulator sim_referee sim_dropper
    docker compose exec sim_referee bash -c 'source /opt/ros/humble/setup.bash && python3 /sim/tests/referee_check.py'

跑完印出每一項 PASS／FAIL，有 FAIL 時結束碼非 0。放球只能做一次（球放掉就不會再吸回去），
所以決賽那一輪要從剛重開的模擬開始。
"""
import json
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SIM_DIR)
from gzreq import Requester  # noqa: E402
HOLD_Z = -4.3           # 抵銷船的淨浮力的力 [N]（帶著球時約 4.2 N、放掉後約 5.8 N）


def yaw_quat(yaw):
    return math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)


class Check(Node):
    def __init__(self, requester):
        super().__init__('referee_check')
        self.req = requester                # gz service 的請求在另一個行程送（見 sim/gzreq.py）
        self.world_file = os.environ.get('WORLD', 'sauvc_final.sdf')
        root = ET.parse(os.path.join(SIM_DIR, 'worlds', self.world_file)).getroot().find('world')
        self.world = root.get('name')
        ax, ay, _az, _r, _p, ayaw = [float(v) for v in root.find("frame[@name='arena']/pose").text.split()]
        self.arena = (ax, ay, ayaw)
        self.props = {}         # include 的名稱 → 在 arena 座標的 (x, y, z)
        self.kinds = {}         # include 的名稱 → 道具種類（uri）
        for inc in root.findall('include'):
            pose = inc.find('pose')
            if pose is not None and pose.get('relative_to') == 'arena':
                kind = inc.findtext('uri').split('://')[-1].strip()
                name = (inc.findtext('name') or kind).strip()
                self.props[name] = tuple(float(v) for v in pose.text.split()[:3])
                self.kinds[name] = kind
        # flare 的順序（隨機佈局每次不同）：世界檔的 <orca:referee flare_order>
        ref = next((el for el in root if el.tag.endswith('}referee') or el.tag == 'referee'), None)
        self.flare_order = (ref.get('flare_order', '') if ref is not None else '').split()
        # 瞬移前先停在這裡等速度消掉（空曠、離所有道具都遠，隨機佈局也一樣）
        self.park = (36.0, 10.0, -1.0) if 'drum_1' in self.props else (5.0, 7.0, -0.6)

        self.events = []
        self.state = {}
        self.poses = {}
        self.lock = threading.Lock()
        latched = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sim/referee/events', self.on_event, latched)
        self.create_subscription(String, '/sim/referee/state', self.on_state, 10)
        self.pub_reset = self.create_publisher(Empty, '/sim/referee/reset', 10)
        self.pub_release = self.create_publisher(Empty, '/ball_dropper/release', 10)
        self.announced = None
        self.create_subscription(String, '/comms/flare_order', self.on_order, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.gz = GzNode()
        self.pub_wrench = self.gz.advertise(f'/world/{self.world}/wrench/persistent', EntityWrench)
        self.pub_clear = self.gz.advertise(f'/world/{self.world}/wrench/clear', Entity)
        self.gz.subscribe(Pose_V, f'/world/{self.world}/dynamic_pose/info', self.on_poses)
        self.results = []

    # ---- 收資料
    def on_event(self, msg):
        with self.lock:
            self.events.append(json.loads(msg.data))

    def on_order(self, msg):
        self.announced = msg.data

    def on_state(self, msg):
        self.state = json.loads(msg.data)

    def on_poses(self, msg):
        for p in msg.pose:
            if p.name == 'tube':            # 橘色 flare 的浮管（名稱只有它一個）
                q = p.orientation
                self.poses['tube_tilt'] = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
            elif p.name in ('flare_red', 'flare_yellow', 'flare_blue'):
                q = p.orientation
                tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
                self.poses[p.name] = (p.position.x, p.position.y, p.position.z, tilt)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- 座標：arena ⇄ 世界
    def to_world(self, x, y):
        ax, ay, yaw = self.arena
        return ax + x * math.cos(yaw) - y * math.sin(yaw), ay + x * math.sin(yaw) + y * math.cos(yaw)

    def to_arena(self, wx, wy):
        ax, ay, yaw = self.arena
        dx, dy = wx - ax, wy - ay
        return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)

    def vehicle_arena(self):
        v = self.state.get('vehicle')
        return (*self.to_arena(v[0], v[1]), v[2]) if v else None

    # ---- 擺布船
    def request(self, service, req, req_type, rep_type, tries=3, timeout=5000):
        """gz service 的請求。模擬忙的時候回應偶爾會晚到，冪等的請求就重試幾次。"""
        for _ in range(tries):
            ok, rep = self.req.request(service, req, rep_type, timeout)
            if ok and rep.data:
                return True
            self.spin(0.5)
        return False

    def teleport(self, x, y, z, heading=0.0, settle=True):
        """船擺到 arena (x, y, z)。heading：船頭相對 arena +x 的角度 [rad]。
        set_pose 只改位置、不會把速度歸零，船會帶著原本的速度繼續滑；所以先擺到空曠的停泊點（self.park），
        等阻力把速度吃掉（約 2.5 秒），再擺到目標。直接在目標旁邊等的話，隨機佈局時可能先滑去碰到旁邊的道具。"""
        if settle:
            self.teleport(*self.park, heading, settle=False)
            self.spin(2.5)
        self.clear()
        wx, wy = self.to_world(x, y)
        req = Pose()
        req.name = 'orca'
        req.position.x, req.position.y, req.position.z = wx, wy, z
        w, qx, qy, qz = yaw_quat(heading + self.arena[2])
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = w, qx, qy, qz
        if not self.request(f'/world/{self.world}/set_pose', req, Pose, Boolean):
            raise RuntimeError('set_pose 失敗')
        self.hold()
        self.spin(0.3)

    def push(self, fx=0.0, fy=0.0, fz=0.0, seconds=0.0):
        """對船施力（arena 座標的方向，N；fz 另外加上抵銷浮力的 HOLD_Z）。seconds > 0 時推完就放開。"""
        self.clear()
        yaw = self.arena[2]
        m = EntityWrench()
        m.entity.name = 'orca::base_link'
        m.entity.type = Entity.LINK
        m.wrench.force.x = fx * math.cos(yaw) - fy * math.sin(yaw)
        m.wrench.force.y = fx * math.sin(yaw) + fy * math.cos(yaw)
        m.wrench.force.z = fz + HOLD_Z
        self.pub_wrench.publish(m)
        if seconds > 0:
            self.spin(seconds)
            self.hold()

    def hold(self):
        self.push()

    def clear(self):
        e = Entity()
        e.name = 'orca::base_link'
        e.type = Entity.LINK
        self.pub_clear.publish(e)
        time.sleep(0.03)

    def reset(self):
        self.spin(0.3)
        with self.lock:
            self.events.clear()
        self.pub_reset.publish(Empty())
        self.wait('reset', 3.0)

    # ---- 斷言
    def wait(self, event, timeout, **match):
        """等到一個符合的事件（從上一次 reset 之後算起）。回傳事件或 None。"""
        end = time.monotonic() + timeout
        while True:
            with self.lock:
                for e in self.events:
                    if e['event'] == event and all(e.get(k) == v for k, v in match.items()):
                        return e
            if time.monotonic() > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.02)

    def expect(self, title, event, timeout=8.0, **match):
        e = self.wait(event, timeout, **match)
        want = event + ''.join(f' {k}={v}' for k, v in match.items())
        self.record(title, e is not None, want if e is None else json.dumps(e, ensure_ascii=False))
        return e

    def record(self, title, ok, detail=''):
        self.results.append((title, ok, detail))
        print(f'{"PASS" if ok else "FAIL"}  {title}\n        {detail}', flush=True)

    # ============================================================ 決賽
    def run_final(self):
        P = self.props
        gate, floor = P['gate'], -2.0
        self.reset()

        # 起始區：沒下潛就往外開，會被浮框擋住
        self.push(fy=30, fz=-HOLD_Z)                    # 在水面，不抵銷浮力
        e = self.expect('起始區：在水面往外開會撞到浮框', 'contact_begin', 6.0, what='start_zone')
        self.spin(2.0)
        v = self.vehicle_arena()
        self.record('起始區：浮框把船擋住（沒下潛出不去）', v[1] < P['start_zone'][1] + 0.72,
                    f'船在 arena y = {v[1]:.2f}，框的前緣在 y = {P["start_zone"][1] + 0.72:.2f}')
        self.teleport(P['start_zone'][0], P['start_zone'][1], -0.2)
        self.reset()

        # 下潛之後離開起始區（推 1.2 秒，約到 0.7 m：推太久球會碰到 2 m 深的池底，那也算船碰池底）。
        # 往前只推 2 秒（約 1.7 m）：再遠可能撞到隨機擺在 y = 4～8 m 的橘色 flare
        self.push(fz=-25, seconds=1.2)
        self.expect('下潛', 'submerged')
        self.push(fy=25, seconds=2.0)
        self.expect('下潛後離開起始區', 'left_start_zone', submerged=True)

        # 導航門：撞柱子（擋住、扣 2 分），再從中間穿過（15 分）
        self.teleport(gate[0] - 0.75, gate[1] - 1.0, -1.2, math.pi / 2)
        self.push(fy=25, seconds=4.0)
        self.expect('碰到導航門', 'contact_begin', what='gate')
        self.expect('碰到導航門扣 2 分', 'penalty_gate', points=-2)
        v = self.vehicle_arena()
        self.record('導航門的柱子把船擋住', v[1] < gate[1], f'船在 arena y = {v[1]:.2f}，門在 y = {gate[1]:.2f}')
        self.teleport(gate[0], gate[1] - 1.0, -1.2, math.pi / 2)
        self.push(fy=25, seconds=3.0)       # 約到 y = 13.5：再遠可能撞到隨機擺在 y ≥ 14.4 m 的 flare
        self.expect('穿過導航門 15 分', 'gate_passed', points=15)
        code = '-'.join({'red': 'R', 'yellow': 'Y', 'blue': 'B'}[c] for c in self.flare_order)
        self.expect(f'過門後告知 flare 順序 {code}', 'flare_order_announced', order=code)
        self.spin(0.5)
        self.record('/comms/flare_order 收到同一個順序', self.announced == code, f'收到 {self.announced!r}')

        # 池底、池壁：各扣 5 分（y = 10：橘色 flare 的區域與門的線之間，隨機佈局也不會有道具）
        self.teleport(36.0, 10.0, -1.7, math.pi / 2)
        self.push(fz=-40, seconds=2.0)
        self.expect('碰到池底扣 5 分', 'penalty_floor_wall', what='floor', points=-5)
        self.teleport(49.0, 10.0, -1.0, 0.0)
        self.push(fx=30, seconds=3.0)
        self.expect('碰到池壁扣 5 分', 'penalty_floor_wall', what='wall', points=-5)
        v = self.vehicle_arena()
        self.record('池壁把船擋住', v[0] < 50.0, f'船在 arena x = {v[0]:.2f}，牆在 x = 50')

        # 照世界檔的順序把三支 flare 的球撞下來
        for colour in self.flare_order:
            fx, fy, _ = P[f'flare_{colour}']
            self.spin(0.2)
            before = self.poses.get(f'flare_{colour}')
            self.teleport(fx, fy - 0.75, floor + 0.65, math.pi / 2)
            self.push(fy=18, seconds=3.5)
            self.expect(f'撞 {colour} flare：碰到', 'contact_begin', what='flare', model=f'flare_{colour}')
            self.expect(f'撞 {colour} flare：球掉下來 20 分', 'flare_ball_dropped', colour=colour, points=20)
            after = self.poses.get(f'flare_{colour}')
            moved = before and after and (math.dist(before[:3], after[:3]) > 0.03 or after[3] > 10)
            self.record(f'{colour} flare 被撞歪或推走（不會自己回正）', bool(moved),
                        f'底盤位移 {math.dist(before[:3], after[:3]):.2f} m、傾斜 {after[3]:.0f}°' if before and after else '沒有位姿')
        self.expect(f'三支 flare 照順序（{code}）加 60 分', 'flare_order_bonus', points=60)

        # 放球：停在第 3 個 drum 上方，發 ROS 的放球指令（是哪一種 drum、幾分，看世界檔）
        drum = P['drum_3']
        kind = self.kinds['drum_3']
        drum_pts = {'drum_blue': 30, 'drum_red_pinger': 50, 'drum_red': 10}[kind]
        self.teleport(drum[0], drum[1], -1.2, math.pi / 2)
        self.spin(0.5)
        self.pub_release.publish(Empty())
        self.expect('放球指令（ROS /ball_dropper/release）→ 球離開船', 'ball_released', 5.0)
        self.expect(f'球掉進 drum_3（{kind}）並留在裡面 {drum_pts} 分', 'ball_in_drum', 25.0, drum=kind, points=drum_pts)

        # Surfacing Square：在框內上浮、沒碰到框
        sq = P['surfacing_square']
        self.teleport(sq[0], sq[1], -1.0, math.pi / 2)
        self.clear()                                    # 放手讓它自己浮上來
        e = self.expect('在 Surfacing Square 內上浮、沒碰框：5 + 20 分', 'surfaced', 15.0, points=25)
        end = self.expect('上浮後這次結束', 'run_ended', reason='surfaced')
        score = end['score'] if end else None
        want = 15 - 2 - 5 - 5 + 60 + 60 + drum_pts + 25
        self.record(f'總分 = {want}', score == want, f'裁判的分數 {score}')

        # 橘色 flare：碰到就 abort；放開後自己回正
        self.reset()
        of = P['orange_flare']
        self.teleport(of[0], of[1] - 0.8, -1.0, math.pi / 2)
        self.push(fy=25, seconds=3.0)
        self.expect('碰到橘色 flare 立即 abort', 'abort', reason='orange_flare')
        tilted = self.poses.get('tube_tilt', 0.0)
        self.teleport(of[0], of[1] - 3.0, -1.0, math.pi / 2)
        self.spin(4.0)
        back = self.poses.get('tube_tilt', 99.0)
        self.record('橘色 flare 被推歪、放開後自己回正', tilted > 5 and back < 2, f'被推時傾斜 {tilted:.0f}°，放開 4 秒後 {back:.1f}°')

        # Surfacing Square：碰到框就沒有 bonus
        self.reset()
        self.teleport(sq[0] + 0.975, sq[1], -1.0, math.pi / 2)
        self.clear()
        self.expect('上浮時碰到 Surfacing Square 的框', 'contact_begin', 12.0, what='surfacing_square')
        self.expect('碰到框：只有上浮的 5 分', 'surfaced', 12.0, points=5)

        # 池底碰 5 次 abort
        self.reset()
        for i in range(5):
            self.teleport(36.0, 10.0, -1.75, math.pi / 2)
            self.push(fz=-40, seconds=1.2)
            self.teleport(36.0, 10.0, -1.2, math.pi / 2)
            self.spin(0.8)
        self.expect('碰池底 5 次 abort', 'abort', reason='floor_wall_touches')

        # 壓在池底超過 10 秒 abort
        self.reset()
        self.teleport(36.0, 10.0, -1.75, math.pi / 2)
        self.push(fz=-40)
        self.expect('壓在池底累計超過 10 秒 abort', 'abort', 16.0, reason='floor_wall_time')
        self.clear()

    # ============================================================ 資格賽
    def run_qual(self):
        # 起點那面牆邊的池底只有 1.28 m 深：下潛只推 0.7 秒（約到 0.7 m），推太久會碰到池底（資格賽失敗）
        P = self.props
        gate = P['q_gate_2']
        start = self.vehicle_arena() or (0.248, 12.2, -0.21)
        home = (start[0], start[1], -0.2)

        def go_home():
            self.teleport(*home)
            self.clear()
            self.spin(1.5)
            self.reset()
            self.spin(0.5)

        # 出發時碰牆 → 去程過門 1 分 → 迴轉穿回來共 2 分
        go_home()
        self.expect('出發時船貼著起點標記的牆', 'start_wall_touch', 4.0)
        self.push(fz=-25, seconds=0.7)
        self.expect('下潛', 'submerged')
        self.teleport(gate[0] - 1.2, gate[1], -0.5, 0.0)
        self.push(fx=25, seconds=5.0)
        self.expect('去程穿過資格賽門 1 分', 'gate_passed', direction='forward', points=1)
        self.teleport(gate[0] + 1.2, gate[1], -0.5, math.pi)
        self.push(fx=-25, seconds=5.0)
        self.expect('迴轉穿回來共 2 分', 'gate_passed', direction='return', points=1)
        end = self.expect('兩趟都過：完成，資格賽 2 分', 'run_ended', reason='completed', qual_points=2)
        self.record('結束事件不另外加分（總分 = 2）', bool(end) and end['score'] == 2 and end['points'] == 0,
                    f'score {end["score"]}、points {end["points"]}' if end else '沒有結束事件')

        # 碰到門：失敗（柱子把船擋住）
        go_home()
        self.push(fz=-25, seconds=0.7)
        self.teleport(gate[0] - 1.0, gate[1] + 0.75, -0.5, 0.0)
        self.push(fx=25, seconds=4.0)
        self.expect('碰到資格賽門的柱子：失敗', 'qualification_failed', reason='touched_q_gate')
        v = self.vehicle_arena()
        self.record('資格賽門的柱子把船擋住', v[0] < gate[0], f'船在 arena x = {v[0]:.2f}，門在 x = {gate[0]:.2f}')

        # 碰到浮桿（門的第三邊）：失敗
        go_home()
        self.teleport(gate[0] - 0.6, gate[1], -0.2, 0.0)
        self.push(fx=25, fz=-HOLD_Z, seconds=4.0)
        self.expect('在水面撞到資格賽門的浮桿：失敗', 'qualification_failed', reason='touched_q_gate')

        # 碰到池底、池壁：失敗
        go_home()
        self.teleport(5.0, 12.0, -1.2, 0.0)
        self.push(fz=-40, seconds=2.5)
        self.expect('碰到池底：失敗', 'qualification_failed', reason='touched_floor')
        go_home()
        self.teleport(0.8, 14.0, -0.5, math.pi)
        self.push(fx=-25, seconds=3.0)
        self.expect('出發後碰到池壁：失敗', 'qualification_failed', reason='touched_wall')

        # 浮出水面：結束
        go_home()
        self.push(fz=-25, seconds=0.7)
        self.teleport(4.0, 12.0, -0.6, 0.0)
        self.clear()
        self.expect('下潛後浮出水面：這次結束', 'run_ended', 12.0, reason='surfaced')

        # 出發時沒碰牆：失敗
        self.teleport(2.0, 12.0, -0.2, 0.0)
        self.clear()
        self.spin(1.0)
        self.reset()
        self.push(fx=25, fz=-HOLD_Z, seconds=3.0)
        self.expect('出發時沒碰牆：失敗', 'qualification_failed', reason='no_wall_touch_at_start')
        self.clear()


def main():
    requester = Requester()                 # 要在建立任何 gz Node 之前
    rclpy.init()
    node = Check(requester)
    try:
        node.spin(2.0)
        if not node.state:
            print('沒收到 /sim/referee/state：sim_referee 有在跑嗎？', file=sys.stderr)
            return 2
        mode = node.state.get('mode')
        print(f'==== {node.world_file}（{mode}）')
        if mode == 'final':
            node.run_final()
        elif mode == 'qualification':
            node.run_qual()
        else:
            print(f'{node.world_file} 沒有 <orca:referee mode=...>，沒有可以檢查的規則', file=sys.stderr)
            return 2
        node.clear()
        failed = [r for r in node.results if not r[1]]
        print(f'==== {node.world_file}：{len(node.results) - len(failed)} / {len(node.results)} 項通過')
        out = os.environ.get('CHECK_OUT')
        if out:
            with open(out, 'w') as f:
                json.dump([{'check': t, 'ok': ok, 'detail': d} for t, ok, d in node.results], f, ensure_ascii=False, indent=1)
        return 1 if failed else 0
    finally:
        requester.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
