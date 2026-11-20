#!/usr/bin/env python3
"""模擬裁判：讀 Gazebo 的碰撞與位姿，照 SAUVC 2027 規則（Rulebook 2027.3.2）回報事件與分數。

    Gazebo（gz topic／service）                         ROS
    /orca/contacts          船碰到什麼（contact sensor）   /sim/referee/events  std_msgs/String（JSON），一個事件一則
    /world/<名稱>/dynamic_pose/info  會動的東西的位姿   →   /sim/referee/state   std_msgs/String（JSON），2 Hz 的現況
    /world/<名稱>/scene/info         模型清單與位置         /sim/referee/reset   std_msgs/Empty，收到就重新開始計分

**只給模擬用**：這些 topic 實機上不存在，任務程式不要訂閱（那等於偷看答案）。用途是驗證模擬與事後看紀錄：
事件也寫進 /logs/<名稱>.jsonl（deploy/sim/logs/referee/），recorder 錄 rosbag 時也會錄到。

另外扮演比賽的 Game Master 告知 flare 順序（任務可以訂閱這一個）：
    /comms/flare_order   std_msgs/String（latched），例如 "R-B-Y"
決賽時船第一次完整穿過導航門，就發布這次的順序（規則：過了門才告訴隊伍，隊伍再用自己的通訊設備傳給 AUV）。
重新計分時發布空字串（還沒告知）。**這個 topic 是暫定的**：實機的通訊方式還沒定，定了之後改 FLARE_ORDER_TOPIC。

決賽（世界檔 <orca:referee mode="final">，sauvc_final.sdf）
    gate_passed            完整穿過導航門                                    +15
    penalty_gate           碰到導航門（每一次）                               -2
    penalty_floor_wall     碰到池底或池壁（每一次）                           -5
    abort                  碰到橘色 flare；或碰池底／池壁累計超過 10 秒或 5 次   之後不再計分
    flare_ball_dropped     紅／黃／藍 flare 頂端的球掉下來                    +20（每支）
    flare_order_bonus      三支都掉了，而且順序跟 flare_order 一樣            +60
    ball_released          放球機構的球離開船
    ball_in_drum           球停在 drum 裡：藍 30、裝 pinger 的紅 50、其他紅 10（只算第一顆）
    ball_missed            球停在 drum 外面
    surfaced               下潛之後浮出水面                                   +5，這次結束
                           過門之後、在 Surfacing Square 內上浮、整趟沒碰到框  再 +20
    flare_order_announced  第一次過門時告知 flare 順序（同時發布 /comms/flare_order）
    其他：submerged（第一次完全下潛）、left_start_zone（離開起始區；submerged=false 表示沒先下潛就走）、
          contact_begin／contact_end（每一段接觸，含碰到的是哪個模型的哪個 collision）。
    規則：過門之前做其他任務不算分（"Navigation must be completed before any other task"），
          事件照發，points 是 0、counted 是 false。

資格賽（<orca:referee mode="qualification">，sauvc_qual.sdf）
    start_wall_touch       出發時船貼著起點標記範圍內的牆
    gate_passed            完整穿過資格賽門：去程 1 分，迴轉穿回來共 2 分（第二次穿過就結束）
    qualification_failed   碰到池底、池壁、門的任何一邊，或出發時沒碰牆
    surfaced               浮出水面，這次結束（已經拿到的分數保留；規則沒寫清楚過門後上浮算不算失敗，
                           這裡當成「結束」）
    目標是離出發位置最近的起點標記與資格賽門。

怎麼判斷：
    碰撞   model://orca_cam 的 bumper_link 上有 contact sensor，訊息裡有對方 collision 的名稱
           （模型::link::collision）。模型是哪一種道具，從世界檔的 <include><uri> 查。
           放球機構的球（model://dropper_ball）也有 contact sensor（/orca/ball_contacts）：球還吸在船上時
           它是船最低的一點，它的接觸算成船的（vehicle_part = ball）；放開之後就不算了。
           同一個東西 0.5 秒內的斷續接觸算同一次。
    位姿   過門（船的中心穿過門的平面，而且在開口範圍內，再往前走完船身的長度）、球掉了沒、
           球在不在 drum 裡、有沒有下潛／上浮、在不在框內，都用 Gazebo 的真值位姿算。
    下面的尺寸常數要跟 sim/models 裡的模型一致。

設定：環境變數 WORLD（世界檔檔名，在 /sim/worlds 底下；隨機佈局是 generated/<世界>_seed<N>.sdf）、
LOG_DIR（/logs）、RUN_NAME（紀錄檔的名稱）。隨機佈局的真值（世界檔旁邊的 .layout.json，tools/sim/layout.py 產生）
會放進第一則 reset 事件的 layout 欄位，所以紀錄檔裡查得到這次的佈局與 seed。
模擬重開（模擬時間倒退）會自動重新開始計分。在 orca-sim image 裡跑，見 deploy/sim/compose.yaml 的 sim_referee 服務。
"""
import json
import math
import os
import re
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

SIM_DIR = os.path.dirname(os.path.abspath(__file__))

# ---- 尺寸（要跟 sim/models 一致）
VEHICLE = 'orca'
FOOTPRINT = [(0.25, 0.242), (0.25, -0.242), (-0.25, 0.242), (-0.25, -0.242)]     # 含推進器的外緣 [m]（CAD 外框，見 model://orca）
HALF_LENGTH = 0.25
GATE_HALF_WIDTH = 0.75          # 兩柱中心距的一半（gate、q_gate 都是 150 cm）
GATE_HEIGHT = 1.48              # 導航門橫桿底下的淨高（原點在池底）
Q_GATE_DEPTH = 1.0              # 資格賽門的柱子往下多深（原點在水面）
SQUARE_INNER = 0.95             # Surfacing Square 內緣的一半
START_ZONE_INNER = 0.70         # 起始區內緣的一半
MARKER_HALF_WIDTH = 0.5         # 資格賽起點標記寬 1 m
DRUM_R, DRUM_H = 0.27, 0.30     # drum 內側半徑（取盆底與盆口之間）、深度
BALL_SEAT_Z = 0.82              # flare 的球擱在球座上時，球心離池底的高度

# ---- 判斷用的門檻
CONTACT_HOLD_S = 0.15           # 這麼久沒收到接觸訊息就算分開了
TOUCH_GAP_S = 0.5               # 分開不到這麼久又碰到，算同一次
SUBMERGED_Z = -0.30             # 船的原點低於這個深度算完全下潛（船殼頂在原點上方 0.215 m）
SURFACE_Z = -0.26               # 高於這個深度算在水面（浮著時原點在 -0.21 m；被浮框壓在底下時約 -0.24 m）
TELEPORT_M = 0.3                # 一次判斷（20 ms）之間位置跳這麼遠，當成被瞬移（set_pose），不算穿過門
SURFACE_HOLD_S = 1.0
DEPART_M = 0.5                  # 離出發位置這麼遠算出發了
BALL_MOVED_M = 0.05             # 球離開原位這麼遠算掉了／放開了
REST_S, REST_M = 0.6, 0.004     # 球在 REST_S 內移動不到 REST_M 算停了
BALL_TIMEOUT_S = 20.0           # 放開後最多等這麼久就判定落點

POINTS_GATE = 15
PENALTY_GATE = -2
PENALTY_FLOOR_WALL = -5
MAX_FLOOR_WALL_TOUCHES = 5
MAX_FLOOR_WALL_S = 10.0
POINTS_FLARE = 20
POINTS_FLARE_ORDER = 60
POINTS_DRUM = {'drum_blue': 30, 'drum_red_pinger': 50, 'drum_red': 10}
POINTS_SURFACE = 5
FLARE_ORDER_TOPIC = '/comms/flare_order'     # 暫定，見檔頭
FLARE_CODE = {'red': 'R', 'yellow': 'Y', 'blue': 'B'}
POINTS_SQUARE = 20


# ---------------------------------------------------------------- 位姿的小工具
def quat_rotate(q, v):
    """q = (w, x, y, z)。把向量 v 從物體座標轉到世界座標。"""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def compose(parent, child):
    """位姿 = ((x, y, z), (w, x, y, z))。parent ∘ child。"""
    (pp, pq), (cp, cq) = parent, child
    r = quat_rotate(pq, cp)
    return (pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), quat_mul(pq, cq)


def to_local(frame, point):
    """世界座標的點 → frame 的座標。"""
    p, q = frame
    return quat_rotate(quat_conj(q), (point[0] - p[0], point[1] - p[1], point[2] - p[2]))


def msg_pose(p):
    o = p.orientation
    w = o.w if (o.w or o.x or o.y or o.z) else 1.0
    return (p.position.x, p.position.y, p.position.z), (w, o.x, o.y, o.z)


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


# ---------------------------------------------------------------- 世界檔
def model_name(kind):
    """sim/models/<kind>/model.sdf 裡 <model name="..."> 的名稱（include 沒寫 <name> 時的實例名稱）。"""
    try:
        with open(os.path.join(SIM_DIR, 'models', kind, 'model.sdf')) as f:
            m = re.search(r'<model\s+name="([^"]+)"', f.read())
        return m.group(1) if m else kind
    except OSError:
        return kind


def load_world(path):
    """回傳 (世界名稱, <orca:referee> 的屬性, {模型實例名稱: 道具種類（uri 的最後一段）})。"""
    root = ET.parse(path).getroot()
    world = root.find('world')
    config, kinds = {}, {}
    for el in world:
        if el.tag.endswith('}referee') or el.tag == 'referee':
            config = dict(el.attrib)
        elif el.tag == 'include':
            kind = el.findtext('uri', '').strip().split('://')[-1].strip('/').split('/')[-1]
            kinds[(el.findtext('name') or model_name(kind)).strip()] = kind
        elif el.tag == 'model':
            kinds[el.get('name')] = el.get('name')
    return world.get('name'), config, kinds


class Referee(Node):
    def __init__(self):
        super().__init__('sim_referee')
        world_file = os.environ.get('WORLD', 'orca_pool.sdf')
        self.world_file = world_file
        world_path = os.path.join(SIM_DIR, 'worlds', world_file)
        self.world, self.config, self.kinds = load_world(world_path)
        self.layout = None                  # 隨機佈局的真值（tools/sim/layout.py）；固定佈局沒有
        try:
            with open(os.path.splitext(world_path)[0] + '.layout.json') as f:
                self.layout = json.load(f)
        except (OSError, ValueError):
            pass
        self.mode = self.config.get('mode', 'none')
        self.flare_order = self.config.get('flare_order', '').split()
        self.get_logger().info(f'世界 {self.world}（{world_file}），模式 {self.mode}'
                               + (f'，flare 順序 {"-".join(self.flare_order)}' if self.flare_order else ''))

        latched = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_event = self.create_publisher(String, '/sim/referee/events', latched)
        self.pub_state = self.create_publisher(String, '/sim/referee/state', 10)
        self.pub_order = self.create_publisher(String, FLARE_ORDER_TOPIC, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Empty, '/sim/referee/reset', lambda _m: self.request_reset('收到 /sim/referee/reset'), 10)

        self.log = None
        log_dir = os.environ.get('LOG_DIR', '/logs')
        if os.path.isdir(log_dir):
            name = os.environ.get('RUN_NAME') or time.strftime('referee_%Y%m%d_%H%M%S')
            self.log = open(os.path.join(log_dir, name + '.jsonl'), 'a', buffering=1)
            self.get_logger().info(f'紀錄：{self.log.name}')

        self.lock = threading.Lock()
        self.gz = GzNode()
        self.links = {}             # entity id → (模型, link)
        self.models = {}            # entity id → 模型
        self.static_pose = {}       # 模型 → 位姿（scene/info 給的，不會動的東西）
        self.model_pose = {}        # 模型 → 位姿（dynamic_pose/info 的最新值）
        self.link_rel = {}          # (模型, link) → 相對模型的位姿
        self.sim_t = None
        self.contacts = {}          # (類別, 模型) → {'t': 最後一次接觸的模擬時間, 'collision': ..., 'pos': ...}
        self.scene_ok = False
        self.reset_reason = '啟動'
        self.reset_state()

        self.gz.subscribe(Contacts, '/orca/contacts', self.on_contacts)
        self.gz.subscribe(Contacts, '/orca/ball_contacts', self.on_ball_contacts)
        self.gz.subscribe(Pose_V, f'/world/{self.world}/dynamic_pose/info', self.on_poses)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.5, self.publish_state)
        self.create_timer(1.0, self.fetch_scene)

    # ------------------------------------------------------------ Gazebo 的資料
    def fetch_scene(self):
        """模型清單與 entity id：/world/<名稱>/scene/info。模擬還沒起來就每秒再試。"""
        if self.scene_ok:
            return
        try:
            ok, scene = self.gz.request(f'/world/{self.world}/scene/info', GzEmpty(), GzEmpty, Scene, 500)
        except Exception as e:                      # noqa: BLE001  服務還不存在
            self.get_logger().debug(f'scene/info: {e}')
            return
        if not ok or not scene.model:
            return
        links, models, static = {}, {}, {}
        for m in scene.model:
            models[m.id] = m.name
            static[m.name] = msg_pose(m.pose)
            for link in m.link:
                links[link.id] = (m.name, link.name)
        with self.lock:
            self.links, self.models, self.static_pose = links, models, static
            self.scene_ok = True
        self.get_logger().info(f'模型：{", ".join(sorted(static))}')

    def on_poses(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            if self.sim_t is not None and t < self.sim_t - 1.0:
                self.reset_reason = '模擬重開（模擬時間倒退）'
                self.scene_ok = False
            self.sim_t = t
            for p in msg.pose:
                if p.id in self.models:
                    self.model_pose[self.models[p.id]] = msg_pose(p)
                elif p.id in self.links:
                    self.link_rel[self.links[p.id]] = msg_pose(p)

    def on_contacts(self, msg, mine_prefix=VEHICLE + '::', part=None):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            for c in msg.contact:
                a, b = c.collision1.name, c.collision2.name
                mine, other = (a, b) if a.startswith(mine_prefix) else (b, a)
                key = self.classify(other)
                if key is None:
                    continue
                pos = (c.position[0].x, c.position[0].y, c.position[0].z) if c.position else None
                self.contacts[key] = {'t': t, 'collision': other, 'vehicle_part': part or mine.split('::')[-1],
                                      'pos': pos}

    def on_ball_contacts(self, msg):
        with self.lock:
            ball = self.ball
            attached = ball is not None and not ball['released'] and not ball['done']
        if attached:
            self.on_contacts(msg, mine_prefix=ball['model'] + '::', part='ball')

    def classify(self, collision):
        """對方 collision 的名稱 → (類別, 模型)。船自己、球不算。"""
        parts = collision.split('::')
        model, link = parts[0], parts[1] if len(parts) > 1 else ''
        kind = self.kinds.get(model, model)
        if model == VEHICLE or kind == 'dropper_ball':
            return None
        if kind.startswith('pool'):
            return ('floor' if link.startswith('floor') else 'wall'), model
        if kind in ('gate', 'q_gate', 'orange_flare', 'surfacing_square', 'start_zone'):
            return kind, model
        if kind.startswith('flare_'):
            return 'flare', model
        if kind.startswith('drum_'):
            return 'drum', model
        return 'other', model

    def pose_of(self, model, link=None):
        base = self.model_pose.get(model) or self.static_pose.get(model)
        if base is None or link is None:
            return base
        rel = self.link_rel.get((model, link))
        return compose(base, rel) if rel is not None else None

    def of_kind(self, *kinds):
        return [m for m, k in self.kinds.items() if k in kinds and (m in self.static_pose or m in self.model_pose)]

    # ------------------------------------------------------------ 狀態
    def request_reset(self, why):
        with self.lock:
            self.reset_reason = why

    def reset_state(self):
        self.score = 0
        self.ended = None           # 結束的原因（abort、surfaced、failed…），None = 進行中
        self.t0 = None
        self.start = None           # 出發時船的位姿
        self.departed = False
        self.submerged = False
        self.surface_since = None
        self.touch = {}             # (類別, 模型) → {'on': bool, 'since': t, 'last': t, 'count': n}
        self.floor_wall_touches = 0
        self.floor_wall_s = 0.0
        self.last_tick_t = None
        self.gate_touches = 0
        self.gate_passed = False
        self.gate_track = {}        # 門 → {'y': 上一次的 y, 'crossing': 方向或 None}
        self.q_points = 0
        self.q_target = None
        self.q_marker = None
        self.start_touch = False
        self.flares = {}            # 模型 → {'colour', 'p0', 'down'}
        self.flare_drops = []       # 掉球的順序（顏色）
        self.flare_bonus = False
        self.ball = None            # {'model', 'offset', 'released', 't_release', 'hist': [(t, p)], 'done'}
        self.square_touched = False
        self.left_start_zone = False
        self.contacts.clear()

    def emit(self, event, points=0, **detail):
        if self.ended is not None and event not in ('reset',):
            detail['after_end'] = True
            points = 0
        self.score += points
        rec = {'t': round(self.sim_t or 0.0, 3), 'event': event, 'points': points, 'score': self.score}
        if self.t0 is not None and self.sim_t is not None:
            rec['elapsed'] = round(self.sim_t - self.t0, 2)
        rec.update(detail)
        text = json.dumps(rec, ensure_ascii=False)
        self.pub_event.publish(String(data=text))
        if self.log:
            self.log.write(text + '\n')
        self.get_logger().info(text)

    def end(self, reason, **detail):
        """這次結束。detail 不能用 points 這個名字（那是 emit 的「這一則加幾分」），資格賽的分數用 qual_points。"""
        if self.ended is None:
            self.emit('run_ended', reason=reason, **detail)
            self.ended = reason

    def publish_state(self):
        with self.lock:
            v = self.pose_of(VEHICLE)
            state = {
                't': round(self.sim_t, 2) if self.sim_t is not None else None, 'mode': self.mode,
                'score': self.score, 'ended': self.ended,
                'vehicle': [round(c, 3) for c in v[0]] if v else None,
                'submerged': self.submerged, 'departed': self.departed,
                'touching': sorted(f'{k[0]}:{k[1]}' for k, s in self.touch.items() if s['on']),
                'floor_wall_touches': self.floor_wall_touches, 'floor_wall_s': round(self.floor_wall_s, 2),
                'gate_touches': self.gate_touches, 'gate_passed': self.gate_passed,
            }
            if self.mode == 'qualification':
                state.update(points=self.q_points, start_wall_touch=self.start_touch, target_gate=self.q_target)
            else:
                state.update(flares_down=list(self.flare_drops), flare_order=self.flare_order,
                             ball=('none' if self.ball is None else 'done' if self.ball['done']
                                   else 'released' if self.ball['released'] else 'attached'),
                             square_touched=self.square_touched)
        self.pub_state.publish(String(data=json.dumps(state, ensure_ascii=False)))

    # ------------------------------------------------------------ 每 20 ms 判斷一次
    def tick(self):
        with self.lock:
            if self.reset_reason is not None:
                why, self.reset_reason = self.reset_reason, None
                first = why == '啟動'
                self.reset_state()
                self.pub_order.publish(String(data=''))          # flare 順序：還沒告知
                extra = dict(world=self.world_file)
                if first and self.layout:
                    extra['layout'] = self.layout
                self.emit('reset', reason=why, mode=self.mode, **extra)
            now, vehicle = self.sim_t, self.pose_of(VEHICLE)
            if now is None or vehicle is None or not self.scene_ok:
                return
            if self.start is None:
                self.init_run(now, vehicle)
            dt = 0.0 if self.last_tick_t is None else max(0.0, now - self.last_tick_t)
            self.last_tick_t = now

            self.update_touches(now, dt)
            self.update_vehicle(now, vehicle)
            self.update_gates(vehicle)
            if self.mode == 'final':
                self.update_flares(vehicle)
                self.update_ball(now, vehicle)

    def init_run(self, now, vehicle):
        self.start, self.t0 = vehicle, now
        for m in self.of_kind('flare_red', 'flare_yellow', 'flare_blue'):
            p = self.pose_of(m, 'ball')
            base = self.pose_of(m)
            if p is None or base is None:
                continue
            # 重新計分時球已經不在球座上的 flare 不再算
            down = abs((p[0][2] - base[0][2]) - BALL_SEAT_Z) > BALL_MOVED_M
            self.flares[m] = {'colour': self.kinds[m].split('_', 1)[1], 'p0': p[0], 'down': down}
        balls = self.of_kind('dropper_ball')
        if balls:
            p = self.pose_of(balls[0])
            off = to_local(vehicle, p[0])
            self.ball = {'model': balls[0], 'offset': off, 'released': dist(off, (0, 0, 0)) > 0.5,
                         't_release': now, 'hist': [], 'done': False}
            if self.ball['released']:
                self.ball['done'] = True            # 一開始球就不在船上：不追蹤
        if self.mode == 'qualification':
            def nearest(kind):
                c = [(dist(self.pose_of(m)[0][:2], vehicle[0][:2]), m) for m in self.of_kind(kind)]
                return min(c)[1] if c else None
            self.q_target, self.q_marker = nearest('q_gate'), nearest('start_marker')

    # ---- 碰撞
    def update_touches(self, now, dt):
        for key, c in self.contacts.items():
            st = self.touch.setdefault(key, {'on': False, 'since': None, 'last': None, 'count': 0})
            touching = now - c['t'] < CONTACT_HOLD_S
            if touching and not st['on']:
                st['on'] = True
                if st['last'] is None or c['t'] - st['last'] > TOUCH_GAP_S:
                    st['count'] += 1
                    st['since'] = c['t']
                    self.on_touch(key, c, st)
            elif not touching and st['on']:
                st['on'] = False
                self.emit('contact_end', what=key[0], model=key[1], duration=round(c['t'] - st['since'], 3))
            if touching:
                st['last'] = c['t']
        if self.mode == 'final' and any(s['on'] for k, s in self.touch.items() if k[0] in ('floor', 'wall')):
            self.floor_wall_s += dt
            if self.floor_wall_s > MAX_FLOOR_WALL_S and self.ended is None:
                self.emit('abort', reason='floor_wall_time', seconds=round(self.floor_wall_s, 2))
                self.end('abort')

    def on_touch(self, key, c, st):
        what, model = key
        detail = dict(what=what, model=model, collision=c['collision'], vehicle_part=c['vehicle_part'],
                      position=[round(x, 3) for x in c['pos']] if c['pos'] else None, count=st['count'])
        self.emit('contact_begin', **detail)
        if what == 'surfacing_square':
            self.square_touched = True
        if self.mode == 'qualification':
            if what == 'wall' and not self.departed:
                return                              # 出發前貼著牆是規則要求的，見 update_vehicle
            if what in ('floor', 'wall', 'q_gate'):
                self.emit('qualification_failed', reason=f'touched_{what}', model=model)
                self.end('failed')
        elif self.mode == 'final':
            if what in ('floor', 'wall'):
                self.floor_wall_touches += 1
                self.emit('penalty_floor_wall', PENALTY_FLOOR_WALL, what=what, touches=self.floor_wall_touches)
                if self.floor_wall_touches >= MAX_FLOOR_WALL_TOUCHES:
                    self.emit('abort', reason='floor_wall_touches', touches=self.floor_wall_touches)
                    self.end('abort')
            elif what == 'gate':
                self.gate_touches += 1
                self.emit('penalty_gate', PENALTY_GATE, touches=self.gate_touches)
            elif what == 'orange_flare':
                self.emit('abort', reason='orange_flare', model=model)
                self.end('abort')

    # ---- 船：出發、下潛、上浮
    def update_vehicle(self, now, vehicle):
        (x, y, z), q = vehicle
        if not self.departed:
            if self.mode == 'qualification' and not self.start_touch and self.q_marker:
                marker = self.pose_of(self.q_marker)
                corners = [to_local(marker, compose(vehicle, ((cx, cy, 0), (1, 0, 0, 0)))[0]) for cx, cy in FOOTPRINT]
                gap = min(c[0] for c in corners)
                lateral = to_local(marker, (x, y, z))[1]
                if gap < 0.03 and abs(lateral) <= MARKER_HALF_WIDTH:
                    self.start_touch = True
                    self.emit('start_wall_touch', marker=self.q_marker, gap=round(gap, 3), lateral=round(lateral, 3))
            if dist((x, y), self.start[0][:2]) > DEPART_M:
                self.departed = True
                self.emit('departed')
                if self.mode == 'qualification' and not self.start_touch:
                    self.emit('qualification_failed', reason='no_wall_touch_at_start')
                    self.end('failed')
        if not self.submerged and z < SUBMERGED_Z:
            self.submerged = True
            self.emit('submerged', depth=round(-z, 3))
        if self.mode == 'final' and not self.left_start_zone:
            zones = self.of_kind('start_zone')
            if zones:
                lx, ly, _ = to_local(self.pose_of(zones[0]), (x, y, z))
                if max(abs(lx), abs(ly)) > START_ZONE_INNER:
                    self.left_start_zone = True
                    self.emit('left_start_zone', submerged=self.submerged, depth=round(-z, 3))
        if self.submerged and self.ended is None:
            if z > SURFACE_Z:
                if self.surface_since is None:
                    self.surface_since = now
                elif now - self.surface_since >= SURFACE_HOLD_S:
                    self.on_surfaced(vehicle)
            else:
                self.surface_since = None

    def on_surfaced(self, vehicle):
        if self.mode == 'qualification':
            self.emit('surfaced', points_kept=self.q_points)
            self.end('surfaced', qual_points=self.q_points)
            return
        detail, bonus = {}, 0
        squares = self.of_kind('surfacing_square')
        if squares:
            sq = self.pose_of(squares[0])
            corners = [to_local(sq, compose(vehicle, ((cx, cy, 0), (1, 0, 0, 0)))[0]) for cx, cy in FOOTPRINT]
            inside = all(max(abs(c[0]), abs(c[1])) <= SQUARE_INNER for c in corners)
            detail = dict(inside_square=inside, square_touched=self.square_touched, gate_passed=self.gate_passed)
            if inside and not self.square_touched and self.gate_passed:
                bonus = POINTS_SQUARE
        self.emit('surfaced', POINTS_SURFACE + bonus, square_bonus=bonus, **detail)
        self.end('surfaced')

    # ---- 過門
    def update_gates(self, vehicle):
        if self.mode == 'qualification':
            gates = [self.q_target] if self.q_target else []
        else:
            gates = self.of_kind('gate')
        for g in gates:
            frame = self.pose_of(g)
            lx, ly, lz = to_local(frame, vehicle[0])
            tr = self.gate_track.setdefault(g, {'y': ly, 'x': lx, 'crossing': None, 'start_side': 1 if ly > 0 else -1})
            prev = tr['y']
            jumped = math.hypot(ly - prev, lx - tr['x']) > TELEPORT_M
            tr['y'], tr['x'] = ly, lx
            if jumped:
                tr['crossing'] = None
            elif (prev < 0) != (ly < 0):                            # 船的中心穿過門的平面
                if self.kinds[g] == 'q_gate':
                    in_opening = abs(lx) < GATE_HALF_WIDTH and -Q_GATE_DEPTH < lz < 0
                else:
                    in_opening = abs(lx) < GATE_HALF_WIDTH and 0 < lz < GATE_HEIGHT
                direction = 1 if ly > 0 else -1
                if in_opening:
                    tr['crossing'] = {'dir': direction, 'offset': round(lx, 3), 'z': round(lz, 3)}
                else:
                    tr['crossing'] = None
                    self.emit('gate_missed', model=g, offset=round(lx, 3), z=round(lz, 3),
                              note='穿過門的平面，但不在開口範圍內（從旁邊、上面或下面過去）')
            c = tr['crossing']
            if c and ly * c['dir'] >= HALF_LENGTH:                  # 整個船身都過去了
                tr['crossing'] = None
                self.on_gate_passed(g, c, forward=(c['dir'] == -tr['start_side']))
            elif c and ly * c['dir'] < 0:
                tr['crossing'] = None                               # 沒走完又退回去

    def on_gate_passed(self, gate, c, forward):
        touched = self.touch.get((self.kinds[gate], gate), {}).get('count', 0)
        detail = dict(model=gate, offset=c['offset'], z=c['z'], touches=touched)
        if self.mode == 'qualification':
            if forward and self.q_points == 0:
                self.q_points = 1
                self.emit('gate_passed', 1, direction='forward', **detail)
            elif not forward and self.q_points == 1:
                self.q_points = 2
                self.emit('gate_passed', 1, direction='return', **detail)
                self.end('completed', qual_points=2)
            else:
                self.emit('gate_passed', 0, direction='forward' if forward else 'return', counted=False, **detail)
        elif not self.gate_passed:
            self.gate_passed = True
            self.emit('gate_passed', POINTS_GATE, **detail)
            if self.flare_order:
                code = '-'.join(FLARE_CODE.get(c, c) for c in self.flare_order)
                self.pub_order.publish(String(data=code))
                self.emit('flare_order_announced', order=code, topic=FLARE_ORDER_TOPIC)
        else:
            self.emit('gate_passed', 0, counted=False, **detail)

    # ---- flare 頂端的球
    def update_flares(self, vehicle):
        for m, f in self.flares.items():
            if f['down']:
                continue
            p = self.pose_of(m, 'ball')
            if p is None or dist(p[0], f['p0']) < BALL_MOVED_M:
                continue
            f['down'] = True
            self.flare_drops.append(f['colour'])
            counted = self.gate_passed
            pole = self.pose_of(m)
            self.emit('flare_ball_dropped', POINTS_FLARE if counted else 0, colour=f['colour'], model=m,
                      order=len(self.flare_drops), counted=counted,
                      vehicle_distance=round(dist(vehicle[0], pole[0]), 2) if pole else None,
                      touched=self.touch.get(('flare', m), {}).get('count', 0) > 0,
                      **({} if counted else {'note': '還沒過導航門，不算分'}))
            if len(self.flare_drops) == 3 and not self.flare_bonus and self.flare_order:
                self.flare_bonus = True
                ok = self.flare_drops == self.flare_order and self.gate_passed
                self.emit('flare_order_bonus', POINTS_FLARE_ORDER if ok else 0, order=self.flare_drops,
                          required=self.flare_order, counted=ok)

    # ---- 放球
    def update_ball(self, now, vehicle):
        b = self.ball
        if b is None or b['done']:
            return
        p = self.pose_of(b['model'])
        if p is None:
            return
        p = p[0]
        if not b['released']:
            expected = compose(vehicle, (b['offset'], (1, 0, 0, 0)))[0]
            if dist(p, expected) > BALL_MOVED_M:
                b['released'], b['t_release'] = True, now
                self.emit('ball_released', position=[round(c, 3) for c in p])
            return
        b['hist'].append((now, p))
        while b['hist'] and now - b['hist'][0][0] > REST_S:
            b['hist'].pop(0)
        resting = (now - b['t_release'] > REST_S and len(b['hist']) > 5
                   and all(dist(h[1], p) < REST_M for h in b['hist']))
        if not resting and now - b['t_release'] < BALL_TIMEOUT_S:
            return
        b['done'] = True
        for d in self.of_kind(*POINTS_DRUM):
            lx, ly, lz = to_local(self.pose_of(d), p)
            if math.hypot(lx, ly) < DRUM_R and 0 <= lz <= DRUM_H:
                kind = self.kinds[d]
                counted = self.gate_passed
                self.emit('ball_in_drum', POINTS_DRUM[kind] if counted else 0, model=d, drum=kind, counted=counted,
                          position=[round(c, 3) for c in p], resting=resting,
                          **({} if counted else {'note': '還沒過導航門，不算分'}))
                return
        self.emit('ball_missed', position=[round(c, 3) for c in p], resting=resting)


def main():
    rclpy.init()
    node = Referee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
