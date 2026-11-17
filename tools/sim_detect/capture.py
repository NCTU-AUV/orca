#!/usr/bin/env python3
"""偵測驗證，第一步：在 Gazebo 裡把相機擺到每個道具前面的固定位姿，存下影像與真值框。

用 deploy/sim/detect_sweep.sh 跑（sim_sweep 服務，orca-sim image）。不需要 ArduSub、不需要船：
自己起一個 gz server（GZ_PARTITION=orca_sweep，跟正在跑的模擬互不干擾），世界是決賽主場地
（sim/worlds/sauvc_final.sdf，含資格賽門與 Surfacing Square）拿掉船之後，加一組相機：

    rgb    跟船上前視相機（sim/models/orca_cam 的 front_camera）完全一樣：640x480、水平視角 0.9725 rad、
           visibility_mask 0x0FEFFFFF
    depth  同位置的 depth camera，給水下後處理用（同 orca_cam）
    seg    同位置的 segmentation camera：每個道具一個 label，算「真的看得到的部分」的真值框（考慮遮擋）

相機永遠水平（船不會俯仰），船頭（光軸）對著道具的瞄準點。位姿 = 道具 × 距離 × 角度 × 深度：
    距離   相機到道具中心的水平距離 [m]
    角度   觀看方向相對「基準方向」轉幾度（基準方向見 PROPS；門的 0° = 正對門面）
    深度   相機在水面下幾公尺
相機出了泳池、低於池底（留 0.25 m）、或跟別的道具擠在一起的位姿跳過。

每個位姿拍兩種條件，跟 UNDERWATER 開關的兩種狀態一模一樣：
    on   世界裡有 model://water（水面、水體霧），影像經過 sim/underwater.py 的後處理
         （噪點的亂數種子固定，每次跑結果一樣）
    off  把 model://water 從世界移除，影像原封不動（= UNDERWATER=0）
先把所有位姿用 on 拍一輪，移除水之後再用 off 拍一輪。

輸出（OUT_DIR/RUN，預設 /out/<RUN>，對應主機的 deploy/sim/logs/detect/<RUN>/）：
    frames/on/NNNN.jpg、frames/off/NNNN.jpg   影像（JPEG 品質 95）
    frames.csv                                 每一張的道具、條件、距離、角度、深度、相機位姿、真值框、看得到的像素數
    meta.json                                  這次的設定（世界、位姿清單、水下效果參數…）
接著跑 evaluate.py（detect_eval 服務）算偵測率。

環境變數：RUN（輸出資料夾名稱）、PROPS（只跑這些道具，逗號分隔，例：gate,flare_red）、QUICK=1（少量位姿，試跑用）、
UNDERWATER_STRENGTH（水下效果強度，預設 1.0）、SWEEP_WORLD（預設 sauvc_final.sdf）。
"""
import csv
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

SIM_DIR = os.environ.get('SIM_DIR', '/sim')
sys.path.insert(0, SIM_DIR)
from gzreq import Requester  # noqa: E402
from underwater import Underwater  # noqa: E402

# ---- 相機（要跟 sim/models/orca_cam 的 front_camera 一樣）
WIDTH, HEIGHT, HFOV = 640, 480, 0.9725
RGB_MASK = 0x0FEFFFFF
RATE = 20

# ---- 泳池（要跟 tools/sim/gen_assets.py 一樣）
POOL_X, POOL_Y = 50.0, 25.0
SHELF_X, SHELF_DEPTH, SLOPE_X, DEEP_DEPTH = 1.8, 1.28, 17.5, 2.0

# ---- 道具：(種類, 期望的類別, 也算對的類別, 瞄準點在模型原點上方幾公尺, 基準方向 [deg，arena 座標，從道具往相機])
# 種類 = 世界檔 <include> 的 uri；同一種有好幾個時，取最靠近場地中央的那一個。
# 基準方向挑的是 AUV 實際接近的方向、而且池子裡拉得出夠長的距離：
#   門從起始區那一側（南）看；資格賽門從出發的牆（西）看；三支 flare、drum 從門那一側（南）看；
#   橘色 flare 從西邊看（從起始區看只拉得到 6 m）；Surfacing Square 從主場地（東）看。
PROPS = {
    'gate':             (['gate'], [], 0.75, -90),
    'q_gate':           (['qualification_gate'], ['gate'], -0.5, 180),
    'orange_flare':     (['orange_flare'], [], 0.75, 180),
    'flare_red':        (['red_flare'], [], 0.40, -90),
    'flare_yellow':     (['yellow_flare'], [], 0.40, -90),
    'flare_blue':       (['blue_flare'], [], 0.40, -90),
    'drum_red':         (['red_drum'], [], 0.15, -90),
    'drum_blue':        (['blue_drum'], [], 0.15, -90),
    'surfacing_square': ([], [], -0.15, 0),      # 模型沒有這一類：預期偵測不到，看它被認成什麼
}
DISTANCES = [1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20]
BEARINGS = [-45, 0, 45]
DEPTHS = [0.5, 1.0, 1.5]
QUICK = ([2, 5, 10], [0], [1.0])
MIN_CLEARANCE = 0.3         # 相機離其他道具中心至少這麼遠（水平）
POOL_MARGIN = 0.4
FLOOR_MARGIN = 0.25


def floor_depth(x):
    if x <= SHELF_X:
        return SHELF_DEPTH
    if x >= SHELF_X + SLOPE_X:
        return DEEP_DEPTH
    return SHELF_DEPTH + (DEEP_DEPTH - SHELF_DEPTH) * (x - SHELF_X) / SLOPE_X


def rig_sdf():
    cam = f'''<horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height>%s</image>
          <clip><near>0.05</near><far>60</far></clip>'''
    return f'''
    <!-- 偵測驗證的相機（tools/sim_detect/capture.py 加的）。靜態模型，用 set_pose 搬 -->
    <model name="sweep_rig">
      <static>true</static>
      <pose>0 0 -1 0 0 0</pose>
      <link name="cam">
        <sensor name="rgb" type="camera">
          <always_on>1</always_on><update_rate>{RATE}</update_rate><topic>/sweep/rgb</topic>
          <camera>{cam % '<format>R8G8B8</format>'}<visibility_mask>{RGB_MASK}</visibility_mask></camera>
        </sensor>
        <sensor name="depth" type="depth_camera">
          <always_on>1</always_on><update_rate>{RATE}</update_rate><topic>/sweep/depth</topic>
          <camera><horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH // 2}</width><height>{HEIGHT // 2}</height><format>R_FLOAT32</format></image>
            <clip><near>0.05</near><far>60</far></clip></camera>
        </sensor>
        <sensor name="seg" type="segmentation">
          <always_on>1</always_on><update_rate>{RATE}</update_rate><topic>/sweep/seg</topic>
          <camera><segmentation_type>semantic</segmentation_type>{cam % ''}<visibility_mask>{RGB_MASK}</visibility_mask></camera>
        </sensor>
      </link>
    </model>
'''


def build_world(src):
    """世界檔 → (sweep 用的世界文字, 世界名稱, arena 座標, {實例: (種類, (x, y, z) arena, label)})。"""
    text = re.sub(r'<!--.*?-->', '', open(src).read(), flags=re.S)       # 註解裡也有 <include> 字樣
    # 水：model://water 原樣放進來，但拿掉水體霧（particle）。segmentation camera 碰到 particle 會讓
    # gz server 當掉（Ogre2SegmentationCamera：ItemIdentityException）；rgb 相機本來就用 visibility_mask
    # 把 particle 排除（見 orca_cam），depth camera 也不含 particle，所以拿掉之後拍到的影像完全一樣。
    water = re.sub(r'<!--.*?-->', '', open(os.path.join(SIM_DIR, 'models', 'water', 'model.sdf')).read(), flags=re.S)
    water = re.search(r'<model name="water">.*?</model>', water, flags=re.S).group(0)
    water = re.sub(r'<link name="haze">.*?</link>', '', water, flags=re.S)
    text, n = re.subn(r'<include>\s*<uri>model://water</uri>(?:(?!</include>).)*?<pose([^>]*)>([^<]*)</pose>\s*</include>',
                      lambda m: water.replace('<model name="water">', f'<model name="water"><pose{m.group(1)}>{m.group(2)}</pose>', 1),
                      text, flags=re.S)
    text = re.sub(r'<plugin[^>]*gz-sim-particle-emitter-system[^>]*/>', '', text)
    # 拿掉船和球
    text = re.sub(r'<include>(?:(?!</include>).)*?model://(?:orca_cam|orca|dropper_ball)</uri>(?:(?!</include>).)*?</include>',
                  '', text, flags=re.S)
    instances = {}

    def label(m):
        block = m.group(0)
        kind = re.search(r'<uri>model://([^<]+)</uri>', block).group(1).strip('/')
        if kind in ('pool_2027', 'pool', 'water'):
            return block
        name = re.search(r'<name>([^<]+)</name>', block)
        name = name.group(1) if name else re.search(r'<model\s+name="([^"]+)"',
                                                   open(os.path.join(SIM_DIR, 'models', kind, 'model.sdf')).read()).group(1)
        pose = re.search(r'<pose[^>]*>([^<]+)</pose>', block)
        xyz = tuple(float(v) for v in pose.group(1).split()[:3]) if pose else (0.0, 0.0, 0.0)
        lab = len(instances) + 1
        instances[name] = (kind, xyz, lab)
        return block.replace('</include>', f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
                                           f'<label>{lab}</label></plugin></include>')

    text = re.sub(r'<include>.*?</include>', label, text, flags=re.S)
    text = text.replace('  </world>', rig_sdf() + '  </world>')
    root = ET.fromstring(text.encode())
    world = root.find('world')
    ax, ay, _, _, _, ayaw = [float(v) for v in world.find("frame[@name='arena']/pose").text.split()]
    return text, world.get('name'), (ax, ay, ayaw), instances


class Sweep:
    def __init__(self, world, arena, requester):
        self.world, self.arena = world, arena
        self.req = requester                # gz service 的請求在另一個行程送（見 sim/gzreq.py）
        self.node = Node()
        self.lock = threading.Lock()
        self.sim_t = None
        self.frames = {'rgb': {}, 'depth': {}, 'seg': {}}       # stamp → array（各留最近幾張）
        self.node.subscribe(Clock, f'/world/{world}/clock', self.on_clock)
        self.node.subscribe(GzImage, '/sweep/rgb', lambda m: self.on_image('rgb', m))
        self.node.subscribe(GzImage, '/sweep/depth', lambda m: self.on_image('depth', m))
        self.node.subscribe(GzImage, '/sweep/seg/labels_map', lambda m: self.on_image('seg', m))

    def on_clock(self, msg):
        self.sim_t = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_image(self, kind, msg):
        stamp = round(msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, 4)
        if kind == 'depth':
            a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
        else:
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.lock:
            buf = self.frames[kind]
            buf[stamp] = a
            for k in sorted(buf)[:-6]:
                del buf[k]

    def to_world(self, x, y, yaw):
        ax, ay, ayaw = self.arena
        return (ax + x * math.cos(ayaw) - y * math.sin(ayaw), ay + x * math.sin(ayaw) + y * math.cos(ayaw), yaw + ayaw)

    def set_pose(self, name, x, y, z, yaw):
        wx, wy, wyaw = self.to_world(x, y, yaw)
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = wx, wy, z
        req.orientation.w, req.orientation.z = math.cos(wyaw / 2), math.sin(wyaw / 2)
        for _ in range(3):              # 模擬忙的時候回應偶爾會晚到；set_pose 是冪等的，重試就好
            ok, rep = self.req.request(f'/world/{self.world}/set_pose', req, Boolean, 5000)
            if ok and rep.data:
                return True
        return False

    def remove(self, name):
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        ok, rep = self.req.request(f'/world/{self.world}/remove', req, Boolean, 5000)
        return ok and rep.data

    def grab(self, after, timeout=4.0):
        """等三台相機都有同一個時間戳、而且晚於 after 的影像。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                common = set(self.frames['rgb']) & set(self.frames['depth']) & set(self.frames['seg'])
                common = [s for s in common if s > after]
                if common:
                    s = max(common)
                    return s, self.frames['rgb'][s], self.frames['depth'][s], self.frames['seg'][s]
            time.sleep(0.01)
        return None


def plan(instances, props, quick):
    """位姿清單。"""
    dists, bearings, depths = QUICK if quick else (DISTANCES, BEARINGS, DEPTHS)
    centre = (POOL_X / 2, POOL_Y / 2)
    poses = []
    for kind in props:
        cands = [(math.dist(xyz[:2], centre), name) for name, (k, xyz, _) in instances.items() if k == kind]
        if not cands:
            print(f'世界裡沒有 {kind}，跳過', flush=True)
            continue
        name = min(cands)[1]
        _, (px, py, pz), lab = instances[name]
        labels, alt, aim, base = PROPS[kind]
        tz = pz + aim
        for d in dists:
            for b in bearings:
                a = math.radians(base + b)
                cx, cy = px + d * math.cos(a), py + d * math.sin(a)
                for depth in depths:
                    why = None
                    if not (POOL_MARGIN <= cx <= POOL_X - POOL_MARGIN and POOL_MARGIN <= cy <= POOL_Y - POOL_MARGIN):
                        why = 'outside_pool'
                    elif depth > floor_depth(cx) - FLOOR_MARGIN:
                        why = 'below_floor'
                    else:
                        for other, (_, oxyz, _) in instances.items():
                            if other != name and math.dist(oxyz[:2], (cx, cy)) < MIN_CLEARANCE:
                                why = f'too_close_to_{other}'
                    poses.append(dict(prop=kind, instance=name, label_id=lab, labels='|'.join(labels), alt_labels='|'.join(alt),
                                      distance=d, bearing=b, depth=depth, cam_x=round(cx, 3), cam_y=round(cy, 3),
                                      cam_z=-depth, yaw=round(math.atan2(py - cy, px - cx), 5),
                                      range3d=round(math.sqrt(d * d + (tz + depth) ** 2), 3), skip=why))
    return poses


def main():
    run = os.environ.get('RUN') or time.strftime('sweep_%Y%m%d_%H%M%S')
    out = os.path.join(os.environ.get('OUT_DIR', '/out'), run)
    world_file = os.environ.get('SWEEP_WORLD', 'sauvc_final.sdf')
    props = [p for p in os.environ.get('PROPS', '').split(',') if p] or list(PROPS)
    unknown = [p for p in props if p not in PROPS]
    if unknown:
        sys.exit(f'不認得的道具：{unknown}（可以用：{", ".join(PROPS)}）')
    quick = os.environ.get('QUICK', '0') == '1'
    strength = float(os.environ.get('UNDERWATER_STRENGTH', '1.0'))

    requester = Requester()                 # 要在建立任何 gz Node 之前
    text, world, arena, instances = build_world(os.path.join(SIM_DIR, 'worlds', world_file))
    poses = plan(instances, props, quick)
    todo = [p for p in poses if not p['skip']]
    print(f'{run}：{world_file}，{len(props)} 種道具，{len(todo)} 個位姿（跳過 {len(poses) - len(todo)}），兩種條件', flush=True)
    os.makedirs(os.path.join(out, 'frames', 'on'), exist_ok=True)
    os.makedirs(os.path.join(out, 'frames', 'off'), exist_ok=True)
    with open('/tmp/sweep.sdf', 'w') as f:
        f.write(text)

    server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '2', '/tmp/sweep.sdf'],
                              stdout=open(os.path.join(out, 'gz.log'), 'w'), stderr=subprocess.STDOUT)
    sweep = Sweep(world, arena, requester)
    try:
        t0 = time.monotonic()
        while sweep.grab(-1, timeout=1.0) is None:
            if server.poll() is not None or time.monotonic() - t0 > 120:
                sys.exit(f'gz server 沒起來，看 {out}/gz.log')
        uw = Underwater(HFOV, enabled=True, strength=strength, seed=12345)
        rows = []
        fails = 0
        for cond in ('on', 'off'):
            if cond == 'off' and not sweep.remove('water'):
                print('警告：移除 model://water 失敗（世界裡沒有水？）', flush=True)
            time.sleep(1.0)
            tc = time.monotonic()
            for i, p in enumerate(todo):
                if not sweep.set_pose('sweep_rig', p['cam_x'], p['cam_y'], p['cam_z'], p['yaw']):
                    sys.exit('set_pose 失敗')
                after = sweep.sim_t + 1.5 / RATE
                got = sweep.grab(after)
                if got is None:
                    fails += 1
                    print(f'  {cond} #{i}：等不到影像，跳過', flush=True)
                    continue
                _, rgb, depth, seg = got
                img = uw.apply(rgb, depth) if cond == 'on' else rgb
                fname = f'frames/{cond}/{i:04d}.jpg'
                cv2.imwrite(os.path.join(out, fname), cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
                mask = seg[:, :, 0] == p['label_id']
                n = int(mask.sum())
                if n:
                    ys, xs = np.nonzero(mask)
                    box = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
                else:
                    box = ('', '', '', '')
                rows.append(dict(id=i, cond=cond, **{k: v for k, v in p.items() if k != 'skip'}, visible_px=n,
                                 x1=box[0], y1=box[1], x2=box[2], y2=box[3], file=fname))
                if (i + 1) % 100 == 0:
                    rate = (i + 1) / (time.monotonic() - tc)
                    print(f'  {cond}：{i + 1}/{len(todo)}（{rate:.1f} 張/秒）', flush=True)
        with open(os.path.join(out, 'frames.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        meta = dict(run=run, world=world_file, created=time.strftime('%Y-%m-%d %H:%M:%S'), git=os.environ.get('GIT_REV', ''),
                    camera=dict(width=WIDTH, height=HEIGHT, hfov=HFOV), quick=quick,
                    distances=QUICK[0] if quick else DISTANCES, bearings=QUICK[1] if quick else BEARINGS,
                    depths=QUICK[2] if quick else DEPTHS,
                    props={k: dict(zip(('labels', 'alt_labels', 'aim_z', 'base_bearing'), PROPS[k])) for k in props},
                    underwater=dict(strength=uw.strength, atten=uw.atten.tolist(), scatter=uw.scatter.tolist(),
                                    veil=uw.veil.tolist(), tint=uw.tint.tolist(), blur=uw.blur, noise=uw.noise, seed=12345),
                    poses=len(todo), skipped={k: sum(1 for p in poses if p['skip'] and p['skip'].startswith(k))
                                              for k in ('outside_pool', 'below_floor', 'too_close')},
                    grab_failures=fails)
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        print(f'完成：{len(rows)} 張，存在 {out}', flush=True)
    finally:
        requester.close()
        server.terminate()
        try:
            server.wait(10)
        except subprocess.TimeoutExpired:
            server.kill()


if __name__ == '__main__':
    main()
