#!/usr/bin/env python3
"""驗收：船的浮沉、姿態跟 model://orca 原本的一樣（碰撞外框、相機、球沒有把浮力改壞）。

對著正在跑的模擬（任何世界），讀 Gazebo 的真值位姿：
    1. 靜置 10 秒：浮在水面的深度、roll／pitch，以及有沒有在漂（不該沉、不該翻）
    2. 用 15 N 的力把船慢慢壓到 0.8 m 深、放開：放開後又往下滑多少、多久浮回水面、上浮速度
       （−0.6 → −0.4 m 那一段，純粹由淨浮力與阻力決定）、過程中最大傾角（要自己回正）
ArduSub 在跑但沒解鎖，推進器不出力。結果印成一行 JSON，也可以用 CHECK_OUT 存檔。

    cd deploy/sim && WORLD=sauvc_final.sdf GUI=0 docker compose up -d simulator sim_referee
    docker compose exec sim_referee bash -c 'source /opt/ros/humble/setup.bash && python3 /sim/tests/float_check.py'

世界要有 ApplyLinkWrench（sauvc_*.sdf 有；orca_pool.sdf 沒有，只做第 1 步）。
"""
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def main():
    world_file = os.environ.get('WORLD', 'orca_pool.sdf')         # sim/worlds 底下的檔名，或世界檔的完整路徑
    path = world_file if '/' in world_file else os.path.join(SIM_DIR, 'worlds', world_file)
    root = ET.parse(path).getroot().find('world')
    world = root.get('name')
    has_wrench = any('apply-link-wrench' in (p.get('filename') or '') for p in root.findall('plugin'))
    node = Node()
    hist = []

    def cb(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == 'orca':
                o = p.orientation
                hist.append((t, p.position.z, *rpy((o.w, o.x, o.y, o.z)), p.position.x, p.position.y))

    node.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', cb)
    time.sleep(1.0)
    if not hist:
        sys.exit(f'沒收到 /world/{world}/dynamic_pose/info：模擬有在跑嗎？')
    n0 = len(hist)
    time.sleep(10.0)
    rest = hist[n0:]
    z = [h[1] for h in rest]
    out = dict(world=world_file, rest_z_mean=round(sum(z) / len(z), 4), rest_z_range=round(max(z) - min(z), 4),
               rest_roll_max=round(max(abs(h[2]) for h in rest), 2), rest_pitch_max=round(max(abs(h[3]) for h in rest), 2),
               rest_drift_xy=round(math.hypot(rest[-1][4] - rest[0][4], rest[-1][5] - rest[0][5]), 3))
    if has_wrench:
        wp = node.advertise(f'/world/{world}/wrench/persistent', EntityWrench)
        wc = node.advertise(f'/world/{world}/wrench/clear', Entity)
        time.sleep(0.5)
        m = EntityWrench()
        m.entity.name = 'orca::base_link'
        m.entity.type = Entity.LINK
        m.wrench.force.z = -15.0
        wp.publish(m)
        t_end = time.monotonic() + 20
        while time.monotonic() < t_end and hist[-1][1] > -0.8:
            time.sleep(0.05)
        e = Entity()
        e.name = 'orca::base_link'
        e.type = Entity.LINK
        wc.publish(e)
        n1 = len(hist)
        t_rel, z_rel = hist[-1][0], hist[-1][1]
        time.sleep(15.0)
        after = hist[n1:]
        up = next((h for h in after if h[1] > out['rest_z_mean'] - 0.02), None)
        a = next((h for h in after if h[1] > -0.6), None)
        b = next((h for h in after if h[1] > -0.4), None)
        out.update(pushed_to_z=round(z_rel, 3), lowest_z=round(min(h[1] for h in after), 3),
                   rise_speed=round(0.2 / (b[0] - a[0]), 3) if a and b and b[0] > a[0] else None,
                   resurface_s=round(up[0] - t_rel, 2) if up else None,
                   rise_tilt_max=round(max(max(abs(h[2]), abs(h[3])) for h in after), 2),
                   final_z=round(after[-1][1], 4), final_roll=round(after[-1][2], 2), final_pitch=round(after[-1][3], 2))
    print(json.dumps(out, ensure_ascii=False))
    if os.environ.get('CHECK_OUT'):
        with open(os.environ['CHECK_OUT'], 'w') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
