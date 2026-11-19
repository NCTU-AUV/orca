"""fc_time：從場次的 rosbag 算出「飛控開機時間 → ROS 時間」的對照，存成 <場次>/info/fc_time.csv。

    ros2 run orca_session fc_time /sessions/<場次>

為什麼要這樣做：船上沒有 GPS，飛控也沒有 RTC，DataFlash（.BIN）裡只有開機後的微秒數（TimeUS），
跟 rosbag、markers.csv 的時間（ROS 時間 = 筆電的系統時間）對不起來。
bag 錄了飛控送來的原始 MAVLink（/uas1/mavlink_source），其中 ATTITUDE 帶著飛控同一個時鐘的
time_boot_ms，mavros 收到時又蓋上 ROS 時間（header.stamp）。兩者相減 = 時鐘偏移 + 傳輸延遲。
延遲只會讓差值變大，所以每 10 秒取差值最小（延遲最小）的那筆，再對這些點做直線擬合：
截距是偏移，斜率是飛控時鐘相對筆電時鐘的漂移。不用改飛控參數，也不用 mavros 的 timesync。

- 換算：ros_time = offset_s + (1 + drift_ppm * 1e-6) * TimeUS / 1e6
- 飛控重開機（time_boot_ms 變小）或差值跳超過 0.5 秒時分段，每段一列。
- 欄位：segment、boot_start_s／boot_end_s（這段在 bag 裡的開機時間範圍）、ros_start／ros_end、
  offset_s、drift_ppm、resid_ms（每 10 秒最小值偏離直線的最大量）、samples（ATTITUDE 筆數）。
- 2026-11-19 實機（遙測 20 Hz）：殘差 ±0.5 ms、漂移 3 ppm。.BIN 的解鎖、上鎖換算後都落在 mavros
  送出指令到收到 COMMAND_ACK 之間，比收到 ACK 早 0.2～0.4 ms，所以誤差最多 5.6 ms（那次 ACK 的來回時間），
  實際約 1 ms。
- 只讀 /uas1/mavlink_source，影像不用解開：4 分鐘的 bag 約 2 秒。
"""
import argparse
import glob
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from mavros_msgs.msg import Mavlink

TOPIC = '/uas1/mavlink_source'
FC_SYS, FC_COMP = 1, 1
MSG_ATTITUDE = 30
WINDOW_S = 10.0         # 每段時間取一次最小值
JUMP_S = 0.5            # 差值跳這麼多就當成另一次開機
FIELDS = ['segment', 'boot_start_s', 'boot_end_s', 'ros_start', 'ros_end', 'offset_s', 'drift_ppm',
          'resid_ms', 'samples']


def read_points(session):
    """回傳 [(ROS 時間, 開機時間)]（秒），照 ROS 時間排序。"""
    pts = []
    for path in sorted(glob.glob(os.path.join(session, 'bag_*', '*.mcap'))):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                    output_serialization_format='cdr'))
            reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
            while reader.has_next():
                _, data, t = reader.read_next()
                m = deserialize_message(data, Mavlink)
                if m.msgid != MSG_ATTITUDE or m.sysid != FC_SYS or m.compid != FC_COMP or not m.payload64:
                    continue
                # payload 是小端序，time_boot_ms（uint32）是第一個欄位
                boot = (m.payload64[0] & 0xFFFFFFFF) / 1e3
                stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
                pts.append((stamp if stamp > 0 else t / 1e9, boot))
        except Exception as e:     # rosbag2 對壞檔丟的例外型別不固定
            print(f'  跳過讀不到的檔 {os.path.relpath(path, session)}（{e}）', file=sys.stderr)
    pts.sort()
    return pts


def segments(pts):
    seg = []
    for ros, boot in pts:
        if seg:
            pr, pb = seg[-1]
            if boot < pb - 1.0 or abs((ros - boot) - (pr - pb)) > JUMP_S:
                yield seg
                seg = []
        seg.append((ros, boot))
    if seg:
        yield seg


def fit(seg):
    """下包絡直線：每 WINDOW_S 秒（開機時間）取 d = ros - boot 的最小值，再最小平方。回傳 (截距, 斜率, 殘差)。"""
    mins = {}
    for ros, boot in seg:
        k = int(boot // WINDOW_S)
        d = ros - boot
        if k not in mins or d < mins[k][1]:
            mins[k] = (boot, d)
    xs = [b for b, _ in mins.values()]
    ys = [d for _, d in mins.values()]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if n >= 2 and sxx > 0 else 0.0
    icpt = my - slope * mx if n >= 2 else min(ys)
    resid = max(abs(y - (icpt + slope * x)) for x, y in zip(xs, ys))
    return icpt, slope, resid


def main():
    ap = argparse.ArgumentParser(description='飛控開機時間 → ROS 時間的對照')
    ap.add_argument('session', help='場次資料夾，例如 /sessions/20261120_093000')
    ap.add_argument('--out', help='輸出檔（預設 <場次>/info/fc_time.csv）')
    args = ap.parse_args()

    pts = read_points(args.session)
    if not pts:
        print(f'{args.session} 的 bag 裡沒有飛控的 ATTITUDE（{TOPIC}）', file=sys.stderr)
        return 1
    rows = []
    for i, seg in enumerate(segments(pts), 1):
        icpt, slope, resid = fit(seg)
        rows.append([i, f'{seg[0][1]:.3f}', f'{seg[-1][1]:.3f}', f'{seg[0][0]:.3f}', f'{seg[-1][0]:.3f}',
                     f'{icpt:.6f}', f'{slope * 1e6:.2f}', f'{resid * 1e3:.2f}', len(seg)])
        print(f'  第 {i} 段：開機後 {seg[0][1]:.0f}～{seg[-1][1]:.0f} s，{len(seg)} 筆，'
              f'漂移 {slope * 1e6:.1f} ppm，殘差 {resid * 1e3:.2f} ms')
    out = args.out or os.path.join(args.session, 'info', 'fc_time.csv')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(','.join(FIELDS) + '\n')
        for r in rows:
            f.write(','.join(str(v) for v in r) + '\n')
    print(f'飛控時間對照 → {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
