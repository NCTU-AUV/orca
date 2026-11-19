#!/usr/bin/env python3
"""bin_time：把場次的飛控 DataFlash（.BIN）轉成 CSV，加上 ROS 時間（跟 rosbag、markers.csv 同一個時鐘）。

    python3 bin_time.py /sessions/<場次>                      # fc_logs/ 底下全部的 .BIN
    python3 bin_time.py /sessions/<場次> --bin 00000014.BIN --types ATT,RCOU
    python3 bin_time.py /sessions/<場次> --types all

平常用 deploy/laptop/fc_logs.sh csv <場次>，它在 orca-sim 映像裡跑這支（要 pymavlink）。

- 要先有 <場次>/info/fc_time.csv（ros2 run orca_session fc_time，down.sh 會產生）。
- 輸出到 <場次>/fc_logs/<編號>/<類型>.csv，欄位是 ros_time 加上那種訊息的全部欄位（含 TimeUS）。
  有好幾個實例的訊息（IMU、BARO 等）用 I 或 Instance 欄區分，同一個檔裡。
- 對應 fc_time.csv 的哪一段：看 ATT 的 TimeUS 落在哪一段的開機時間範圍裡最多。
  找不到、或飛控在場次中重開機而兩段都對得上時跳過，要用 --segment 指定。
- 預設只輸出落在那一段開機時間範圍（前後各多 1 秒）的資料。LOG_FILE_DSRMROT 0 的時候，
  飛控沒重開機就會一直寫同一個檔，裡面可能有上一個場次的資料。--all 全部輸出（範圍外用同一條直線外插）。
- ArduSub 4.7.2 寫的 RATE 的 TimeUS 一律是 0，這裡改用前一筆 ATT 的時間（同一次呼叫先寫 ATT 再寫 RATE）。
"""
import argparse
import csv
import glob
import os
import sys

from pymavlink import mavutil

# 系統辨識常用的：姿態、角速度、IMU、馬達輸出、深度計、控制器、電池，以及事件
DEFAULT_TYPES = ['ATT', 'RATE', 'IMU', 'RCOU', 'BARO', 'CTUN', 'MOTB', 'BAT', 'ARM', 'EV', 'MODE', 'MSG', 'ERR']
SKIP = {'FMT', 'FMTU', 'MULT', 'UNIT'}
MARGIN_S = 1.0


def load_segments(path):
    with open(path, newline='') as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def read_bin(path, types):
    """回傳 ({類型: (欄位名, [資料列])}, [ATT 的 TimeUS])。"""
    m = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    want = None if types is None else set(types) | {'ATT'}
    data = {}
    att_us = []
    last_us = 0
    while True:
        msg = m.recv_match(type=list(want) if want else None)
        if msg is None:
            break
        t = msg.get_type()
        if t in SKIP or t == 'BAD_DATA':
            continue
        us = getattr(msg, 'TimeUS', None)
        if us:
            last_us = us
        if t == 'ATT':
            att_us.append(us)
        if types is not None and t not in types:
            continue
        if t not in data:
            data[t] = (list(msg.get_fieldnames()), [])
        d = msg.to_dict()
        row = [d.get(k) for k in data[t][0]]
        data[t][1].append((us or last_us, row))
    return data, att_us


def pick_segment(segs, att_us):
    counts = []
    for s in segs:
        lo, hi = s['boot_start_s'] - MARGIN_S, s['boot_end_s'] + MARGIN_S
        counts.append(sum(1 for us in att_us if lo <= us / 1e6 <= hi))
    best = max(counts) if counts else 0
    if best == 0 or counts.count(best) > 1:
        return None, counts
    return segs[counts.index(best)], counts


def main():
    ap = argparse.ArgumentParser(description='.BIN → 帶 ROS 時間的 CSV')
    ap.add_argument('session', help='場次資料夾，例如 /sessions/20261120_093000')
    ap.add_argument('--bin', nargs='*', help='只轉這些檔（fc_logs/ 底下的檔名），預設全部')
    ap.add_argument('--types', default=','.join(DEFAULT_TYPES),
                    help=f'要哪些訊息，逗號分隔，或 all（預設 {",".join(DEFAULT_TYPES)}）')
    ap.add_argument('--segment', type=int, help='指定用 fc_time.csv 的第幾段')
    ap.add_argument('--all', action='store_true', help='開機時間範圍外的資料也輸出')
    args = ap.parse_args()

    tpath = os.path.join(args.session, 'info', 'fc_time.csv')
    if not os.path.exists(tpath):
        print(f'沒有 {tpath}，先跑 fc_time（./fc_logs.sh time <場次>）', file=sys.stderr)
        return 1
    segs = load_segments(tpath)
    logdir = os.path.join(args.session, 'fc_logs')
    files = ([os.path.join(logdir, b) for b in args.bin] if args.bin
             else sorted(glob.glob(os.path.join(logdir, '*.BIN'))))
    if not files:
        print(f'{logdir} 底下沒有 .BIN', file=sys.stderr)
        return 1
    types = None if args.types == 'all' else [t.strip() for t in args.types.split(',') if t.strip()]

    fail = 0
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        data, att_us = read_bin(path, types)
        if args.segment:
            seg = next((s for s in segs if int(s['segment']) == args.segment), None)
            counts = []
        else:
            seg, counts = pick_segment(segs, att_us)
        if seg is None:
            print(f'{name}.BIN：對不到 fc_time.csv 的哪一段（每段的 ATT 筆數 {counts}），'
                  '用 --segment 指定', file=sys.stderr)
            fail += 1
            continue
        lo, hi = seg['boot_start_s'] - MARGIN_S, seg['boot_end_s'] + MARGIN_S
        off, k = seg['offset_s'], 1 + seg['drift_ppm'] * 1e-6
        out = os.path.join(logdir, name)
        os.makedirs(out, exist_ok=True)
        dropped = 0
        for t, (fields, rows) in sorted(data.items()):
            with open(os.path.join(out, f'{t}.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['ros_time'] + fields)
                for us, row in rows:
                    s = us / 1e6
                    if not args.all and not lo <= s <= hi:
                        dropped += 1
                        continue
                    w.writerow([f'{off + k * s:.6f}'] + row)
        print(f'{name}.BIN → {out}/（第 {int(seg["segment"])} 段，{len(data)} 種訊息'
              + (f'，範圍外 {dropped} 筆沒輸出' if dropped else '') + '）')
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
