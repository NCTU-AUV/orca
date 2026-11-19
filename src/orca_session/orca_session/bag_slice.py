"""bag_slice：從場次的 rosbag 切出一段，另存成獨立的 bag（一個 .mcap）。

    ros2 run orca_session bag_slice /sessions/<場次> --match gate_pass_20261120_101500
    ros2 run orca_session bag_slice /sessions/<場次> --match 'keyboard: #3' --duration 60
    ros2 run orca_session bag_slice /sessions/<場次> --start 10:15 --end 10:18:30

- --match：markers.csv 裡「來源: 內容」含這段文字的標記，從第一筆切到最後一筆。
  行為樹、測試腳本開始和結束都有標記，給 run 的名稱（<樹>_<時間>）就是整次任務。
  只找到一筆時要再給 --duration（從那筆往後切幾秒）。
- --start／--end：當天的時刻 HH:MM[:SS]，或 Unix 時間（秒）。
- 前後各多切 --pad 秒（預設 2）。
- 讀場次底下所有 bag_*/ 的 .mcap（recorder 每次啟動一個資料夾、每 5 分鐘切一個檔），
  輸出到 <場次>/slices/<名稱>/（<名稱>_0.mcap 加 metadata.yaml），名稱預設取自 --match 或時間範圍。
- 正在錄的 .mcap 讀得到，但最後幾秒還在 recorder 的緩衝區、沒寫進檔案（剛開的檔則完全讀不到）。
  Humble 的 recorder 沒有「現在就切檔」的服務，所以要切的範圍超出已寫進檔案的資料時不產生切片
  （免得留下一份不完整、看起來又像完整的檔），等一下或 down.sh 之後再切，或加 --partial。
  被強制結束（沒收尾）的檔通常還讀得到，只是比較慢。
"""
import argparse
import csv
import datetime
import glob
import os
import re
import sys

import rosbag2_py

NS = 1_000_000_000


def markers(session):
    path = os.path.join(session, 'markers.csv')
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return [(float(r['unix_time']), f"{r['source']}: {r['text']}") for r in csv.DictReader(f)]


def parse_time(text, session):
    """HH:MM[:SS] → 場次那天的那個時刻（早於場次開始就當成隔天）；純數字 → Unix 時間。回傳秒。"""
    if re.fullmatch(r'\d+(\.\d+)?', text):
        return float(text)
    m = re.fullmatch(r'(\d{1,2}):(\d{2})(?::(\d{2}(?:\.\d+)?))?', text)
    if not m:
        raise ValueError(f'看不懂的時間：{text}（用 HH:MM[:SS] 或 Unix 秒數）')
    start = datetime.datetime.strptime(os.path.basename(os.path.normpath(session))[:15], '%Y%m%d_%H%M%S')
    sec = float(m.group(3) or 0)
    t = start.replace(hour=int(m.group(1)), minute=int(m.group(2)), second=int(sec),
                      microsecond=int(sec % 1 * 1e6))
    if t < start - datetime.timedelta(hours=1):
        t += datetime.timedelta(days=1)
    return t.timestamp()


def time_range(args):
    """回傳 (開始, 結束, 預設名稱)，單位秒。"""
    if args.match:
        hits = [t for t, text in markers(args.session) if args.match in text]
        if not hits:
            raise ValueError(f'markers.csv 裡沒有含「{args.match}」的標記')
        start, end = hits[0], hits[-1]
        if args.duration is not None:
            end = start + args.duration
        elif len(hits) == 1:
            raise ValueError(f'只找到一筆含「{args.match}」的標記，要再給 --duration')
        return start, end, args.match
    if not (args.start and (args.end or args.duration is not None)):
        raise ValueError('要給 --match，或 --start 加上 --end／--duration')
    start = parse_time(args.start, args.session)
    end = start + args.duration if args.duration is not None else parse_time(args.end, args.session)
    name = (datetime.datetime.fromtimestamp(start).strftime('%H%M%S') + '-'
            + datetime.datetime.fromtimestamp(end).strftime('%H%M%S'))
    return start, end, name


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    return reader


def file_range(reader):
    """這個檔的時間範圍（ns）。"""
    meta = reader.get_metadata()
    start = meta.starting_time.nanoseconds
    return start, start + meta.duration.nanoseconds


def dir_start(path):
    """bag_<YYYYmmdd_HHMMSS>/ 的開始時間（ns）。"""
    name = os.path.basename(os.path.dirname(path))
    try:
        return datetime.datetime.strptime(name[4:19], '%Y%m%d_%H%M%S').timestamp() * NS
    except ValueError:
        return 0


def scan(files):
    """打開每個檔。回傳 (讀得到的 [(路徑, reader, 開始, 結束)], 讀不到的 [(路徑, 錯誤, 估計開始, 估計結束)])。

    讀不到的檔不知道涵蓋哪段時間，用前後的檔估：從同資料夾前一個檔的結尾（或資料夾的開始時間）
    到同資料夾下一個檔的開頭（或下一個資料夾的開始時間；都沒有就是到現在還在錄）。
    """
    opened = []
    for path in files:
        try:
            reader = open_reader(path)
            opened.append((path, reader, *file_range(reader)))
        except Exception as e:     # rosbag2 對壞檔丟的例外型別不固定
            opened.append((path, e, None, None))
    readable, broken = [], []
    for i, (path, reader, f_start, f_end) in enumerate(opened):
        if f_start is not None:
            readable.append((path, reader, f_start, f_end))
            continue
        same_dir = lambda j: os.path.dirname(opened[j][0]) == os.path.dirname(path)  # noqa: E731
        prev = opened[i - 1] if i > 0 and same_dir(i - 1) else None
        nxt = opened[i + 1] if i + 1 < len(opened) else None
        b_start = prev[3] if prev and prev[3] is not None else dir_start(path)
        if nxt is None:
            b_end = float('inf')
        elif same_dir(i + 1):
            b_end = nxt[2] if nxt[2] is not None else float('inf')
        else:
            b_end = dir_start(nxt[0])
        broken.append((path, reader, b_start, b_end))
    return readable, broken


def main():
    ap = argparse.ArgumentParser(description='從場次的 rosbag 切出一段')
    ap.add_argument('session', help='場次資料夾，例如 /sessions/20261120_093000')
    ap.add_argument('--match', help='標記含這段文字：從第一筆切到最後一筆')
    ap.add_argument('--start', help='開始：HH:MM[:SS] 或 Unix 秒數')
    ap.add_argument('--end', help='結束：HH:MM[:SS] 或 Unix 秒數')
    ap.add_argument('--duration', type=float, help='從開始往後切幾秒（代替 --end）')
    ap.add_argument('--pad', type=float, default=2.0, help='前後各多切幾秒（預設 2）')
    ap.add_argument('--name', help='輸出的名稱（預設取自 --match 或時間範圍）')
    ap.add_argument('--partial', action='store_true',
                    help='最新的 bag 還在錄時，只切到已經寫進檔案的部分（預設拒絕，免得切出不完整的檔）')
    args = ap.parse_args()

    try:
        start, end, name = time_range(args)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2
    start_ns, end_ns = int((start - args.pad) * NS), int((end + args.pad) * NS)
    name = re.sub(r'[^\w.-]+', '_', args.name or name).strip('_') or 'slice'
    out = os.path.join(args.session, 'slices', name)
    if os.path.exists(out):
        print(f'{out} 已經存在，換個 --name 或先刪掉', file=sys.stderr)
        return 2

    files = sorted(glob.glob(os.path.join(args.session, 'bag_*', '*.mcap')))
    if not files:
        print(f'{args.session} 底下沒有 bag_*/*.mcap', file=sys.stderr)
        return 2

    readable, broken = scan(files)
    hit = [(path, err) for path, err, b_start, b_end in broken if b_start <= end_ns and b_end >= start_ns]
    if hit:
        print('要切的範圍有一部分在讀不到的檔裡：', file=sys.stderr)
        for path, err in hit:
            print(f'  {os.path.relpath(path, args.session)}（{err}）', file=sys.stderr)
        print('最新的檔還在錄的話，等它關檔（最多 5 分鐘）或 ./down.sh 之後再切', file=sys.stderr)
        return 1
    # 還在錄的檔讀得到，但最後幾秒還在 recorder 的緩衝區、沒寫進檔案
    last_dir = os.path.dirname(files[-1])
    covered = max(f_end for *_, f_end in readable) if readable else 0
    if (not os.path.exists(os.path.join(last_dir, 'metadata.yaml')) and covered < end_ns
            and not args.partial):
        print(f'最新的 bag（{os.path.basename(last_dir)}）還在錄或沒正常收尾，寫進檔案的資料只到 '
              f'{datetime.datetime.fromtimestamp(covered / NS).strftime("%H:%M:%S")}。'
              '等一下或 ./down.sh 之後再切；確定只要到目前為止的部分就加 --partial', file=sys.stderr)
        return 1

    writer = None
    created = set()
    count = 0
    for path, reader, f_start, f_end in readable:
        if f_end < start_ns or f_start > end_ns:
            continue
        topics = {t.name: t for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, t = reader.read_next()
            if t < start_ns or t > end_ns:
                continue
            if writer is None:
                os.makedirs(os.path.dirname(out), exist_ok=True)
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'),
                            rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                        output_serialization_format='cdr'))
            if topic not in created:
                writer.create_topic(topics[topic])
                created.add(topic)
            writer.write(topic, data, t)
            count += 1

    if writer is None:
        print('這段時間沒有任何訊息', file=sys.stderr)
        return 1
    del writer      # 關檔，寫 metadata.yaml
    fmt = '%H:%M:%S'
    print(f'{datetime.datetime.fromtimestamp(start_ns / NS).strftime(fmt)} ～ '
          f'{datetime.datetime.fromtimestamp(end_ns / NS).strftime(fmt)}，'
          f'{len(created)} 個 topic、{count} 筆訊息 → {out}/')
    return 0


if __name__ == '__main__':
    sys.exit(main())
