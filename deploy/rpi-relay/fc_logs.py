#!/usr/bin/env python3
"""在 RPi 上跑：用 MAVFTP 列出、下載飛控 SD 卡上的 DataFlash 紀錄（/APM/LOGS/*.BIN）。
筆電的 deploy/common.sh 經 ssh 呼叫（down.sh、fc_logs.sh），平常不用手動跑。

    ~/mavproxy-venv/bin/python ~/workspace/fc-logs/fc_logs.py list
    ~/mavproxy-venv/bin/python ~/workspace/fc-logs/fc_logs.py get <目的資料夾> <編號>...

- 經 mavproxy.service 的 --out=udp:127.0.0.1:14551 跟飛控對話。這個輸出原本給 RPi 上的 mavros，
  那個容器停用後沒人用；同一時間只能有一個程式綁這個埠，所以兩個指令不能同時跑。
- 用 mavproxy-venv 裡的 pymavlink（MAVFTP）。2026-11-19 實測約 420 KiB/s，跟 LOG_REQUEST_DATA
  （約 370 KiB/s）差不多，兩種下載的檔逐位元組相同。
- 飛控解鎖時拒絕下載：ArduPilot 解鎖時本來就不給下載 log，而且下載會佔用 USB 的頻寬。
- 上鎖後 15 秒內飛控還在寫（ArduPilot 上鎖後會多記 15 秒），所以先等要下載的檔大小 2 秒內不變
  （最多 30 秒）再下載，免得拉到寫一半的檔。
- stdout：list 每行「編號 大小」，get 每行「編號 大小 秒數」。其他訊息、錯誤在 stderr。
"""
import argparse
import logging
import os
import re
import sys
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, FtpError, MAVFTPSettings

DEVICE = 'udpin:127.0.0.1:14551'
LOG_DIR = '/APM/LOGS'
FC_SYS, FC_COMP = 1, 1
# mavros 是 255/240，用別的 component，MAVFTP 只收發給自己的回覆
SRC_SYS, SRC_COMP = 255, 191


def connect():
    """連上飛控，回傳 (連線, 是否解鎖)。"""
    m = mavutil.mavlink_connection(DEVICE, source_system=SRC_SYS, source_component=SRC_COMP)
    end = time.time() + 10
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == FC_SYS and hb.get_srcComponent() == FC_COMP:
            m.target_system, m.target_component = FC_SYS, FC_COMP
            return m, bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    sys.exit('10 秒內沒收到飛控的心跳：mavproxy.service 有在跑、飛控有接上嗎？'
             '（或另一個 fc_logs.py 正在用 14551）')


def list_logs(ftp):
    ret = ftp.cmd_list([LOG_DIR])
    if ret.error_code != FtpError.Success:
        sys.exit(f'列不出 {LOG_DIR}（{ret.error_code.name}）')
    logs = [(int(e.name[:8]), e.size_b) for e in ftp.list_result
            if not e.is_dir and re.fullmatch(r'\d{8}\.BIN', e.name)]
    return sorted(logs)


def wait_stable(ftp, nums, timeout=30.0):
    prev = None
    end = time.time() + timeout
    while True:
        sizes = {n: s for n, s in list_logs(ftp) if n in nums}
        if sizes == prev:
            return
        if time.time() > end:
            print(f'{timeout:.0f} 秒內檔案大小還在變，照樣下載', file=sys.stderr)
            return
        prev = sizes
        time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description='列出、下載飛控的 .BIN')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list', help='列出 SD 卡上的 .BIN')
    g = sub.add_parser('get', help='下載 .BIN')
    g.add_argument('dest', help='存到 RPi 上的這個資料夾')
    g.add_argument('nums', type=int, nargs='+', help='log 編號（檔名的數字）')
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING, format='%(message)s')   # pymavlink 的 INFO 太多
    m, armed = connect()
    if args.cmd == 'get' and armed:
        sys.exit('飛控解鎖中，不下載。上鎖後再試')
    # 跟 MAVFTP 的預設一樣，只把 burst_read_size 從 80 加到上限 239：實測 1.5 MB 的檔 5.4 s → 3.7 s
    settings = MAVFTPSettings([
        ('debug', int, 0), ('pkt_loss_tx', int, 0), ('pkt_loss_rx', int, 0), ('max_backlog', int, 5),
        ('burst_read_size', int, 239), ('write_size', int, 80), ('write_qsize', int, 5),
        ('idle_detection_time', float, 3.7), ('read_retry_time', float, 1.0), ('retry_time', float, 0.5),
    ])
    ftp = MAVFTP(m, target_system=FC_SYS, target_component=FC_COMP, settings=settings)

    if args.cmd == 'list':
        for num, size in list_logs(ftp):
            print(num, size)
        return 0

    os.makedirs(args.dest, exist_ok=True)
    wait_stable(ftp, set(args.nums))
    fail = 0
    for num in args.nums:
        local = os.path.join(args.dest, f'{num:08d}.BIN')
        t0 = time.time()
        ftp.cmd_get([f'{LOG_DIR}/{num:08d}.BIN', local])
        # timeout=0：不限總時間（大檔要好幾分鐘），卡住由 MAVFTP 自己的閒置偵測處理
        ret = ftp.process_ftp_reply('get', timeout=0)
        if ret.error_code != FtpError.Success or not os.path.exists(local):
            print(f'{num:08d}.BIN 下載失敗（{ret.error_code.name}）', file=sys.stderr)
            fail += 1
            continue
        print(num, os.path.getsize(local), f'{time.time() - t0:.1f}', flush=True)
    m.close()
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
