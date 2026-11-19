"""mark：從指令列發一筆標記（deploy/<跑法>/mark.sh 用的就是這個）。

    ros2 run orca_session mark surge 0.4 第 2 趟
    ros2 run orca_session mark --source up.sh 場次開始

先等 recorder、marker_log 訂閱上（最多 3 秒）再發，發完等一下讓訊息送出去才結束。
一個訂閱者都沒有時回傳 1：這筆標記沒被記下來。
"""
import argparse
import os
import sys
import time

import rclpy

from orca_session.session import MARKER_SUBSCRIBERS, MARKER_TOPIC, Marker


def main():
    ap = argparse.ArgumentParser(description='發一筆標記到 /session/marker')
    ap.add_argument('text', nargs='+', help='標記的內容')
    ap.add_argument('--source', default='mark', help='來源（預設 mark）')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node(f'mark_{os.getpid()}')
    marker = Marker(node, args.source)
    n = marker.wait_subscribers(3.0)
    marker(' '.join(args.text))
    time.sleep(0.5)
    node.destroy_node()
    rclpy.try_shutdown()
    if n == 0:
        print(f'沒有人訂閱 {MARKER_TOPIC}（recorder、marker_log 沒在跑？），這筆標記沒記下來',
              file=sys.stderr)
        return 1
    if n < MARKER_SUBSCRIBERS:
        print(f'只有 {n} 個訂閱者（應該有 recorder、marker_log 兩個），標記可能只進了其中一邊',
              file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
