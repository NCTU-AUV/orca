"""marker_log：把 /session/marker 的每一筆標記附加到目前場次的 markers.csv。

    ros2 run orca_session marker_log

- 欄位：unix_time、local_time、source（來源）、text（內容）。標記的格式是「<來源>: <內容>」。
- 各程式只發 topic（同時錄進 rosbag），這個節點是 markers.csv 唯一的寫入者，
  不會有好幾個程式同時寫同一個檔。
- 每收到一筆才讀 /sessions/current，換場次不用重開。沒有場次時只印警告。
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from orca_session.session import MARKER_QOS, MARKER_TOPIC, SESSIONS_DIR, current

HEADER = ['unix_time', 'local_time', 'source', 'text']


class MarkerLog(Node):
    def __init__(self):
        super().__init__('marker_log')
        self.create_subscription(String, MARKER_TOPIC, self.on_marker, MARKER_QOS)
        self.get_logger().info(f'把 {MARKER_TOPIC} 寫進 {SESSIONS_DIR}/<場次>/markers.csv')

    def on_marker(self, msg):
        now = time.time()
        source, sep, text = msg.data.partition(': ')
        if not sep:
            source, text = '', msg.data
        name = current()
        if name is None:
            self.get_logger().warn(f'沒有進行中的場次，標記沒寫進 markers.csv：{msg.data}')
            return
        path = os.path.join(SESSIONS_DIR, name, 'markers.csv')
        new = not os.path.exists(path)
        local = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)) + f'.{int(now % 1 * 1000):03d}'
        with open(path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(HEADER)
            w.writerow([f'{now:.3f}', local, source, text])


def main():
    rclpy.init()
    node = MarkerLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
