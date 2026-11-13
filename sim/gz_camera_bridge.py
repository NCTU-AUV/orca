#!/usr/bin/env python3
"""Gazebo 相機 → ROS，發布成跟實機 cam_bridge／realsense2_camera 一樣的 topic，下游不用改。

    訂閱（gz）  /orca/camera/image                gz.msgs.Image（sim/models/orca_cam 的前視相機）
    發布（ROS） /camera/camera/color/image_raw     sensor_msgs/Image（rgb8）
               /camera/camera/color/camera_info   sensor_msgs/CameraInfo（針孔、無畸變，內參由視角算）

在 orca-sim image 裡跑（有 gz-transport 的 Python 綁定和 ROS 2 Humble），見 deploy/sim/compose.yaml
的 camera 服務。不需要 ros_gz。

時間戳：收到影像時的 ROS 時鐘（主機時間）。模擬以約 1 倍速跑，跟實機 cam_bridge 一樣用主機時間。
參數（環境變數）：GZ_CAMERA_TOPIC（/orca/camera/image）、CAMERA_HFOV（0.9725 rad，要跟模型一致）
"""
import math
import os
import threading

import rclpy
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

GZ_TOPIC = os.environ.get('GZ_CAMERA_TOPIC', '/orca/camera/image')
HFOV = float(os.environ.get('CAMERA_HFOV', '0.9725'))
FRAME_ID = 'camera_color_optical_frame'
RGB_INT8 = 3        # gz.msgs.PixelFormatType


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera', namespace='camera')
        self.pub_img = self.create_publisher(Image, '~/color/image_raw', 1)
        self.pub_info = self.create_publisher(CameraInfo, '~/color/camera_info', 1)
        self.lock = threading.Lock()
        self.n = 0
        self.warned = False
        self.gz = GzNode()
        if not self.gz.subscribe(GzImage, GZ_TOPIC, self.on_gz_image):
            raise RuntimeError(f'訂閱 gz topic {GZ_TOPIC} 失敗')
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{GZ_TOPIC} → /camera/camera/color/image_raw、camera_info')

    def on_gz_image(self, msg):
        # gz-transport 的執行緒呼叫進來；rclpy 的 publish 可以跨執行緒用
        if msg.pixel_format_type != RGB_INT8:
            if not self.warned:
                self.warned = True
                self.get_logger().error(f'只支援 RGB_INT8，收到 pixel_format_type {msg.pixel_format_type}')
            return
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = FRAME_ID
        img.height, img.width = msg.height, msg.width
        img.encoding = 'rgb8'
        img.step = msg.step or msg.width * 3
        img.data.frombytes(msg.data)
        self.pub_img.publish(img)
        self.pub_info.publish(self.info(msg.width, msg.height, stamp))
        with self.lock:
            self.n += 1

    @staticmethod
    def info(width, height, stamp):
        f = (width / 2) / math.tan(HFOV / 2)
        cx, cy = width / 2, height / 2
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = FRAME_ID
        m.width, m.height = width, height
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def report(self):
        with self.lock:
            n, self.n = self.n, 0
        if n:
            self.get_logger().info(f'{n / 10:.1f} fps')
        else:
            self.get_logger().warn(f'最近 10 秒沒收到 {GZ_TOPIC}（世界檔有用 model://orca_cam 嗎？）')


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
