#!/usr/bin/env python3
"""Gazebo 相機 → ROS，發布成跟實機一樣的 topic，下游不用改。

    gz topic（sim/models/orca_cam）        ROS topic                                  frame_id
    /orca/camera/image          前視  →  /camera/camera/color/image_raw、camera_info   camera_color_optical_frame
    /orca/bottom_camera/image   底部  →  /bottom_camera/image_raw、camera_info          bottom_camera_optical_frame

影像 sensor_msgs/Image（rgb8）；camera_info 是針孔、無畸變，內參由視角算。
frame_id 跟 src/orca_mission/config/orca.yaml 一致：偵測靠它分辨是哪一台相機。

在 orca-sim image 裡跑（有 gz-transport 的 Python 綁定和 ROS 2 Humble），見 deploy/sim/compose.yaml
的 sim_cameras 服務。不需要 ros_gz。世界檔沒有某台相機時，那台就不會有影像，其他照常。

時間戳：收到影像時的 ROS 時鐘（主機時間）。模擬以約 1 倍速跑，跟實機 cam_bridge 一樣用主機時間。
視角要跟模型一致：環境變數 CAMERA_HFOV（前視，0.9725 rad）、BOTTOM_CAMERA_HFOV（底部，1.2217 rad）。
"""
import math
import os
import threading

import rclpy
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

RGB_INT8 = 3        # gz.msgs.PixelFormatType

# (名稱, gz topic, ROS 影像 topic, ROS camera_info topic, frame_id, 水平視角 rad)
CAMERAS = [
    ('front', os.environ.get('GZ_CAMERA_TOPIC', '/orca/camera/image'),
     '/camera/camera/color/image_raw', '/camera/camera/color/camera_info',
     'camera_color_optical_frame', float(os.environ.get('CAMERA_HFOV', '0.9725'))),
    ('bottom', os.environ.get('GZ_BOTTOM_CAMERA_TOPIC', '/orca/bottom_camera/image'),
     '/bottom_camera/image_raw', '/bottom_camera/camera_info',
     'bottom_camera_optical_frame', float(os.environ.get('BOTTOM_CAMERA_HFOV', '1.2217'))),
]


class CameraBridge(Node):
    def __init__(self):
        super().__init__('sim_cameras')
        self.lock = threading.Lock()
        self.gz = GzNode()
        self.cams = []
        for name, gz_topic, img_topic, info_topic, frame_id, hfov in CAMERAS:
            cam = {'name': name, 'gz': gz_topic, 'frame_id': frame_id, 'hfov': hfov, 'n': 0, 'warned': False,
                   'img': self.create_publisher(Image, img_topic, 1),
                   'info': self.create_publisher(CameraInfo, info_topic, 1)}
            if not self.gz.subscribe(GzImage, gz_topic, lambda msg, c=cam: self.on_gz_image(c, msg)):
                raise RuntimeError(f'訂閱 gz topic {gz_topic} 失敗')
            self.cams.append(cam)
            self.get_logger().info(f'{name}: {gz_topic} → {img_topic}、{info_topic}')
        self.create_timer(10.0, self.report)

    def on_gz_image(self, cam, msg):
        # gz-transport 的執行緒呼叫進來；rclpy 的 publish 可以跨執行緒用
        if msg.pixel_format_type != RGB_INT8:
            if not cam['warned']:
                cam['warned'] = True
                self.get_logger().error(f'{cam["name"]}：只支援 RGB_INT8，收到 pixel_format_type {msg.pixel_format_type}')
            return
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = cam['frame_id']
        img.height, img.width = msg.height, msg.width
        img.encoding = 'rgb8'
        img.step = msg.step or msg.width * 3
        img.data.frombytes(msg.data)
        cam['img'].publish(img)
        cam['info'].publish(self.info(msg.width, msg.height, stamp, cam['frame_id'], cam['hfov']))
        with self.lock:
            cam['n'] += 1

    @staticmethod
    def info(width, height, stamp, frame_id, hfov):
        f = (width / 2) / math.tan(hfov / 2)
        cx, cy = width / 2, height / 2
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.width, m.height = width, height
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def report(self):
        parts = []
        with self.lock:
            for cam in self.cams:
                parts.append(f'{cam["name"]} {cam["n"] / 10:.1f} fps')
                cam['n'] = 0
        self.get_logger().info('、'.join(parts) + '（0 fps：世界檔有用 model://orca_cam 嗎？）'
                               if any(' 0.0 fps' in p for p in parts) else '、'.join(parts))


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
