#!/usr/bin/env python3
"""Gazebo 相機 → ROS，發布成跟實機一樣的 topic，下游不用改。

    gz topic（sim/models/orca_cam）        ROS topic                                  frame_id
    /orca/camera/image          前視  →  /camera/camera/color/image_raw、camera_info   camera_color_optical_frame
    /orca/bottom_camera/image   底部  →  /bottom_camera/image_raw、camera_info          bottom_camera_optical_frame

影像 sensor_msgs/Image（rgb8）；camera_info 是針孔、無畸變，內參由視角算。
frame_id 跟 src/orca_mission/config/orca.yaml 一致：偵測靠它分辨是哪一台相機。

水下效果：Gazebo 算出來的畫面乾淨明亮，轉成 ROS 之前先用同一顆相機的深度圖
（gz topic /orca/camera/depth、/orca/bottom_camera/depth，不會變成 ROS topic）做水下成像的後處理：
越遠越暗、偏藍、對比下降、模糊、噪點。做法與參數見 sim/underwater.py。
只改像素；topic 名稱、frame_id、影像大小與編碼、camera_info 都跟沒開效果時完全一樣。
    環境變數   UNDERWATER=1|0（0 = 完全關掉，影像原封不動）、UNDERWATER_STRENGTH=1.0（整體強度）
    ROS 參數   underwater（bool）、underwater_strength（double），執行中可以改：
               ros2 param set /sim_cameras underwater false
世界檔沒有 depth camera（或深度一直沒來）時，等 2 秒後改成原封不動轉送，並印一次警告。

在 orca-sim image 裡跑（有 gz-transport 的 Python 綁定和 ROS 2 Humble），見 deploy/sim/compose.yaml
的 sim_cameras 服務。不需要 ros_gz。世界檔沒有某台相機時，那台就不會有影像，其他照常。

時間戳：收到影像時的 ROS 時鐘（主機時間）。模擬以約 1 倍速跑，跟實機 cam_bridge 一樣用主機時間。
視角要跟模型一致：環境變數 CAMERA_HFOV（前視，0.9725 rad）、BOTTOM_CAMERA_HFOV（底部，1.2217 rad）。
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from underwater import Underwater  # noqa: E402

RGB_INT8 = 3        # gz.msgs.PixelFormatType
R_FLOAT32 = 13
DEPTH_WAIT_S = 2.0  # 開了水下效果但深度還沒來時，最多扣住影像這麼久

# (名稱, gz 影像 topic, gz 深度 topic, ROS 影像 topic, ROS camera_info topic, frame_id, 水平視角 rad)
CAMERAS = [
    ('front', os.environ.get('GZ_CAMERA_TOPIC', '/orca/camera/image'),
     os.environ.get('GZ_CAMERA_DEPTH_TOPIC', '/orca/camera/depth'),
     '/camera/camera/color/image_raw', '/camera/camera/color/camera_info',
     'camera_color_optical_frame', float(os.environ.get('CAMERA_HFOV', '0.9725'))),
    ('bottom', os.environ.get('GZ_BOTTOM_CAMERA_TOPIC', '/orca/bottom_camera/image'),
     os.environ.get('GZ_BOTTOM_CAMERA_DEPTH_TOPIC', '/orca/bottom_camera/depth'),
     '/bottom_camera/image_raw', '/bottom_camera/camera_info',
     'bottom_camera_optical_frame', float(os.environ.get('BOTTOM_CAMERA_HFOV', '1.2217'))),
]


class CameraBridge(Node):
    def __init__(self):
        super().__init__('sim_cameras')
        self.lock = threading.Lock()
        self.gz = GzNode()
        self.cams = []
        for name, gz_topic, gz_depth, img_topic, info_topic, frame_id, hfov in CAMERAS:
            cam = {'name': name, 'gz': gz_topic, 'gz_depth': gz_depth, 'frame_id': frame_id, 'hfov': hfov,
                   'n': 0, 'warned': False, 'depth': None, 'depth_sub': False, 'first_rgb': None,
                   'no_depth_warned': False, 'uw': Underwater(hfov),
                   'img': self.create_publisher(Image, img_topic, 1),
                   'info': self.create_publisher(CameraInfo, info_topic, 1)}
            if not self.gz.subscribe(GzImage, gz_topic, lambda msg, c=cam: self.on_gz_image(c, msg)):
                raise RuntimeError(f'訂閱 gz topic {gz_topic} 失敗')
            self.cams.append(cam)
            self.get_logger().info(f'{name}: {gz_topic} → {img_topic}、{info_topic}')

        uw = self.cams[0]['uw']
        self.declare_parameter('underwater', uw.enabled)
        self.declare_parameter('underwater_strength', uw.strength)
        self.add_on_set_parameters_callback(self.on_params)
        self.set_underwater(uw.enabled, uw.strength)
        self.create_timer(10.0, self.report)

    # ---- 水下效果的開關
    def set_underwater(self, enabled, strength):
        for cam in self.cams:
            cam['uw'].enabled, cam['uw'].strength = enabled, strength
            if enabled and not cam['depth_sub']:
                # 深度只在需要時才訂閱：depth camera 沒人訂閱就不算圖
                if not self.gz.subscribe(GzImage, cam['gz_depth'], lambda msg, c=cam: self.on_gz_depth(c, msg)):
                    raise RuntimeError(f'訂閱 gz topic {cam["gz_depth"]} 失敗')
                cam['depth_sub'] = True
        self.get_logger().info(f'水下效果：{"開" if enabled else "關"}' + (f'，強度 {strength:g}' if enabled else ''))

    def on_params(self, params):
        uw = self.cams[0]['uw']
        enabled, strength = uw.enabled, uw.strength
        for p in params:
            if p.name == 'underwater':
                enabled = bool(p.value)
            elif p.name == 'underwater_strength':
                if p.value < 0:
                    return SetParametersResult(successful=False, reason='underwater_strength 不能是負的')
                strength = float(p.value)
        self.set_underwater(enabled, strength)
        return SetParametersResult(successful=True)

    # ---- gz-transport 的執行緒呼叫進來；rclpy 的 publish 可以跨執行緒用
    def on_gz_depth(self, cam, msg):
        if msg.pixel_format_type != R_FLOAT32:
            return
        cam['depth'] = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)

    def on_gz_image(self, cam, msg):
        if msg.pixel_format_type != RGB_INT8:
            if not cam['warned']:
                cam['warned'] = True
                self.get_logger().error(f'{cam["name"]}：只支援 RGB_INT8，收到 pixel_format_type {msg.pixel_format_type}')
            return
        data = msg.data
        step = msg.step or msg.width * 3
        uw = cam['uw']
        if uw.enabled:
            depth = cam['depth']
            if depth is None:
                now = time.monotonic()
                if cam['first_rgb'] is None:
                    cam['first_rgb'] = now
                if now - cam['first_rgb'] < DEPTH_WAIT_S:
                    return          # 深度馬上就來，先不要送出沒處理過的乾淨畫面
                if not cam['no_depth_warned']:
                    cam['no_depth_warned'] = True
                    self.get_logger().warn(
                        f'{cam["name"]}：沒有收到深度（{cam["gz_depth"]}），水下效果沒有作用，影像原封不動轉送。'
                        '世界檔的船是 model://orca_cam 嗎？')
            else:
                rgb = np.frombuffer(data, np.uint8).reshape(msg.height, step)[:, :msg.width * 3]
                data = uw.apply(rgb.reshape(msg.height, msg.width, 3), depth).tobytes()
                step = msg.width * 3
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = cam['frame_id']
        img.height, img.width = msg.height, msg.width
        img.encoding = 'rgb8'
        img.step = step
        img.data.frombytes(data)
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
