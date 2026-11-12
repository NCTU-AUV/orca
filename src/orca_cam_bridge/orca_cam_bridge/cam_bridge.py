#!/usr/bin/env python3
"""cam_bridge：接 RPi cam-relay 的 JPEG 串流，發布成跟 realsense2_camera 一樣的 topic。

    /camera/camera/color/image_raw             sensor_msgs/Image（rgb8）
    /camera/camera/color/image_raw/compressed  sensor_msgs/CompressedImage（RPi 送來的原始 JPEG，
                                               給 recorder 錄；格式同 image_transport 的 compressed）
    /camera/camera/color/camera_info           sensor_msgs/CameraInfo

節點名稱 camera、namespace camera，跟 realsense2_camera 預設一樣，下游節點搬到 Jetson
換成 realsense2_camera 時不用改。通訊協定見 deploy/rpi-relay/cam_relay.py。

時間戳：header.stamp = RPi 拍攝時間換算成筆電時鐘。兩台時鐘的差用每幀的 "sent"
估：sent(RPi) - 收到(筆電) = 時鐘差 - 傳輸時間，取最近一段的最大值當時鐘差。

參數：host（預設 10.42.0.1）、port（5600）、frame_id（camera_color_optical_frame）
"""

import json
import socket
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError('connection closed by RPi')
        got += k
    return bytes(buf)


class CamBridge(Node):
    def __init__(self):
        super().__init__('camera', namespace='camera')
        self.host = self.declare_parameter('host', '10.42.0.1').value
        self.port = self.declare_parameter('port', 5600).value
        self.frame_id = self.declare_parameter('frame_id', 'camera_color_optical_frame').value

        self.pub_img = self.create_publisher(Image, '~/color/image_raw', 1)
        self.pub_jpeg = self.create_publisher(CompressedImage, '~/color/image_raw/compressed', 1)
        self.pub_info = self.create_publisher(CameraInfo, '~/color/camera_info', 1)
        self.info_msg = None
        self.offsets = deque(maxlen=300)   # sent - recv 的樣本（秒）

        self.n_frames = 0
        self.n_bytes = 0
        self.lat_sum = 0.0
        self.create_timer(10.0, self.report)
        threading.Thread(target=self.net_loop, daemon=True).start()

    # ---- 網路 ----
    def net_loop(self):
        backoff = 1.0
        while rclpy.ok():
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as sock:
                    sock.settimeout(5)   # 5 秒收不到東西就當斷線重連
                    self.get_logger().info(f'connected to {self.host}:{self.port}')
                    backoff = 1.0
                    self.offsets.clear()
                    while rclpy.ok():
                        (hlen,) = struct.unpack('>I', recv_exact(sock, 4))
                        header = json.loads(recv_exact(sock, hlen))
                        payload = recv_exact(sock, header['size']) if header['size'] else b''
                        self.on_message(header, payload, time.time())
            except (OSError, ConnectionError, ValueError) as e:
                self.get_logger().warn(f'{self.host}:{self.port}: {e}; retry in {backoff:.0f} s')
                time.sleep(backoff)
                backoff = min(backoff * 2, 8.0)

    def on_message(self, header, payload, recv_time):
        kind = header['type']
        if kind == 'info':
            self.offsets.append(header['server_time'] - recv_time)
            self.info_msg = self.make_info(header)
            self.get_logger().info(
                f"camera {header['device']} ({header['serial']}) USB {header['usb']}, "
                f"{header['width']}x{header['height']}@{header['fps']}")
            if str(header['usb']).startswith('2'):
                self.get_logger().warn('camera is on USB 2 at the RPi side')
        elif kind == 'color':
            self.offsets.append(header['sent'] - recv_time)
            offset = max(self.offsets)
            capture_local = header['stamp'] - offset      # 換成筆電時鐘的拍攝時間
            img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn('bad JPEG frame, skipped')
                return
            self.publish(img, payload, capture_local)
            self.n_frames += 1
            self.n_bytes += len(payload)
            self.lat_sum += recv_time - capture_local

    def make_info(self, h):
        m = CameraInfo()
        m.header.frame_id = self.frame_id
        m.width, m.height = h['width'], h['height']
        m.distortion_model = h['distortion_model']
        m.d = [float(c) for c in h['coeffs']]
        fx, fy, cx, cy = h['fx'], h['fy'], h['ppx'], h['ppy']
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    # ---- 發布 ----
    def publish(self, bgr, jpeg, stamp_sec):
        stamp = Time(seconds=stamp_sec).to_msg()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data.frombytes(rgb.tobytes())   # 直接塞 bytes，避免逐元素檢查
        self.pub_img.publish(msg)
        if self.pub_jpeg.get_subscription_count():
            cmsg = CompressedImage()
            cmsg.header = msg.header
            # image_transport 的寫法：「解碼後要還原成的 encoding; jpeg compressed 壓縮時的 encoding」
            cmsg.format = 'rgb8; jpeg compressed bgr8'
            cmsg.data = jpeg
            self.pub_jpeg.publish(cmsg)
        if self.info_msg is not None:
            self.info_msg.header.stamp = stamp
            self.pub_info.publish(self.info_msg)

    def report(self):
        n, b, lat = self.n_frames, self.n_bytes, self.lat_sum
        self.n_frames = self.n_bytes = 0
        self.lat_sum = 0.0
        if n:
            self.get_logger().info(
                f'{n / 10:.1f} fps, {b * 8 / 10 / 1e6:.1f} Mbit/s, '
                f'capture->received {lat / n * 1000:.0f} ms')
        else:
            self.get_logger().warn('no frames in the last 10 s')


def main():
    rclpy.init()
    node = CamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
