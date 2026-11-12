#!/usr/bin/env python3
"""RealSense 彩色影像轉接：RPi 讀 D435i，壓成 JPEG，用 TCP 送給筆電的 cam_bridge。

RPi 上不跑 ROS，只負責把相機「接出來」。筆電主動連進來（RPi:5600），
同時只服務一個 client；新的 client 連進來就取代舊的（筆電重開 cam_bridge 時，
舊連線可能還沒被偵測到斷線）。

通訊協定（每則訊息）：
    4 bytes big-endian 長度 N | N bytes JSON header | header["size"] bytes payload

    {"type": "info", ...}   連上時、相機重新開啟時送：解析度、內參、裝置資訊，size = 0
    {"type": "color", "seq", "stamp", "sent", "size"}   一幀 JPEG
        stamp：拍到這幀的時間（RPi 時鐘，time.time()）
        sent ：開始送出這則訊息的時間（RPi 時鐘），筆電用來估兩台的時鐘差

網路比相機慢時直接丟幀：送出端永遠只送最新的一幀，延遲不會累積。

用法（在 RPi 上）：
    .venv/bin/python cam_relay.py                     # 640x480@30, JPEG 品質 80
    .venv/bin/python cam_relay.py --size 1280x720 --fps 15
"""

import argparse
import json
import socket
import struct
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def log(*args):
    print(*args, flush=True)


class Camera:
    """背景讀相機、壓 JPEG，只保留最新的一幀。相機斷線就每 2 秒重試。"""

    def __init__(self, width, height, fps, quality):
        self.width, self.height, self.fps, self.quality = width, height, fps, quality
        self.cond = threading.Condition()
        self.frame = None          # (seq, stamp, jpeg bytes)
        self.seq = 0
        self.info = None           # 目前相機的 info header（dict）
        self.info_version = 0      # 相機每重新開一次就 +1，sender 看到變了就重送 info
        self.captured = 0          # 統計用

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            pipeline = rs.pipeline()
            try:
                self._stream(pipeline)
            except Exception as e:  # 相機拔掉、被占用、逾時都會到這裡
                log(f'[cam] error: {e}; retry in 2 s')
            try:
                pipeline.stop()
            except Exception:
                pass
            time.sleep(2)

    def _stream(self, pipeline):
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = pipeline.start(config)

        dev = profile.get_device()
        usb = dev.get_info(rs.camera_info.usb_type_descriptor) \
            if dev.supports(rs.camera_info.usb_type_descriptor) else '?'
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        log(f'[cam] {name} serial {serial} on USB {usb}, color {self.width}x{self.height}@{self.fps}')
        if usb.startswith('2'):
            log('[cam] WARNING: camera is on USB 2 — plug it into a blue USB 3 port '
                '(or re-plug the Type-C end firmly) for full bandwidth')

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        info = {
            'type': 'info', 'size': 0,
            'width': intr.width, 'height': intr.height, 'fps': self.fps,
            'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy,
            # realsense2_camera 對 Brown-Conrady 系列一律發 plumb_bob，係數順序相同
            'distortion_model': 'plumb_bob', 'coeffs': list(intr.coeffs),
            'device': name, 'serial': serial, 'usb': usb,
        }
        with self.cond:
            self.info = info
            self.info_version += 1
            self.cond.notify_all()

        params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        while True:
            frames = pipeline.wait_for_frames(5000)
            stamp = time.time()
            color = frames.get_color_frame()
            if not color:
                continue
            ok, jpg = cv2.imencode('.jpg', np.asanyarray(color.get_data()), params)
            if not ok:
                continue
            with self.cond:
                self.seq += 1
                self.frame = (self.seq, stamp, jpg.tobytes())
                self.captured += 1
                self.cond.notify_all()


def send_msg(sock, header, payload=b''):
    h = json.dumps(header).encode()
    sock.sendall(struct.pack('>I', len(h)) + h + payload)


class Server:
    def __init__(self, cam, port):
        self.cam, self.port = cam, port
        self.client_id = 0         # 每來一個新 client 就 +1，舊的 sender 看到就自己結束
        self.sent_frames = 0
        self.sent_bytes = 0

    def serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(2)
        log(f'[net] listening on 0.0.0.0:{self.port}')
        threading.Thread(target=self._report, daemon=True).start()
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 對方斷電、拔線時，sendall 不會馬上出錯；keepalive 讓連線在約 10 秒內被判定死亡
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_USER_TIMEOUT, 10000)
            with self.cam.cond:
                self.client_id += 1
                my_id = self.client_id
                self.cam.cond.notify_all()   # 叫醒舊的 sender，讓它發現自己被取代
            log(f'[net] client {addr[0]}:{addr[1]} connected (#{my_id})')
            threading.Thread(target=self._send_loop, args=(conn, addr, my_id), daemon=True).start()

    def _send_loop(self, conn, addr, my_id):
        cam = self.cam
        last_seq, last_info = -1, -1
        try:
            while True:
                with cam.cond:
                    cam.cond.wait_for(lambda: self.client_id != my_id
                                      or cam.info_version != last_info
                                      or (cam.frame is not None and cam.frame[0] != last_seq),
                                      timeout=1.0)
                    if self.client_id != my_id:
                        log(f'[net] client #{my_id} replaced by a newer one')
                        return
                    info, info_ver, frame = cam.info, cam.info_version, cam.frame
                if info is not None and info_ver != last_info:
                    send_msg(conn, dict(info, server_time=time.time()))
                    last_info = info_ver
                if frame is not None and frame[0] != last_seq:
                    seq, stamp, jpg = frame
                    send_msg(conn, {'type': 'color', 'seq': seq, 'stamp': stamp,
                                    'sent': time.time(), 'size': len(jpg)}, jpg)
                    last_seq = seq
                    self.sent_frames += 1
                    self.sent_bytes += len(jpg)
        except OSError as e:
            log(f'[net] client #{my_id} {addr[0]} disconnected: {e}')
        finally:
            conn.close()

    def _report(self):
        """每 10 秒印一次：相機 fps、送出 fps、平均每幀大小、頻寬。"""
        period = 10.0
        while True:
            time.sleep(period)
            cap, self.cam.captured = self.cam.captured, 0
            n, b = self.sent_frames, self.sent_bytes
            self.sent_frames = self.sent_bytes = 0
            avg = b / n / 1024 if n else 0
            log(f'[stat] camera {cap / period:.1f} fps, sent {n / period:.1f} fps, '
                f'{avg:.0f} KiB/frame, {b * 8 / period / 1e6:.1f} Mbit/s')


def main():
    p = argparse.ArgumentParser(description='RealSense color -> JPEG over TCP (for cam_bridge)')
    p.add_argument('--size', default='640x480', help='彩色解析度，例如 640x480、848x480、1280x720')
    p.add_argument('--fps', type=int, default=30, help='相機幀率（6/15/30/60，依解析度而定）')
    p.add_argument('--quality', type=int, default=80, help='JPEG 品質 1-100')
    p.add_argument('--port', type=int, default=5600)
    a = p.parse_args()

    w, h = (int(v) for v in a.size.lower().split('x'))
    cam = Camera(w, h, a.fps, a.quality)
    cam.start()
    Server(cam, a.port).serve()


if __name__ == '__main__':
    main()
