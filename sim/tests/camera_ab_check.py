#!/usr/bin/env python3
"""驗收：水下效果開與關，影像明顯不同，而 camera_info、frame_id、影像格式完全相同。

對著正在跑的模擬（有相機的世界）與 sim_cameras 服務：用 ROS 參數把 /sim_cameras 的 underwater
切成開、關，各收幾張前視與底部的影像和 camera_info 來比，最後把參數恢復原狀。
船靜靜浮在水面（沒解鎖），所以兩組影像拍的是同一個場景。

    cd deploy/sim && WORLD=sauvc_qual.sdf GUI=0 docker compose --profile vision up -d simulator sim_referee sim_cameras
    docker compose exec sim_referee bash -c 'source /opt/ros/humble/setup.bash && python3 /sim/tests/camera_ab_check.py'

結果印成一行 JSON；CHECK_OUT 有設就存檔，並在旁邊存一張開／關的對照圖（同名 .jpg）。
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

CAMS = {
    'front': ('/camera/camera/color/image_raw', '/camera/camera/color/camera_info'),
    'bottom': ('/bottom_camera/image_raw', '/bottom_camera/camera_info'),
}
N = 5


def info_fields(m):
    return dict(frame_id=m.header.frame_id, width=m.width, height=m.height, distortion_model=m.distortion_model,
                d=list(m.d), k=list(m.k), r=list(m.r), p=list(m.p), binning=(m.binning_x, m.binning_y),
                roi=(m.roi.x_offset, m.roi.y_offset, m.roi.height, m.roi.width, m.roi.do_rectify))


def img_fields(m):
    return dict(frame_id=m.header.frame_id, width=m.width, height=m.height, encoding=m.encoding, step=m.step,
                is_bigendian=m.is_bigendian)


class AB(Node):
    def __init__(self):
        super().__init__('camera_ab_check')
        self.buf = {k: {'img': [], 'info': []} for k in CAMS}
        self.collect = False
        for name, (it, ct) in CAMS.items():
            self.create_subscription(Image, it, lambda m, n=name: self.on(n, 'img', m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, ct, lambda m, n=name: self.on(n, 'info', m), qos_profile_sensor_data)
        self.set_cli = self.create_client(SetParameters, '/sim_cameras/set_parameters')
        self.get_cli = self.create_client(GetParameters, '/sim_cameras/get_parameters')

    def on(self, cam, kind, m):
        if self.collect and len(self.buf[cam][kind]) < N:
            self.buf[cam][kind].append(m)

    def call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=10):
            sys.exit(f'{cli.srv_name} 不存在：sim_cameras 有在跑嗎？')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        return fut.result()

    def get_underwater(self):
        r = self.call(self.get_cli, GetParameters.Request(names=['underwater']))
        return r.values[0].bool_value

    def set_underwater(self, on):
        p = Parameter(name='underwater', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on))
        r = self.call(self.set_cli, SetParameters.Request(parameters=[p]))
        if not r.results[0].successful:
            sys.exit(f'設定 underwater={on} 失敗：{r.results[0].reason}')

    def grab(self):
        for v in self.buf.values():
            v['img'].clear()
            v['info'].clear()
        end = time.monotonic() + 1.0             # 切換後先丟掉 1 秒內的影像
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collect = True
        end = time.monotonic() + 15
        while time.monotonic() < end and any(len(v[k]) < N for v in self.buf.values() for k in v):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collect = False
        return {c: {k: list(v[k]) for k in v} for c, v in self.buf.items()}


def to_array(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return a.astype(np.float32)


def main():
    rclpy.init()
    node = AB()
    original = node.get_underwater()
    try:
        node.set_underwater(True)
        on = node.grab()
        node.set_underwater(False)
        off = node.grab()
    finally:
        node.set_underwater(original)
    out = {'restored_underwater': original}
    ok = True
    sheet = []
    for cam in CAMS:
        if not on[cam]['img'] or not off[cam]['img'] or not on[cam]['info'] or not off[cam]['info']:
            out[cam] = 'no images'
            ok = False
            continue
        info_same = all(info_fields(a) == info_fields(on[cam]['info'][0]) for a in on[cam]['info'] + off[cam]['info'])
        img_same = all(img_fields(a) == img_fields(on[cam]['img'][0]) for a in on[cam]['img'] + off[cam]['img'])
        a_on = np.mean([to_array(m) for m in on[cam]['img']], axis=0)
        a_off = np.mean([to_array(m) for m in off[cam]['img']], axis=0)
        # 同一個條件裡前後幾張的差（船在水面微微晃、噪點），拿來跟開／關的差比
        within = float(np.mean(np.abs(to_array(off[cam]['img'][0]) - to_array(off[cam]['img'][-1]))))
        between = float(np.mean(np.abs(a_on - a_off)))
        out[cam] = dict(camera_info_identical=info_same, image_format_identical=img_same,
                        frame_id=on[cam]['info'][0].header.frame_id,
                        mean_rgb_on=[round(float(x), 1) for x in a_on.reshape(-1, 3).mean(0)],
                        mean_rgb_off=[round(float(x), 1) for x in a_off.reshape(-1, 3).mean(0)],
                        mean_abs_diff_on_vs_off=round(between, 1), mean_abs_diff_within_off=round(within, 1),
                        k=list(on[cam]['info'][0].k))
        ok &= info_same and img_same and between > 5 * max(within, 1.0)
        tiles = [cv2.cvtColor(np.clip(x, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR) for x in (a_off, a_on)]
        for t, label in zip(tiles, ('underwater off', 'underwater on')):
            cv2.putText(t, f'{cam}: {label}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        sheet.append(np.hstack(tiles))
    out['pass'] = bool(ok)
    print(json.dumps(out, ensure_ascii=False))
    path = os.environ.get('CHECK_OUT')
    if path:
        with open(path, 'w') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        if sheet:
            cv2.imwrite(os.path.splitext(path)[0] + '.jpg', np.vstack(sheet), [cv2.IMWRITE_JPEG_QUALITY, 85])
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
