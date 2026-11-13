#!/usr/bin/env python3
"""RF-DETR 偵測節點：訂閱一或多台相機的彩色影像，發布偵測框。

    訂閱 image_topics                     sensor_msgs/Image（rgb8 或 bgr8），預設：
           /camera/camera/color/image_raw   前視（D435i）
           /bottom_camera/image_raw         底部
    發布 /detections                      vision_msgs/Detection2DArray（header 沿用影像的）
         annotated_topics                 sensor_msgs/Image（畫好框，給人看），跟 image_topics 一一對應：
           /detections/image、/detections/bottom_image
         <annotated_topics>/compressed     sensor_msgs/CompressedImage（同一張畫好框的影像，JPEG；
                                          給 recorder 錄、Foxglove 看。格式同 image_transport 的 compressed）

所有相機共用同一個 /detections：用 header.frame_id 分辨是哪一台（每台相機的 frame_id 要不一樣）。
模型只載入一次，各相機的影像輪流推論；每台只處理最新的一幀（佇列深度 1），
推論比相機慢時舊幀直接丟掉，延遲不會累積。沒接的相機不會有影像，不影響其他台。

畫框的影像只在有人訂閱時才產生（原始、JPEG 各自判斷），沒人看就不花時間。

參數：weights（/model/checkpoint_best_total.pth）、threshold（0.4）、
      image_topics、annotated_topics、publish_image（true）、jpeg_quality（80）
"""

import time

import cv2
import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rfdetr import RFDETRLarge
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# 每個類別一個框色（BGR）
COLORS = [(0, 200, 0), (0, 255, 255), (0, 128, 255), (0, 0, 255),
          (0, 220, 220), (255, 80, 0), (60, 20, 220), (200, 100, 0)]


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        weights = self.declare_parameter('weights', '/model/checkpoint_best_total.pth').value
        self.threshold = self.declare_parameter('threshold', 0.4).value
        topics = list(self.declare_parameter(
            'image_topics', ['/camera/camera/color/image_raw', '/bottom_camera/image_raw']).value)
        annotated = list(self.declare_parameter(
            'annotated_topics', ['/detections/image', '/detections/bottom_image']).value)
        if len(annotated) != len(topics):
            raise ValueError('annotated_topics 要跟 image_topics 一樣多')
        self.publish_image = self.declare_parameter('publish_image', True).value
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)

        self.get_logger().info(f'loading {weights} ...')
        self.model = RFDETRLarge(pretrain_weights=weights, num_classes=8)
        # predict() 的 class_id 是 class_names 的 0 起算索引
        self.names = list(self.model.class_names)
        self.get_logger().info(f'classes: {self.names}')

        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        # 深度 1、best effort：推論比相機慢時只留最新一幀。
        # （qos_profile_sensor_data 的深度是 5，會排隊排出約 150 ms 的延遲）
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, out in zip(topics, annotated):
            pubs = (self.create_publisher(Image, out, 1),
                    self.create_publisher(CompressedImage, out + '/compressed', 1))
            self.create_subscription(Image, topic, lambda m, p=pubs: self.on_image(m, p), qos)

        self.n = 0
        self.infer_sum = 0.0
        self.lat_sum = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'waiting for images on {", ".join(topics)}')

    def on_image(self, msg, pubs):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'unsupported encoding {msg.encoding}', throttle_duration_sec=5)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        rgb = img if msg.encoding == 'rgb8' else img[:, :, ::-1]

        t0 = time.time()
        det = self.model.predict(PILImage.fromarray(np.ascontiguousarray(rgb)),
                                 threshold=self.threshold)
        t1 = time.time()

        out = Detection2DArray()
        out.header = msg.header
        for (x1, y1, x2, y2), cid, conf in zip(det.xyxy, det.class_id, det.confidence):
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = float(x1 + x2) / 2
            d.bbox.center.position.y = float(y1 + y2) / 2
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = self.names[int(cid)]
            h.hypothesis.score = float(conf)
            d.results.append(h)
            out.detections.append(d)
        self.pub_det.publish(out)

        pub_img, pub_jpeg = pubs
        want_raw = pub_img.get_subscription_count() > 0
        want_jpeg = pub_jpeg.get_subscription_count() > 0
        if self.publish_image and (want_raw or want_jpeg):
            bgr = self.draw(rgb, det)
            if want_raw:
                pub_img.publish(self.to_image(bgr, msg.header))
            jpeg = self.to_jpeg(bgr, msg.header) if want_jpeg else None
            if jpeg is not None:
                pub_jpeg.publish(jpeg)

        self.n += 1
        self.infer_sum += t1 - t0
        self.lat_sum += time.time() - Time.from_msg(msg.header.stamp).nanoseconds / 1e9

    def draw(self, rgb, det):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for (x1, y1, x2, y2), cid, conf in zip(det.xyxy, det.class_id, det.confidence):
            c = COLORS[int(cid) % len(COLORS)]
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(bgr, p1, p2, c, 2)
            label = f'{self.names[int(cid)]} {conf:.2f}'
            cv2.putText(bgr, label, (p1[0], max(p1[1] - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
        return bgr

    @staticmethod
    def to_image(bgr, header):
        m = Image()
        m.header = header
        m.height, m.width = bgr.shape[:2]
        m.encoding = 'bgr8'
        m.step = m.width * 3
        m.data.frombytes(bgr.tobytes())
        return m

    def to_jpeg(self, bgr, header):
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return None
        m = CompressedImage()
        m.header = header
        # image_transport 的寫法：「解碼後要還原成的 encoding; jpeg compressed 壓縮時的 encoding」
        m.format = 'bgr8; jpeg compressed bgr8'
        m.data.frombytes(buf.tobytes())
        return m

    def report(self):
        n = self.n
        if n:
            self.get_logger().info(
                f'{n / 5:.1f} fps, inference {self.infer_sum / n * 1000:.0f} ms, '
                f'capture->detections {self.lat_sum / n * 1000:.0f} ms')
        else:
            self.get_logger().warn('no images in the last 5 s')
        self.n = 0
        self.infer_sum = self.lat_sum = 0.0


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
