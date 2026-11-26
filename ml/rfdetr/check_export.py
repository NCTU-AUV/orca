#!/usr/bin/env python3
"""驗證匯出的 RF-DETR（ONNX、TensorRT）跟 PyTorch 版的結果一致。

    check_export.py reference <影像資料夾> --out check_reference.json   # PyTorch predict()，存成參考答案（要 rfdetr）
    check_export.py onnx rfdetr-large.onnx --ref check_reference.json   # ONNX Runtime（FP32），筆電上
    check_export.py trt <引擎> --ref check_reference.json               # TensorRT（FP16），Jetson 上

onnx、trt 用的前後處理是 orca_perception/rfdetr_trt.py（Jetson 的偵測節點用的同一份），所以這個比對同時驗證了
匯出的模型和我們自己寫的前後處理。一般由 ml/rfdetr/export_onnx.sh（reference、onnx）和
deploy/jetson/up.sh（第一次轉引擎後跑 trt）呼叫。

比對：參考答案裡分數 ≥ 0.4（偵測節點的門檻）的每個框，在受測結果裡找同類別、IoU 最大的框。
- ONNX（FP32）：每個框都要找到，IoU ≥ 0.98、分數差 ≤ 0.02
- TensorRT（FP16）：每個框都要找到，IoU ≥ 0.9、分數差 ≤ 0.05
  例外：參考分數在門檻附近（0.4～0.45）的框，FP16 的誤差可能讓它掉到門檻下，找不到不算失敗
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

try:
    from orca_perception import rfdetr_trt
except ImportError:      # 筆電上的匯出容器：src/orca_perception 掛在 /orca_perception，沒有 colcon 安裝
    sys.path.insert(0, os.environ.get('ORCA_PERCEPTION', '/orca_perception'))
    from orca_perception import rfdetr_trt

LIST_THRESHOLD = 0.3     # 參考答案多存一些低分的框，受測端分數稍高時才有東西對
CHECK_THRESHOLD = 0.4
BORDER = 0.45
TOL = {'onnx': (0.98, 0.02), 'trt': (0.9, 0.05)}


def load_rgb(path):
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f'讀不到 {path}')
    return np.ascontiguousarray(bgr[:, :, ::-1])


def to_list(det, names):
    return [{'class': names[int(c)], 'conf': float(s), 'xyxy': [float(v) for v in b]}
            for b, c, s in zip(det.xyxy, det.class_id, det.confidence)]


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def reference(args):
    from PIL import Image
    from rfdetr import RFDETRLarge
    model = RFDETRLarge(pretrain_weights=args.weights, num_classes=8)
    names = list(model.class_names)
    images = sorted(glob.glob(os.path.join(args.images, '*.jpg')) + glob.glob(os.path.join(args.images, '*.png')))
    out = {'class_names': names, 'images': {}}
    for path in images:
        rgb = load_rgb(path)
        det = model.predict(Image.fromarray(rgb), threshold=LIST_THRESHOLD)
        out['images'][os.path.basename(path)] = {'path': path, 'detections': to_list(det, names)}
        n = sum(d['conf'] >= CHECK_THRESHOLD for d in out['images'][os.path.basename(path)]['detections'])
        print(f'  {os.path.basename(path)}: {n} 個框 ≥ {CHECK_THRESHOLD}')
    with open(args.out, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(f'參考答案：{len(images)} 張影像 → {args.out}')


class OnnxModel:
    def __init__(self, path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        self.input = inp.name
        _, _, self.height, self.width = inp.shape
        outs = [o.name for o in self.session.get_outputs()]
        self.order = [next(n for n in outs if 'dets' in n), next(n for n in outs if 'labels' in n)]
        self.class_names = rfdetr_trt.load_meta(path)['class_names']

    def predict(self, rgb, threshold):
        boxes, logits = self.session.run(self.order, {self.input: rfdetr_trt.preprocess(rgb, self.height, self.width)})
        return rfdetr_trt.decode(boxes[0], logits[0], rgb.shape[1], rgb.shape[0], threshold, len(self.class_names))


def compare(args):
    with open(args.ref) as f:
        ref = json.load(f)
    model = OnnxModel(args.model) if args.mode == 'onnx' else rfdetr_trt.TrtModel(args.model)
    if list(model.class_names) != ref['class_names']:
        sys.exit(f'類別名稱不同：{model.class_names} vs {ref["class_names"]}')
    min_iou, max_dconf = TOL[args.mode]
    image_dir = args.images
    failures, checked, times = 0, 0, []
    for name, item in ref['images'].items():
        path = os.path.join(image_dir, name) if image_dir else item['path']
        rgb = load_rgb(path)
        model.predict(rgb, LIST_THRESHOLD)          # 第一次（暖機）不計時
        t0 = time.time()
        got = to_list(model.predict(rgb, LIST_THRESHOLD), model.class_names)
        times.append(time.time() - t0)
        for r in item['detections']:
            if r['conf'] < CHECK_THRESHOLD:
                continue
            checked += 1
            cands = [g for g in got if g['class'] == r['class']]
            best = max(cands, key=lambda g: iou(g['xyxy'], r['xyxy']), default=None)
            i = iou(best['xyxy'], r['xyxy']) if best else 0.0
            d = abs(best['conf'] - r['conf']) if best else 1.0
            ok = i >= min_iou and d <= max_dconf
            if not ok and r['conf'] < BORDER and (best is None or best['conf'] < CHECK_THRESHOLD):
                print(f'  {name}: {r["class"]} {r["conf"]:.3f} 在門檻附近，找不到不算錯')
                continue
            if not ok:
                failures += 1
                print(f'  不一致 {name}: {r["class"]} 參考 {r["conf"]:.3f}，'
                      f'受測 {best["conf"] if best else 0:.3f}，IoU {i:.3f}')
    print(f'{args.mode}：比對 {checked} 個框，不一致 {failures} 個；'
          f'每張推論＋前後處理 {np.mean(times) * 1000:.0f} ms（{len(times)} 張平均）')
    if checked == 0:
        sys.exit('參考答案裡沒有 ≥ 門檻的框，沒比到東西')
    sys.exit(1 if failures else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    r = sub.add_parser('reference')
    r.add_argument('images')
    r.add_argument('--out', required=True)
    r.add_argument('--weights', default='/model/checkpoint_best_total.pth')
    for mode in ('onnx', 'trt'):
        c = sub.add_parser(mode)
        c.add_argument('model')
        c.add_argument('--ref', required=True)
        c.add_argument('--images', help='影像資料夾（預設用參考答案裡記的路徑）')
    args = ap.parse_args()
    reference(args) if args.mode == 'reference' else compare(args)


if __name__ == '__main__':
    main()
