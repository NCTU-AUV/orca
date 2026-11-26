"""RF-DETR 的 TensorRT 推論（Jetson 用）：只需要 numpy、OpenCV、TensorRT、cuda-python，不需要 torch、rfdetr。

模型的檔案（都在 ml/rfdetr/weights/，容器裡掛在 /model）：
    rfdetr-large.onnx         筆電上用 ml/rfdetr/export_onnx.sh 從 .pth 匯出
    rfdetr-large.json         匯出時一起寫的：類別名稱、輸入解析度
    rfdetr-large.<GPU>.trt<版本>.fp16.engine
                              在 Jetson 上用 build_engine（本檔的 main）從 ONNX 轉的。引擎只能在同一種 GPU、
                              同一版 TensorRT 上用，所以檔名帶這兩個；換了就會重轉

前後處理照 rfdetr 1.12 的 predict()（rfdetr/export/_runtime/preprocess.py、decode.py），
結果跟 PyTorch 版一樣（差異只在 FP16 的數值誤差），驗證用 ml/rfdetr/check_export.py：
- 前處理：RGB → [0,1] → 雙線性縮放到模型解析度（像素中心對齊、不抗鋸齒，同 cv2.INTER_LINEAR）→ ImageNet 正規化
- 輸出：dets（Q×4，正規化的 cx cy w h）、labels（Q×C，各類別的 logit）
- 後處理：每個類別各自 sigmoid（不是 softmax），Q×C 個（框, 類別）攤平取前 300 名，再留分數 > threshold 的，
  框換算回原圖像素。類別索引超過類別名稱數量的（訓練時沒用到的空類別）丟掉
"""
import argparse
import json
import os
import re
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
NUM_SELECT = 300


@dataclass
class Detections:
    """跟 supervision.Detections 一樣的三個欄位，detector.py 兩種後端共用同一段程式。"""
    xyxy: np.ndarray        # N×4，原圖像素
    class_id: np.ndarray    # N，class_names 的索引
    confidence: np.ndarray  # N


def preprocess(rgb, height, width):
    """uint8 RGB（H×W×3）→ 1×3×height×width 的 float32。"""
    x = cv2.resize(rgb.astype(np.float32) * (1.0 / 255.0), (width, height), interpolation=cv2.INTER_LINEAR)
    x = (x - MEAN) / STD
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None])


def decode(boxes, logits, width, height, threshold, num_classes):
    """一張圖的模型輸出（boxes Q×4、logits Q×C）→ Detections。width、height 是原圖大小。"""
    logits = logits.astype(np.float32)
    scores = 1.0 / (1.0 + np.exp(-np.clip(logits, -88, 88)))
    flat = scores.reshape(-1)
    k = min(NUM_SELECT, flat.size)
    top = np.argpartition(-flat, k - 1)[:k]
    top = top[np.lexsort((top, -flat[top]))]          # 分數由高到低，同分時索引小的在前（同 rfdetr）
    top = top[flat[top] > threshold]
    query, cls = np.divmod(top, scores.shape[1])
    keep = cls < num_classes
    query, cls, conf = query[keep], cls[keep], flat[top][keep]
    cx, cy, w, h = boxes[query].astype(np.float32).T
    scale = np.array([width, height, width, height], np.float32)
    xyxy = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1) * scale
    return Detections(np.clip(xyxy, 0, scale), cls.astype(np.int64), conf)


def load_meta(onnx_or_engine):
    """模型旁邊的 .json（類別名稱、解析度）。引擎的檔名是 <模型>.<GPU>.trt<版本>.fp16.engine，往回找 <模型>.json。"""
    base = re.sub(r'(\.[^./]+)*\.(onnx|engine)$', '', onnx_or_engine)
    with open(base + '.json') as f:
        return json.load(f)


# ---------- TensorRT ----------

def _cudart():
    try:
        from cuda.bindings import runtime as cudart     # cuda-python 12.8 起
    except ImportError:
        from cuda import cudart
    return cudart


def _check(result):
    err, *rest = result
    if int(err) != 0:
        raise RuntimeError(f'CUDA error {err}')
    return rest[0] if len(rest) == 1 else rest


class TrtModel:
    """載入 TensorRT 引擎，predict(rgb, threshold) → Detections。輸入、輸出的位置在建立時配置一次，之後重複用。"""

    def __init__(self, engine_path):
        import tensorrt as trt
        self.cudart = cudart = _cudart()
        meta = load_meta(engine_path)
        self.class_names = list(meta['class_names'])
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'{engine_path} 載入失敗（TensorRT 版本或 GPU 跟轉檔時不同？刪掉重轉）')
        self.context = self.engine.create_execution_context()
        self.stream = _check(cudart.cudaStreamCreate())
        self.host, self.device = {}, {}
        self.inputs, self.outputs = [], []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            self.host[name] = np.empty(shape, dtype)
            self.device[name] = _check(cudart.cudaMalloc(self.host[name].nbytes))
            self.context.set_tensor_address(name, self.device[name])
            (self.inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else self.outputs).append(name)
        if len(self.inputs) != 1:
            raise RuntimeError(f'預期一個輸入，引擎有 {self.inputs}')
        self.input = self.inputs[0]
        _, _, self.height, self.width = self.host[self.input].shape
        self.out_boxes = next(n for n in self.outputs if 'dets' in n)
        self.out_logits = next(n for n in self.outputs if 'labels' in n)

    def infer(self, x):
        """前處理好的輸入 → (boxes Q×4, logits Q×C)。"""
        cudart = self.cudart
        h2d, d2h = cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost
        inp = self.host[self.input]
        np.copyto(inp, x, casting='same_kind')
        _check(cudart.cudaMemcpyAsync(self.device[self.input], inp.ctypes.data, inp.nbytes, h2d, self.stream))
        if not self.context.execute_async_v3(self.stream):
            raise RuntimeError('TensorRT 推論失敗')
        for name in self.outputs:
            out = self.host[name]
            _check(cudart.cudaMemcpyAsync(out.ctypes.data, self.device[name], out.nbytes, d2h, self.stream))
        _check(cudart.cudaStreamSynchronize(self.stream))
        return self.host[self.out_boxes][0], self.host[self.out_logits][0]

    def predict(self, rgb, threshold):
        boxes, logits = self.infer(preprocess(rgb, self.height, self.width))
        return decode(boxes, logits, rgb.shape[1], rgb.shape[0], threshold, len(self.class_names))


def engine_path(onnx_path):
    """這台機器、這版 TensorRT 對應的引擎檔名。"""
    import tensorrt as trt
    cudart = _cudart()
    prop = _check(cudart.cudaGetDeviceProperties(0))
    gpu = f'sm{prop.major}{prop.minor}'
    return re.sub(r'\.onnx$', '', onnx_path) + f'.{gpu}.trt{trt.__version__}.fp16.engine'


def build_engine(onnx_path, out_path):
    """ONNX → FP16 TensorRT 引擎。Orin NX 上要幾分鐘。先寫到暫存檔，成功才改名，中斷不會留下壞檔。"""
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(0)
    parser = trt.OnnxParser(network, logger)
    if not parser.parse_from_file(onnx_path):
        errors = '\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors))
        raise RuntimeError(f'ONNX 解析失敗：\n{errors}')
    config = builder.create_builder_config()
    config.set_flag(trt.BuilderFlag.FP16)
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 4 << 30)
    t0 = time.time()
    plan = builder.build_serialized_network(network, config)
    if plan is None:
        raise RuntimeError('引擎建立失敗，看上面 TensorRT 的訊息')
    tmp = out_path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(plan)
    os.replace(tmp, out_path)
    print(f'{out_path}（{os.path.getsize(out_path) >> 20} MiB，{time.time() - t0:.0f} 秒）')


def main():
    """build_engine：ONNX 轉成這台的 TensorRT 引擎，已經有就略過。印出引擎路徑（最後一行）。"""
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('onnx', nargs='?', default='/model/rfdetr-large.onnx')
    ap.add_argument('--force', action='store_true', help='已經有引擎也重轉')
    args = ap.parse_args()
    if not os.path.exists(args.onnx):
        sys.exit(f'找不到 {args.onnx}：先在筆電上跑 ml/rfdetr/export_onnx.sh，再 deploy/jetson/sync.sh')
    out = engine_path(args.onnx)
    if args.force or not os.path.exists(out):
        print(f'轉 TensorRT 引擎：{args.onnx} → {out}（要幾分鐘）', file=sys.stderr)
        build_engine(args.onnx, out)
    print(out)


if __name__ == '__main__':
    main()
