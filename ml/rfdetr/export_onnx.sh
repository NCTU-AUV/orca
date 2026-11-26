#!/usr/bin/env bash
# 在筆電上跑：把 weights/checkpoint_best_total.pth 匯出成 ONNX（給 Jetson 轉 TensorRT），並驗證匯出的結果。
#
#   ml/rfdetr/export_onnx.sh
#
# 產生（都在 weights/，不進 git；Jetson 由 deploy/jetson/sync.sh 帶過去）：
#   rfdetr-large.onnx        輸入 1×3×704×704，輸出 dets（300×4）、labels（300×9）
#   rfdetr-large.json        類別名稱、輸入解析度（Jetson 的偵測節點讀這個）
#   check_reference.json     PyTorch 版 predict() 在 docs/sim/reference/*.jpg 上的結果，Jetson 上用來比對 TensorRT
#
# 驗證：同樣的影像用 ONNX Runtime ＋ src/orca_perception/orca_perception/rfdetr_trt.py 的前後處理跑一次，
# 跟 PyTorch 版比對（check_export.py）。對不上就不算成功。
#
# 用 orca-perception 映像（deploy/laptop 的 object_detector）加裝 rfdetr[onnx]、onnxruntime，
# 存成 orca-rfdetr-export 映像，第二次起不用重裝。
set -euo pipefail
cd "$(dirname "$0")/../.."
ROOT=$PWD

if [[ ! -f ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ml/rfdetr/fetch_weights.sh
fi
if ! docker image inspect orca-perception > /dev/null 2>&1; then
  (cd deploy/laptop && docker compose build object_detector)
fi
if ! docker image inspect orca-rfdetr-export > /dev/null 2>&1; then
  docker build -t orca-rfdetr-export - << 'EOF'
FROM orca-perception
RUN pip3 install --no-cache-dir "rfdetr[onnx]==1.12.0" onnxruntime "numpy<2"
EOF
fi

docker run --rm --gpus all --entrypoint bash \
  -v "$ROOT/ml/rfdetr/weights:/model" \
  -v "$ROOT/ml/rfdetr:/rfdetr:ro" \
  -v "$ROOT/src/orca_perception:/orca_perception:ro" \
  -v "$ROOT/docs/sim/reference:/images:ro" \
  -v orca_torch-cache:/root/.cache \
  orca-rfdetr-export -c '
set -e
python3 - << EOF 2>&1 | grep -vE "Warning|warn\(|^  "
import json
from rfdetr import RFDETRLarge
m = RFDETRLarge(pretrain_weights="/model/checkpoint_best_total.pth", num_classes=8)
p = m.export(output_dir="/model", verbose=False)
res = m.model_config.resolution if hasattr(m, "model_config") else 704
json.dump({"class_names": list(m.class_names), "resolution": int(res),
           "source": "checkpoint_best_total.pth", "rfdetr": "1.12.0"},
          open("/model/rfdetr-large.json", "w"), ensure_ascii=False, indent=1)
print("匯出", p)
EOF
python3 /rfdetr/check_export.py reference /images --out /model/check_reference.json
python3 /rfdetr/check_export.py onnx /model/rfdetr-large.onnx --ref /model/check_reference.json
'
ls -la ml/rfdetr/weights/
