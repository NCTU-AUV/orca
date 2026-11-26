# RF-DETR v2（SAUVC 物件偵測）

RF-DETR Large（DINOv2 small windowed backbone，輸入解析度 704），8 類。在 Colab A100 上訓練（2026-11-07）。

## 類別

`gate`、`qualification_gate`、`orange_flare`、`red_flare`、`yellow_flare`、`blue_flare`、`red_drum`、`blue_drum`

## 結果（valid）

最佳 epoch 13：mAP@50 0.867、mAP@50:95 0.631。分數在第 3 個 epoch 左右就到平台。還沒跑 test 集。

## 檔案

| 路徑 | 內容 |
|---|---|
| `fetch_weights.sh` | 從 GitHub Releases（tag `rfdetr-v2`）下載 `checkpoint_best_total.pth` 到 `weights/`，並檢查 sha256 |
| `export_onnx.sh` | 匯出 ONNX 給 Jetson 轉 TensorRT（`weights/rfdetr-large.onnx`、`.json`），並跟 PyTorch 版比對（`check_export.py`）。見 [docs/jetson.md](../../docs/jetson.md#偵測tensorrt) |
| `check_export.py` | 比對 ONNX／TensorRT 跟 PyTorch 版的偵測結果 |
| `config/training_config.json` | 完整的訓練與模型設定，含 `class_names` |
| `config/hparams.yaml`、`config/run_config.json` | 訓練參數 |
| `config/metrics.csv` | 每個 epoch 的指標 |
| `notebook/train_rfdetr_v2.ipynb` | 訓練、評估、視覺化、匯出 ONNX（Colab，資料從 Google Drive 讀取；輸出已清掉） |

## 資料集

約 1.8 GB，不放進 repo，在隊上 Google Drive 的 `SAUVC_RFDETR/`。Roboflow COCO 格式，`train/`、`valid/`、`test/` 各自有影像和 `_annotations.coco.json`。train 25,220 筆（真實影像約 84%、合成約 11%），valid、test 只有真實影像，按來源影片切分。

## Jetson Orin NX 實測（2026-11-26）

ONNX（`model.export()`，輸入 `[1,3,704,704]`，輸出 `dets [1,300,4]`、`labels [1,300,9]`）→ `trtexec --fp16` 建 engine，接 D435i 640×480 量 300 幀：

| 功耗模式 | GPU 時脈 | GPU 推論（trtexec） | 每幀總時間（前處理＋推論＋後處理） |
|---|---|---|---|
| 25W | 408 MHz | 48.6 ms | 60 ms（約 16 fps，相機 30 fps 時會跳幀） |
| MAXN + `jetson_clocks` | 918 MHz | 21.8 ms | 28 ms |

- 跟 `predict()` 比對：`labels` 前 8 個通道就是 `class_names` 的順序，第 9 個是「沒有物體」，直接丟掉；前處理是直接縮放成 704×704（不補邊）、ImageNet mean/std。官方照片上類別一致、框差不到 1 px、分數差不到 0.02。
- 25W 模式把 GPU 鎖在 408 MHz，一定要切 MAXN（`sudo nvpmodel -m 0`）並跑 `sudo jetson_clocks`。這台目前刷的不是 Super 版，沒有 40W／MAXN_SUPER（1173 MHz）。
- 匯出 ONNX 只需要 CPU 版 torch；PyPI 的 aarch64 torch 會拉 CUDA 13 的套件，在 Jetson 上用不到。
- 前視相機固定 15 fps（見 [docs/architecture.md](../../docs/architecture.md#幀率)）。
- 這次的推論、串流腳本只在 Jetson 的 `~/rfdetr_jetson/`，還沒收進 repo；Jetson 版的偵測 Dockerfile 也還沒寫。

## 在本機跑

ROS 節點見 `src/orca_perception`，用 `deploy/laptop` 的 `object_detector` 服務啟動。不經 ROS 直接用：

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install "rfdetr==1.12.0"     # 跟訓練時同版本
```

```python
from rfdetr import RFDETRLarge
from PIL import Image
model = RFDETRLarge(pretrain_weights="weights/checkpoint_best_total.pth", num_classes=8)
det = model.predict(Image.open("某張圖.jpg").convert("RGB"), threshold=0.4)
# det.class_id 是 model.class_names 的 0 起算索引
```
