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
| `config/training_config.json` | 完整的訓練與模型設定，含 `class_names` |
| `config/hparams.yaml`、`config/run_config.json` | 訓練參數 |
| `config/metrics.csv` | 每個 epoch 的指標 |
| `notebook/train_rfdetr_v2.ipynb` | 訓練、評估、視覺化、匯出 ONNX（Colab，資料從 Google Drive 讀取；輸出已清掉） |

## 資料集

約 1.8 GB，不放進 repo，在隊上 Google Drive 的 `SAUVC_RFDETR/`。Roboflow COCO 格式，`train/`、`valid/`、`test/` 各自有影像和 `_annotations.coco.json`。train 25,220 筆（真實影像約 84%、合成約 11%），valid、test 只有真實影像，按來源影片切分。

## 在本機跑

ROS 節點見 `src/orca_perception`，用 `deploy/laptop` 的 `detector` 服務啟動。不經 ROS 直接用：

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
