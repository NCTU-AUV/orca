# Jetson：AUV 獨自跑（deploy/jetson）

新機體的主電腦是 Jetson Orin NX（JetPack 6.2.1），相機（D435i）和飛控（AET-H743-Basic）都直接插在 Jetson 上，所有 ROS 節點在 Jetson 的 Docker 裡跑。下水時不接筆電：任務從岸上用 ssh 啟動、在背景倒數，倒數期間拔網路線、放船下水。筆電只在岸上用：同步程式、下指令、看畫面、拉紀錄。

```
                          Jetson（deploy/jetson，ROS 2 Humble，全部在 Docker）
 D435i ─USB─> camera（realsense2_camera）─> /camera/camera/color/* ─> object_detector（TensorRT）─> /detections
                                                                                       │
 AET-H743 ─USB─> fc_adapter（mavros serial + auv_adapter）<── /auv_control ── mission（背景，orca-mission）
                 recorder（全程 rosbag）、marker_log、foxglove_bridge（127.0.0.1:8765）

 筆電 ─ssh（網路線或 Wi-Fi）─> 下指令；view.sh 轉埠看 Foxglove；pull.sh 拉紀錄；sync.sh 同步程式
```

日常的指令（同步、開工、任務、中止、收工、看畫面、拉紀錄）在 [README 的「Jetson」](../README.md#jetsonauv-獨自跑)。

## 第一次設定（每台 Jetson 一次）

在筆電上，repo 根目錄：

```bash
ml/rfdetr/export_onnx.sh                               # .pth → ONNX，並跟 PyTorch 版比對（只要權重沒換就不用重跑）
deploy/jetson/sync.sh                                  # 程式和模型同步到 Jetson 的 ~/workspace/orca
ssh -t orin workspace/orca/deploy/jetson/setup.sh      # 裝 Docker（要 sudo 密碼）
ssh orin                                               # 以下在 Jetson 上
cd ~/workspace/orca/deploy/jetson
./up.sh                                                # 第一次：build 映像（要上網，約 40 分鐘）、轉 TensorRT 引擎（約 2.5 分鐘）並驗證
./autostart.sh enable                                  # 開機自動起相機、偵測、飛控（不開場次、不錄影）
```

| 檔案 | 內容 |
|---|---|
| `setup.sh` | 裝 Docker 27.5.1（擋住升級：JetPack 6 的 kernel 沒有 `iptable_raw`，新版可能起不來）、登記 nvidia runtime、帳號加進 docker 群組 |
| `sync.sh` | 筆電 → Jetson，`rsync --delete`，含 `.git` 和模型。Jetson 上的 `logs/`、`*.engine` 不動 |
| `up.sh`、`mission.sh`、`stop.sh`、`down.sh` | 開工、任務、中止、收工 |
| `autostart.sh` | systemd 使用者服務 `orca.service`，開機起服務（要 linger，`setup.sh` 前就開了） |
| `view.sh`、`pull.sh` | 筆電上跑：Foxglove 轉埠、拉場次紀錄 |
| `mark.sh`、`slice.sh`、`pool_test.sh` | 連到 `deploy/laptop` 的同名腳本，跟跑法無關 |

## 偵測：TensorRT

- 筆電上 `ml/rfdetr/export_onnx.sh` 把 `checkpoint_best_total.pth` 匯出成 `rfdetr-large.onnx`（輸入 704×704），同時產生 `rfdetr-large.json`（類別名稱）和 `check_reference.json`（PyTorch 版在 `docs/sim/reference/` 14 張圖上的結果）。
- Jetson 上 `up.sh` 第一次跑時用 `build_engine` 轉成 FP16 引擎 `rfdetr-large.sm87.trt10.3.0.fp16.engine`，再跟 `check_reference.json` 比對（`ml/rfdetr/check_export.py`），對不上就停下來。引擎只能在同一種 GPU、同一版 TensorRT 上用，檔名帶這兩個，換了會自動重轉。
- 前後處理用 numpy 寫在 `src/orca_perception/orca_perception/rfdetr_trt.py`，照 rfdetr 1.12 的 `predict()`，Jetson 的映像不裝 torch。偵測節點看權重的副檔名選後端：`.pth` 是 PyTorch（筆電、模擬），`.onnx` 是 TensorRT（Jetson）。
- 映像 `docker/perception-jetson.Dockerfile`：NVIDIA 的 `l4t-tensorrt:r10.3.0-runtime`（CUDA 12.6、TensorRT 10.3，跟 JetPack 6.2 同版）加 ROS 2 Humble。JetPack 6 的 nvidia runtime 只把驅動掛進容器，CUDA、TensorRT 要在映像裡。

換了權重：筆電上 `ml/rfdetr/export_onnx.sh`、`deploy/jetson/sync.sh`，Jetson 上刪掉舊的 `*.engine`、`*.checked` 再 `./up.sh`。

## 任務怎麼跑

`./mission.sh` 先跑 `./up.sh`，在終端機列出樹的檢查清單，按 Enter 後用 `docker compose run -d --name orca-mission` 在背景啟動 `mission <樹> --start-delay <秒>`：

- 行為樹照常先等就緒（`WaitReady`），檢查清單那一步（`Confirm`）改成倒數，倒數完才歸零、解鎖、下潛。歸零在倒數之後，所以是船放到水面、靜止時的深度與航向。
- ssh 斷線（拔網路線）不影響任務。重新連上後 `docker logs -f orca-mission` 接著看。
- 任務結束後容器留著，下一次 `./mission.sh` 才刪，`./down.sh` 會把它的輸出存進場次的 `containers.log`。
- `./stop.sh` 送 SIGTERM，mission 收尾（MANUAL、全 0、上鎖），結束的標記寫「SIGTERM」。

出事時讓船上鎖、浮上來的三層保護見 [architecture.md 的「分層」](architecture.md#分層)。

## 紀錄

場次的規則、目錄結構跟筆電一樣（`deploy/common.sh` 檔頭、[laptop-rpi-relay.md 的「紀錄」](laptop-rpi-relay.md#紀錄)），在 Jetson 的 `deploy/jetson/logs/session/`，筆電用 `deploy/jetson/pull.sh` 拉回同一個路徑。差別：

- 相機的壓縮影像由 Jetson 自己壓（image_transport），每秒約 1.9 MB；全部合計約每小時 9～10 GB。Jetson 的 NVMe 剩約 400 GB。
- 飛控的 `.BIN` 還不會自動拉（筆電是經 RPi 的 mavproxy 用 MAVFTP 拉）。之後打算用飛控的第二個 USB 序列埠（`…-if02`）。

## 實測數字（2026-11-26，Orin NX 16 GB、25 W 模式）

| 項目 | 數字 |
|---|---|
| 偵測 | 約 23～25 fps，推論 38～40 ms，拍攝到 `/detections` 約 90～97 ms |
| 相機 | 640×480 彩色約 29 fps |
| 飛控遙測 | `imu/data`、`rel_alt`、`rc/out` 都是 20 Hz |
| 負載 | GPU 約 30%、記憶體 3.5／15.6 GB、整機約 18 W、48 °C |
| 引擎 | 轉檔 146 秒，62 MB，載入不到 1 秒 |

## 踩過的坑

- **改了 `MAV1_*` 要重開飛控**：遙測頻率寫進參數後不會馬上生效，重開才會。重開後 mavros 自己會重新連上，`fc_adapter` 不用重開。
- **`docker compose run` 會讀標準輸入**：`mission.sh` 列檢查清單時沒擋掉的話，會吃掉等 Enter 要讀的輸入。腳本裡跑的 `docker compose run`／`exec` 都加 `< /dev/null`。
- **標記漏寫進 `markers.csv`**：Jetson 上 DDS 的配對比筆電慢，發布端看到訂閱者時，marker_log 那端可能還沒配對好。`Marker` 等訂閱者到齊後再等 0.3 秒才發。
- **`import tensorrt` 在 docker build 時會失敗**：要主機的驅動函式庫（`libnvdla_compiler.so`），nvidia runtime 只在容器跑起來時掛進來。Dockerfile 只檢查檔案在不在。
- **剛加進 docker 群組時，使用者的 systemd 沒有這個群組**：它是開機時啟動的，要重開機才會更新。`orca.service` 用 `sg docker` 跑，不用等重開機。
- **D435i 接在 USB2 也能跑**：640×480 彩色 30 fps 夠用；要開深度影像就要接 USB3。
- **別的程式會佔相機和飛控**：隊友的 `~/workspace/hw_check` 面板（已停用開機自啟，`systemctl --user enable orca-dashboard` 可以開回來）、`~/rfdetr_jetson/stream_rs.py`。跑 orca 時這些要關掉。

## 還沒做

- 飛控的 `.BIN` 自動拉回場次。
- DVL（Nortek DVL1000，192.168.50.2）、底部相機。
- 岸上用 QGC 看飛控（mavros 的 `gcs_url` 轉到筆電，或用飛控的第二個 USB 序列埠）。
