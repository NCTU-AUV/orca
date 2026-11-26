# 架構

## 分層

| 層 | 誰負責 | 頻率 | 內容 |
|---|---|---|---|
| 內迴路 | ArduSub（飛控 AET-H743-Basic） | 約 400 Hz | 姿態估測（EKF3）、姿態與定深控制、8 顆推進器的推力分配 |
| 介面 | `orca_bridge`（mavros + `auv_adapter.py`） | 20 Hz 送搖桿指令 | `/auv_control` 轉成 MAVLink：模式、解鎖、六自由度 effort、目標深度、目標航向 |
| 感知 | `orca_perception` | 跟相機同步，前視 15 fps | 影像 → `/detections` |
| 決策 | `orca_mission`（py_trees） | 行為樹 10 Hz、指令 20 Hz | 偵測結果 + 飛控狀態 → `/auv_control`。任務層（行為樹：順序、重試、逾時、就緒檢查、歸零、解鎖）與執行層（TaskRunner + 執行器：送目標深度、目標航向、正規化 surge／sway）分開，見 [src/orca_mission/README.md](../src/orca_mission/README.md) |
| 安全（未來） | `orca_safety` | | 監看飛控與各節點，有問題時中止任務 |

AUV 獨自在水裡時沒有人能按 Ctrl-C，出事時靠三層讓船上鎖、浮上來：

1. `orca_mission`：樹結束（完成、逾時、失敗）、程式出錯、收到 SIGTERM（`./stop.sh`）都會 MANUAL、全 0、上鎖。
2. `auv_adapter`：上層 5 秒沒送 `/auv_control`（mission 被強制結束、沒走收尾）就上鎖。
3. 飛控：3 秒收不到搖桿指令（`FS_PILOT_INPUT 2`，mavros 或 adapter 掛掉、Jetson 當機）、5 秒沒有地面站心跳（`FS_GCS_ENABLE 2`）就上鎖。

閉迴路放在飛控，是因為飛控內部約 400 Hz，而上層經 mavproxy 拿到的遙測只有 20 Hz，還多了 USB 網卡和筆電的延遲，不適合在上層做姿態和深度的閉迴路。水平方向（前後、左右）目前沒有感測器，是開迴路。

## 介面

- `/auv_control`（`orca_msgs/AuvControl`）：上層唯一的控制出口，欄位說明在 [`AuvControl.msg`](../src/orca_msgs/msg/AuvControl.msg) 的註解。上層要以 20 Hz 持續發送：adapter 超過 0.5 秒沒收到就改送中位（不切模式、不上鎖），超過 5 秒而且飛控還解鎖著就上鎖（上層被強制結束、沒走收尾時，船才會浮上來）。同一時間只能有一個程式發（鍵盤、水池測試腳本、之後的 `orca_mission` 擇一）。
- `/auv_adapter/zero`（`std_srvs/Trigger`）：以當下的深度與航向為 0。剛下水、浮在水面時呼叫一次。
- `/camera/camera/color/image_raw`、`camera_info`：前視相機，名稱跟 `realsense2_camera` 預設一樣，換相機來源時下游不用改。`frame_id` 是 `camera_color_optical_frame`。
- `/bottom_camera/image_raw`、`camera_info`：底部相機，`frame_id` 是 `bottom_camera_optical_frame`。模擬已經有；實機還沒接（RPi 轉接沒有這一路，Jetson 上之後用 `usb_cam` 之類的 driver 發這兩個 topic 即可）。
- `/detections`（`vision_msgs/Detection2DArray`）：`class_id` 是類別名稱，例如 `gate`、`red_drum`。前視、底部共用這個 topic，`header.frame_id` 分辨是哪一台相機，`header.stamp` 是拍攝時間；bbox 是原始影像的像素座標。
- `/mission/status`（`std_msgs/String`，JSON，2 Hz）：行為樹目前跑到哪個節點、執行器的回饋、深度、航向、指令。

## 幀率

前視相機固定 **15 fps**（2026-11-26 決定），偵測每一幀都處理，所以偵測也是 15 fps。

- **為什麼**：RF-DETR Large（704、TensorRT FP16）在 Jetson Orin NX（MAXN、`jetson_clocks`）上每幀 28 ms，其中 GPU 推論約 24 ms。30 fps 時 GPU 約 73% 的時間在跑偵測；15 fps 約 36%，留一半給底部相機、其他節點。整機功耗實測 19.9 W → 14.1 W。
- **犧牲**：`/detections` 每 66 ms 更新一次（30 fps 是 33 ms），決策拿到的資訊平均舊約 17 ms；船 0.5 m/s 時兩次更新間移動約 3 cm。單幀從拍攝到偵測出來的時間不變。決策層不受影響：行為樹 10 Hz、判斷用「持續幾秒」而不是「連續幾幀」；只是偵測有效期 0.5 秒內的幀數從約 15 變約 7。
- **要注意**：相機 15 fps 時自動曝光最長可到約 66 ms，水下暗時可能動態模糊（細的 flare 最明顯）。真的遇到時，改成相機 30 fps、偵測節點每兩幀處理一幀，或限制曝光上限。
- **在哪裡設定**：只在相機來源設定，偵測與決策的程式不用改。
  - 現行（RPi 轉接）：[`deploy/rpi-relay/cam_relay.py`](../deploy/rpi-relay/cam_relay.py) 的 `--fps`（預設 15）。
  - 模擬：[`sim/models/orca_cam/model.sdf`](../sim/models/orca_cam/model.sdf) 前視相機 15 Hz。
  - Jetson：[`deploy/jetson/compose.yaml`](../deploy/jetson/compose.yaml) 的 `camera` 服務，`rgb_camera.color_profile:=640,480,15`。
- 確認實際幀率：`docker compose logs -f object_detector` 每 5 秒印一次 fps。

## 部署

程式不綁死在哪台機器上，差別只在 `deploy/` 底下的設定：

| 跑法 | 相機 | 飛控連線（mavros 設定） | 狀態 |
|---|---|---|---|
| `deploy/laptop` + `deploy/rpi-relay` | RPi 讀 D435i → JPEG over TCP → `orca_cam_bridge` | RPi mavproxy 的 UDP 14560 → `mavros_rpi_relay.yaml` | 舊機體用。相機到偵測、飛控、場次紀錄都實跑過 |
| `deploy/sim` | （還沒有模擬相機） | SITL 的 mavproxy → `mavros_sitl.yaml` | 可用 |
| `deploy/jetson`（Orin NX） | `realsense2_camera` 直接讀 | 飛控 USB 直插 → `mavros_serial.yaml` | 新機體（2026-12）用，AUV 獨自跑。見 [jetson.md](jetson.md) |

Jetson 跟筆電的差別只在 `deploy/jetson`：相機改用 `realsense2_camera`（topic 名稱一樣），`fc_adapter` 用 `serial`，偵測改用 TensorRT（RF-DETR 在筆電匯出 ONNX，Jetson 上轉成 FP16 引擎，`docker/perception-jetson.Dockerfile`）。任務由 ssh 啟動、在背景倒數後開始，拔掉網路線也繼續跑。節點程式共用，偵測節點依權重的副檔名選後端（`.pth` PyTorch、`.onnx` TensorRT）。

## 大檔案

- 模型權重放 GitHub Releases，用 `ml/<模型>/fetch_weights.sh` 下載並檢查 sha256。
- 資料集不放進 repo（RF-DETR v2 的資料集約 1.8 GB，在隊上的 Google Drive）。
