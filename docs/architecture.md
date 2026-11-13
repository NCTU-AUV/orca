# 架構

## 分層

| 層 | 誰負責 | 頻率 | 內容 |
|---|---|---|---|
| 內迴路 | ArduSub（飛控 CUAV V5） | 約 400 Hz | 姿態估測（EKF3）、姿態與定深控制、8 顆推進器的推力分配 |
| 介面 | `orca_bridge`（mavros + `auv_adapter.py`） | 20 Hz 送搖桿指令 | `/auv_control` 轉成 MAVLink：模式、解鎖、六自由度 effort、目標深度、目標航向 |
| 感知 | `orca_perception` | 跟相機同步 | 影像 → `/detections` |
| 決策 | `orca_mission`（py_trees） | 行為樹 10 Hz、指令 20 Hz | 偵測結果 + 飛控狀態 → `/auv_control`。任務層（行為樹：順序、重試、逾時、就緒檢查、歸零、解鎖）與執行層（TaskRunner + 執行器：送目標深度、目標航向、正規化 surge／sway）分開，見 [src/orca_mission/README.md](../src/orca_mission/README.md) |
| 安全（未來） | `orca_safety` | | 監看飛控與各節點，有問題時中止任務 |

閉迴路放在飛控，是因為飛控內部約 400 Hz，而上層經 mavproxy 拿到的遙測只有約 4 Hz，不適合在上層做姿態和深度的閉迴路。水平方向（前後、左右）目前沒有感測器，是開迴路。

## 介面

- `/auv_control`（`orca_msgs/AuvControl`）：上層唯一的控制出口，欄位說明在 [`AuvControl.msg`](../src/orca_msgs/msg/AuvControl.msg) 的註解。上層要以 20 Hz 持續發送：adapter 超過 0.5 秒沒收到就改送中位（不切模式、不上鎖）。同一時間只能有一個程式發（鍵盤、水池測試腳本、之後的 `orca_mission` 擇一）。
- `/auv_adapter/zero`（`std_srvs/Trigger`）：以當下的深度與航向為 0。剛下水、浮在水面時呼叫一次。
- `/camera/camera/color/image_raw`、`camera_info`：前視相機，名稱跟 `realsense2_camera` 預設一樣，換相機來源時下游不用改。`frame_id` 是 `camera_color_optical_frame`。
- `/bottom_camera/image_raw`、`camera_info`：底部相機，`frame_id` 是 `bottom_camera_optical_frame`。模擬已經有；實機還沒接（RPi 轉接沒有這一路，Jetson 上用 `usb_cam` 之類的 driver 發這兩個 topic 即可）。
- `/detections`（`vision_msgs/Detection2DArray`）：`class_id` 是類別名稱，例如 `gate`、`red_drum`。前視、底部共用這個 topic，`header.frame_id` 分辨是哪一台相機，`header.stamp` 是拍攝時間；bbox 是原始影像的像素座標。
- `/mission/status`（`std_msgs/String`，JSON，2 Hz）：行為樹目前跑到哪個節點、執行器的回饋、深度、航向、指令。

## 部署

程式不綁死在哪台機器上，差別只在 `deploy/` 底下的設定：

| 跑法 | 相機 | 飛控連線（mavros 設定） | 狀態 |
|---|---|---|---|
| `deploy/laptop` + `deploy/rpi-relay` | RPi 讀 D435i → JPEG over TCP → `orca_cam_bridge` | RPi mavproxy 的 UDP 14560 → `mavros_rpi_relay.yaml` | **現行**。相機到偵測實跑過；飛控連線待驗證 |
| `deploy/sim` | （還沒有模擬相機） | SITL 的 mavproxy → `mavros_sitl.yaml` | 可用 |
| Jetson（未建） | `realsense2_camera` 直接讀 | 飛控 USB 直插 → `mavros_serial.yaml` | 主電腦用 RPi 5 還是 Jetson Orin NX 還沒決定 |

從筆電換到 Jetson（Orin NX）時：`front_camera` 換成 `realsense2_camera`，加一個底部相機的服務，`fc_adapter` 改用 `serial`（飛控 USB 直插，不需要 mavproxy），偵測改用 Jetson 版 Dockerfile（尚未寫）。偵測不用 Isaac ROS：RF-DETR 匯出 ONNX → TensorRT engine，在 L4T JetPack 的輕量映像裡推論即可。節點程式不用改。

## 大檔案

- 模型權重放 GitHub Releases，用 `ml/<模型>/fetch_weights.sh` 下載並檢查 sha256。
- 資料集不放進 repo（RF-DETR v2 的資料集約 1.8 GB，在隊上的 Google Drive）。
