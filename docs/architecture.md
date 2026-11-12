# 架構

## 分層

| 層 | 誰負責 | 頻率 | 內容 |
|---|---|---|---|
| 內迴路 | ArduSub（飛控 CUAV V5） | 約 400 Hz | 姿態估測（EKF3）、姿態與定深控制、8 顆推進器的推力分配 |
| 介面 | `orca_bridge`（mavros + `auv_adapter.py`） | 20 Hz 送搖桿指令 | `/auv_control` 轉成 MAVLink：模式、解鎖、六自由度 effort、目標深度、目標航向 |
| 感知 | `orca_perception` | 跟相機同步 | 影像 → `/detections` |
| 決策（未來） | `orca_mission` | | 偵測結果 → `/auv_control` |
| 安全（未來） | `orca_safety` | | 監看飛控與各節點，有問題時中止任務 |

閉迴路放在飛控，是因為飛控內部約 400 Hz，而上層經 mavproxy 拿到的遙測只有約 4 Hz，不適合在上層做姿態和深度的閉迴路。水平方向（前後、左右）目前沒有感測器，是開迴路。

## 介面

- `/auv_control`（`orca_msgs/AuvControl`）：上層唯一的控制出口，欄位說明在 [`AuvControl.msg`](../src/orca_msgs/msg/AuvControl.msg) 的註解。
- `/auv_adapter/zero`（`std_srvs/Trigger`）：以當下的深度與航向為 0。剛下水、浮在水面時呼叫一次。
- `/camera/camera/color/image_raw`、`camera_info`：名稱跟 `realsense2_camera` 預設一樣，換相機來源時下游不用改。
- `/detections`（`vision_msgs/Detection2DArray`）：`class_id` 是類別名稱，例如 `gate`、`red_drum`。

## 部署

程式不綁死在哪台機器上，差別只在 `deploy/` 底下的設定：

| 跑法 | 相機 | 飛控連線（mavros 設定） | 狀態 |
|---|---|---|---|
| `deploy/laptop` + `deploy/rpi-relay` | RPi 讀 D435i → JPEG over TCP → `orca_cam_bridge` | RPi mavproxy 的 UDP 14560 → `mavros_rpi_relay.yaml` | **現行**。相機到偵測實跑過；飛控連線待驗證 |
| `deploy/sim` | （還沒有模擬相機） | SITL 的 mavproxy → `mavros_sitl.yaml` | 可用 |
| Jetson（未建） | `realsense2_camera` 直接讀 | 飛控 USB 直插 → `mavros_serial.yaml` | 主電腦用 RPi 5 還是 Jetson Orin NX 還沒決定 |

從筆電換到 Jetson 時：`cam_bridge` 換成 `realsense2_camera`，bridge 改用 `serial`，偵測改用 Jetson 版 Dockerfile（arm64 + JetPack 的 PyTorch，尚未寫）。節點程式不用改。

## 大檔案

- 模型權重放 GitHub Releases，用 `ml/<模型>/fetch_weights.sh` 下載並檢查 sha256。
- 資料集不放進 repo（RF-DETR v2 的資料集約 1.8 GB，在隊上的 Google Drive）。
