# orca

NCTU-AUV 參加 SAUVC（Singapore AUV Challenge）的自主水下載具 **Orca** 的軟體。控制、感知、模擬、模型訓練、飛控參數、部署設定都放在這一個 repo。

```
相機 ──> cam_bridge／realsense2_camera ──> orca_perception ──> /detections ──┐
                                                                           v
                                                         orca_mission（未來）
                                                                           │ /auv_control
                                                                           v
                     ArduSub（CUAV V5）<── mavros <── orca_bridge（auv_adapter）
                     姿態、定深內迴路、推力分配
```

姿態與定深的閉迴路、推力分配都交給飛控上的 ArduSub。上層（ROS 2 Humble）只送六自由度 effort、目標深度、目標航向，介面是 [`orca_msgs/AuvControl`](src/orca_msgs/msg/AuvControl.msg)。整體設計見 [docs/architecture.md](docs/architecture.md)。

## 現在怎麼跑

| 情境 | 位置 | 說明 |
|---|---|---|
| 實機：ROS 全部跑在筆電，RPi 只當轉接（**現行做法**） | [`deploy/laptop/`](deploy/laptop/) | [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md) |
| 模擬：Gazebo + ArduSub SITL | [`deploy/sim/`](deploy/sim/) | [docs/sim/README.md](docs/sim/README.md) |

需求：Linux、Docker（含 compose plugin）、NVIDIA 驅動與 NVIDIA Container Toolkit。主機不用裝 ROS。

```bash
ml/rfdetr/fetch_weights.sh          # 第一次：下載偵測模型權重
cd deploy/laptop
docker compose up -d                # cam_bridge、detector、bridge
```

## 資料夾

| 路徑 | 內容 |
|---|---|
| `src/` | ROS 2 package（colcon workspace） |
| `src/orca_msgs/` | 自訂介面。所有 package 都依賴它，改介面時相關程式在同一個 commit 一起改 |
| `src/orca_bridge/` | `/auv_control` → mavros → ArduSub；mavros 設定依連線方式分成 `rpi_relay`、`sitl`、`serial` |
| `src/orca_cam_bridge/` | 接收 RPi 轉出的相機影像，發布成跟 `realsense2_camera` 一樣的 topic（只有 RPi 轉接時用） |
| `src/orca_perception/` | 物件偵測節點（RF-DETR） |
| `sim/` | Gazebo 世界與模型、SITL 啟動腳本 |
| `firmware/ardusub/` | 飛控參數：實機快照、SITL |
| `ml/` | 模型訓練的 notebook、設定、下載權重的腳本。權重放在 GitHub Releases，資料集不放進 repo |
| `docker/` | 各 image 的 Dockerfile |
| `deploy/` | 每種跑法一個資料夾：compose 與機器上的服務設定 |
| `tools/` | 水池測試腳本等不屬於 ROS package 的工具 |
| `docs/` | 架構、操作、排錯紀錄 |

之後會加的 package（用到時再建）：`orca_mission`（行為樹或 FSM 決策）、`orca_safety`（漏水、電池、kill switch、節點看門狗）、`orca_localization`、`orca_actuators`（電磁鐵、投放）、`orca_description`（URDF、TF）、`orca_bringup`（同一台機器上多個節點的 launch）。

## 開發

- `src/` 會掛進容器，而且 workspace 用 `--symlink-install` build，改 `.py` 之後重開該服務就生效。改了 `.msg`、`package.xml`、`setup.py` 或 Dockerfile 才要 `docker compose build`。
- 同一台電腦上的 ROS 節點都用 `ROS_LOCALHOST_ONLY=1`。要跟別的 ROS 系統隔開（例如同時有人在跑），設不同的 `ROS_DOMAIN_ID`。
- 實機的 bridge 與模擬的 bridge 都綁 UDP 14551，不能同時跑。

## 授權

[Apache-2.0](LICENSE)
