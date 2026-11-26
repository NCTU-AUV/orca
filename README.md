# orca

NCTU-AUV 參加 SAUVC（Singapore AUV Challenge）的自主水下載具 **Orca** 的軟體。控制、感知、模擬、模型訓練、飛控參數、部署設定都放在這一個 repo。

```
前視／底部相機 ──> cam_bridge／realsense2_camera ──> orca_perception ──> /detections ──┐
                                                                                     v
                                                   orca_mission（行為樹，py_trees）
                                                                                     │ /auv_control
                                                                                     v
                    ArduSub（AET-H743）<── mavros <── orca_bridge（auv_adapter）
                    姿態、定深內迴路、推力分配
```

姿態與定深的閉迴路、推力分配都交給飛控上的 ArduSub。上層（ROS 2 Humble）只送六自由度 effort、目標深度、目標航向，介面是 [`orca_msgs/AuvControl`](src/orca_msgs/msg/AuvControl.msg)。整體設計見 [docs/architecture.md](docs/architecture.md)。

## 日常指令

需求：Linux、Docker（含 compose plugin）、NVIDIA 驅動與 NVIDIA Container Toolkit，主機不用裝 ROS。第一次使用（RPi 端的設定、需求細節）和這裡沒列的指令見[文件](#文件)。

每個區塊從 repo 根目錄開始，照順序貼就能跑。實機與模擬的 `fc_adapter` 都綁 UDP 14551，不能同時跑。

### 實機

```bash
cd deploy/laptop
./up.sh                                  # 開工：開場次、起服務、開始全程錄影，等飛控、相機、偵測就緒
docker compose exec fc_adapter bash -ic 'ros2 topic echo --once --qos-durability volatile /mavros/state'   # 上岸檢查飛控，要看到 connected: true
./mission.sh                             # 過門（gate_pass），解鎖前列檢查清單、等 Enter
./mission.sh gate_pass_yaw               # 過門：下潛後先轉 yaw 對準門，再鎖航向
./mission.sh gate_pass -p depth=0.5 -p pass_s=6   # -p 覆寫樹的參數
./mark.sh 第2趟 surge 0.4                 # 插一筆標記（錄進 rosbag 和場次的 markers.csv）
./down.sh                                # 收工：場次收尾再全部關掉，不要直接 docker compose down
```

任何時候 Ctrl-C 都會上鎖，船浮上來。

電磁鐵吸放的岸上測試（在 Jetson 上、repo 根目錄，不經 ROS，步驟見 [檔頭](tools/hw_tests/magnet.py)）：

```bash
python3 tools/hw_tests/magnet.py --pin 29
```

### 模擬

```bash
cd deploy/sim
./mission.sh                             # 過門（gate_pass），資格賽場地；起模擬、等就緒、錄 rosbag、跑行為樹一次做完
./mission.sh gate_pass_yaw               # 過門：下潛後先轉 yaw 對準門，再鎖航向
./mission.sh gate_pass -p depth=0.5 -p pass_s=6   # -p 覆寫樹的參數
WORLD=sauvc_final.sdf ./mission.sh       # 決賽主場地
WORLD=sauvc_final.sdf ./mission.sh drop_ball   # 投球：要從看得到 drum 的地方出發（見 src/orca_mission/README.md）
GUI=0 ./mission.sh                       # 不開 Gazebo 視窗
./run.sh                                 # 只起模擬（空池），手動操控用；前景執行，Ctrl-C 結束
WORLD=sauvc_qual.sdf ./run.sh            # 只起模擬，有門、相機與偵測
docker compose --profile vision down     # 全部關掉（只打 docker compose down 不會停相機和偵測）
```

每次 `./mission.sh` 都會重開 `simulator`，船回到出發位置。

### 跑的時候（實機、模擬都一樣）

另開終端機，在 `deploy/laptop` 或 `deploy/sim` 底下：

```bash
xhost +SI:localuser:root                 # 開視窗的工具要先跑一次；模擬的 run.sh、mission.sh 開 Gazebo 視窗時已經跑過
docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py   # 鍵盤操控，按鍵見 tools/teleop/keyboard.py 檔頭
docker compose run --rm gui_tools ros2 run rqt_image_view rqt_image_view /detections/image   # 看畫好偵測框的畫面，左上的下拉選單可以換影像
docker compose --profile mission run --rm mission py-trees-tree-watcher    # 看行為樹每個節點的狀態
docker compose logs -f fc_adapter        # mavros、auv_adapter 的輸出
```

- 同一時間只能有一個程式發 `/auv_control`：行為樹、鍵盤擇一。
- QGroundControl 開著就會自動連上（UDP 14550），joystick 要關掉，否則跟 mavros 的搖桿指令打架。

### 紀錄

- **實機**：`deploy/laptop/logs/session/<場次>/`，一次下水（`./up.sh` 到 `./down.sh`）一個資料夾，全程 rosbag、標記、行為樹的 CSV、飛控參數都在裡面。詳見 [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md#紀錄)。
- **模擬**：`deploy/sim/logs/`，`./mission.sh` 每跑一次，在 `mission/`（CSV）、`bag/`（rosbag）、`referee/`（裁判的事件）各留一份，名稱都是 `<樹>_<時間>`。詳見 [docs/sim/README.md](docs/sim/README.md#紀錄)。
- rosbag 用 [Foxglove](https://foxglove.dev) 開 `.mcap`，layout 在 [`tools/foxglove/orca_mission.json`](tools/foxglove/orca_mission.json)。

## 文件

| 要看什麼 | 文件 |
|---|---|
| 實機：RPi 端的設定、啟動與收工流程、紀錄，以及不常用的指令（水池測試、切片、飛控 `.BIN`） | [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md) |
| 模擬：需求、世界、服務，不常用的指令（隨機佈局、水下效果、水池測試）、驗收、排錯 | [docs/sim/README.md](docs/sim/README.md) |
| SAUVC 2027 的場地、道具、計分、偵測驗證的結果 | [docs/sim/world-2027.md](docs/sim/world-2027.md) |
| 行為樹：架構、怎麼寫一棵樹、參數 | [src/orca_mission/README.md](src/orca_mission/README.md) |
| 自己寫上層控制：以 20 Hz 發 `/auv_control`（欄位見 [`AuvControl.msg`](src/orca_msgs/msg/AuvControl.msg) 的註解），下水時呼叫 `/auv_adapter/zero` 歸零 | [docs/architecture.md](docs/architecture.md) |
| 不經 ROS 直接跑偵測模型、重新訓練 | [ml/rfdetr/README.md](ml/rfdetr/README.md) |
| 改飛控參數：實機改完存回 `real.parm`，影響控制的參數同步到 `sitl.parm` | [firmware/ardusub/README.md](firmware/ardusub/README.md) |
| 舊的計畫書、需求書（已完成，只留參考） | [docs/archive/](docs/archive/) |

## 資料夾

| 路徑 | 內容 |
|---|---|
| `src/` | ROS 2 package（colcon workspace） |
| `src/orca_msgs/` | 自訂介面。所有 package 都依賴它，改介面時相關程式在同一個 commit 一起改 |
| `src/orca_bridge/` | `/auv_control` → mavros → ArduSub；mavros 設定依連線方式分成 `rpi_relay`、`sitl`、`serial` |
| `src/orca_cam_bridge/` | 接收 RPi 轉出的相機影像，發布成跟 `realsense2_camera` 一樣的 topic（只有 RPi 轉接時用） |
| `src/orca_perception/` | 物件偵測節點（RF-DETR），前視、底部相機共用一個模型 |
| `src/orca_mission/` | 行為樹決策層：任務流程寫在 `trees/*.yaml`（旁邊有產生出來的流程圖 `.md`），見 [README](src/orca_mission/README.md) |
| `src/orca_actuators/` | 致動器：放球的電磁鐵（`ball_dropper`，Jetson 40-pin GPIO），見 [README](src/orca_actuators/README.md) |
| `src/orca_session/` | 場次紀錄的共用部分：紀錄放哪裡、標記（`/session/marker`、`markers.csv`）、飛控參數快照、rosbag 切片。實機的 `deploy/laptop/*.sh` 透過 [`deploy/common.sh`](deploy/common.sh) 使用 |
| `sim/` | Gazebo 世界與模型、自寫的 gz 外掛、SITL 啟動腳本 |
| `firmware/ardusub/` | 飛控參數：實機快照、SITL |
| `ml/` | 模型訓練的 notebook、設定、下載權重的腳本。權重放在 GitHub Releases，資料集不放進 repo |
| `docker/` | 各 image 的 Dockerfile |
| `deploy/` | 每種跑法一個資料夾：compose 與機器上的服務設定。`deploy/common.sh` 是各跑法的腳本共用的函式（等待就緒、場次） |
| `tools/` | 水池測試腳本、鍵盤操控（`teleop/keyboard.py`）、系統辨識（`sysid/`）、CAD 轉模擬網格（`cad/`）等不屬於 ROS package 的工具 |
| `docs/` | 架構、操作、排錯紀錄；`docs/archive/` 是已完成的計畫書、需求書 |

之後會加的 package（用到時再建）：`orca_safety`（漏水、電池、kill switch、節點看門狗）、`orca_localization`、`orca_description`（URDF、TF）、`orca_bringup`（同一台機器上多個節點的 launch）。

## 開發

```bash
cd src/orca_mission
python3 -m pytest test                   # 行為樹測試：含用簡化模型跑完整過門的情境，CI 也會跑；要有 py_trees（或在 mission 容器裡跑）
```

- `src/` 會掛進容器，而且 workspace 用 `--symlink-install` build，改 `.py` 之後重開該服務就生效。改了 `.msg`、`package.xml`、`setup.py` 或 Dockerfile 才要 `docker compose build`。
- 同一台電腦上的 ROS 節點都用 `ROS_LOCALHOST_ONLY=1`。要跟別的 ROS 系統隔開（例如同時有人在跑），設不同的 `ROS_DOMAIN_ID`。

## 授權

[Apache-2.0](LICENSE)
