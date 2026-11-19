# orca

NCTU-AUV 參加 SAUVC（Singapore AUV Challenge）的自主水下載具 **Orca** 的軟體。控制、感知、模擬、模型訓練、飛控參數、部署設定都放在這一個 repo。

```
前視／底部相機 ──> cam_bridge／realsense2_camera ──> orca_perception ──> /detections ──┐
                                                                                     v
                                                   orca_mission（行為樹，py_trees）
                                                                                     │ /auv_control
                                                                                     v
                     ArduSub（CUAV V5）<── mavros <── orca_bridge（auv_adapter）
                     姿態、定深內迴路、推力分配
```

姿態與定深的閉迴路、推力分配都交給飛控上的 ArduSub。上層（ROS 2 Humble）只送六自由度 effort、目標深度、目標航向，介面是 [`orca_msgs/AuvControl`](src/orca_msgs/msg/AuvControl.msg)。整體設計見 [docs/architecture.md](docs/architecture.md)。

## 使用方法

需求：Linux、Docker（含 compose plugin）、NVIDIA 驅動與 NVIDIA Container Toolkit。主機不用裝 ROS。

這一節列出所有用法，每項的細節見連結的文件或程式檔頭。以下 `docker compose` 指令都在對應的 `deploy/<跑法>/` 底下執行。

### 跑法

| 情境 | 位置 | 詳細說明 |
|---|---|---|
| 實機：ROS 全部跑在筆電，RPi 只當轉接（**現行做法**） | [`deploy/laptop/`](deploy/laptop/)、[`deploy/rpi-relay/`](deploy/rpi-relay/) | [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md) |
| 模擬：Gazebo + ArduSub SITL | [`deploy/sim/`](deploy/sim/) | [docs/sim/README.md](docs/sim/README.md) |

兩種跑法的 `fc_adapter` 都綁 UDP 14551，不能同時跑。

各 deploy 的服務名稱照功能取：`simulator`（Gazebo + SITL）、`fc_adapter`（mavros + auv_adapter）、`front_camera`／`sim_cameras`（相機）、`object_detector`（RF-DETR）、`recorder`（rosbag）、`mission`（行為樹）、`gui_tools`（看畫面、鍵盤操控等臨時工具）。

**實機**

```bash
# RPi 端（只要做一次，在 deploy/rpi-relay/ 執行；RPi 的 ssh 名稱預設 rpi，可用 RPI_HOST 改）
./deploy.sh                    # 安裝 cam-relay 並設成開機自動跑（取代 realsense-web）
./deploy.sh revert             # 改回 realsense-web
scp mavproxy-udpin.sh rpi: && ssh -t rpi sudo bash mavproxy-udpin.sh   # mavproxy 加 udpin 14560

# 筆電端（在 deploy/laptop/ 執行）
../../ml/rfdetr/fetch_weights.sh            # 第一次：下載偵測模型權重
./up.sh                                     # 開場次（或沿用）、起服務、開始全程錄影並等就緒；映像不存在或過期才 build，已經在跑的不重開
./up.sh --build                             # 不管指紋，全部重 build（過期的判斷見 docker/build-hash.sh）
./mission.sh                                # 接著開始過門（會先跑 ./up.sh）
./pool_test.sh water_depth                  # 水池測試腳本（見下面的「功能」）
./mark.sh <內容>                            # 插一筆標記（錄進 rosbag 和場次的 markers.csv）
./slice.sh --match <文字>                   # 從場次的 rosbag 切出一段，例如一次任務
docker compose up -d front_camera object_detector   # 只跑影像與偵測（飛控沒接時；不開場次、不錄）
docker compose logs -f object_detector      # 每 5 秒印 fps 與延遲
docker compose exec fc_adapter bash -ic 'ros2 topic echo --once /mavros/state'   # 上岸檢查飛控，要看到 connected: true
./down.sh                                   # 收工：場次收尾再全部關掉（不要直接 docker compose down）
```

**模擬**

```bash
# 在 deploy/sim/ 執行
./run.sh                       # build（第一次）並啟動 simulator、fc_adapter，前景執行，Ctrl-C 結束
GUI=0 ./run.sh                 # 不開 Gazebo 視窗
WORLD=<檔名> ./run.sh          # 換 sim/worlds/ 底下的世界檔
./mission.sh                   # 行為樹過門：SAUVC 2027 資格賽（sauvc_qual.sdf），見 docs/sim/README.md
```

QGroundControl 開著就會自動連上（UDP 14550），joystick 要關掉，否則跟 mavros 的搖桿指令打架。

### 功能

以下在實機與模擬都能用，除非特別註明。同一時間只能有一個程式發 `/auv_control`（鍵盤、測試腳本、過門擇一）。

| 功能 | 指令 | 說明 |
|---|---|---|
| 過門 | `./mission.sh` | 起服務、等飛控與偵測就緒（實機是 `./up.sh`：映像不存在或過期才 build，已經在跑的容器不重開），再用 `mission` 服務跑 [`src/orca_mission/trees/gate_pass.yaml`](src/orca_mission/trees/gate_pass.yaml)（流程圖 [gate_pass.md](src/orca_mission/trees/gate_pass.md)）。實機解鎖前列檢查清單、等 Enter。`-p` 覆寫參數，例如 `./mission.sh gate_pass -p depth=0.5 -p pass_s=6`；`./mission.sh gate_pass_yaw` 是下潛後先轉 yaw 對準門、再鎖航向的版本（[`gate_pass_yaw.yaml`](src/orca_mission/trees/gate_pass_yaw.yaml)，流程圖 [gate_pass_yaw.md](src/orca_mission/trees/gate_pass_yaw.md)）；`./mission.sh smoke_test` 是不需要門的動作測試。說明見 [src/orca_mission/README.md](src/orca_mission/README.md) |
| 水池測試：定深 | 實機 `./pool_test.sh water_depth`；模擬 `docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'` | 預設 0.15 → 0.20 → 0.10 m。要改就把深度接在後面，例如 `./pool_test.sh water_depth 0.1 0.2` |
| 水池測試：定向 | 同上，腳本換成 `water_heading` | 定深 0.15 m，預設 90 → 0 → −90 → 0°。要改就把角度接在後面，例如 `./pool_test.sh water_heading 45 0` |
| 水池測試：小任務 | 同上，腳本換成 `water_mission` | 下潛 → 直走 → 轉 180° → 走回來 |
| 鍵盤操控 | `docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py` | pygame 視窗，按鍵與安全機制見 [`keyboard.py`](tools/teleop/keyboard.py) 檔頭。可加 `--gain 0.3`（初始推力）、`--max-depth 0.25`（定深上限） |
| 看相機畫面（只限實機） | `docker compose run --rm gui_tools ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw` | 原始影像。topic 換成 `/detections/image` 是畫好偵測框的畫面，視窗左上的下拉選單也能切換 |
| 行為樹測試 | `cd src/orca_mission && python3 -m pytest test` | 要有 py_trees（或在 `mission` 容器裡跑）。含用簡化模型跑完整過門的情境，CI 也會跑 |

- 開視窗的功能（鍵盤操控、看畫面）要先在主機跑 `xhost +SI:localuser:root`；模擬的 `./run.sh` 已經跑過。
- 測試腳本和過門都會先列檢查清單、歸零（以浮在水面時的深度與航向為 0）。任何時候 Ctrl-C 都會上鎖，船浮上來。
- 紀錄存在 `deploy/<跑法>/logs/` 底下（`<跑法>` 是 `laptop` 或 `sim`）。實機和模擬目前的錄法不同：
  - **實機**以場次為單位：一次下水（`./up.sh` 到 `./down.sh`）的全部紀錄都在 `logs/session/<場次>/`，包括全程 rosbag（每 5 分鐘一個 `.mcap`，含相機的原始 JPEG，估計約 9 GB／小時）、標記 `markers.csv`、行為樹和水池測試的 CSV、開始與結束時的飛控參數、程式版本、各容器的終端輸出。標記來自 `./mark.sh`、鍵盤操控的 M 鍵、行為樹與水池測試的開始和結束；只要其中一段就用 `./slice.sh` 切出來。詳見 [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md#紀錄)。
  - **模擬**只在跑行為樹時錄：`./mission.sh` 樹開始前開始錄，樹結束（含 Ctrl-C）、上鎖、船浮上水面後自動停止並收尾（最多等 30 秒），一次任務一個 `.mcap`，存在 `logs/bag/<樹>_<時間>/`，跟 `logs/mission/` 的 CSV 同名；水池測試的 CSV 在 `logs/fc_adapter/`。影像只錄畫好框的 JPEG。不想錄用 `RECORD=0 ./mission.sh`。詳見 [docs/sim/README.md](docs/sim/README.md#log)。
  - 用 Foxglove 開 `.mcap`（還在寫的檔可能打不開或缺最後幾秒）。

### 其他

| 要做的事 | 怎麼做 |
|---|---|
| 自己寫上層控制 | 以 20 Hz 發 `/auv_control`，欄位見 [`AuvControl.msg`](src/orca_msgs/msg/AuvControl.msg) 的註解；下水時呼叫 `/auv_adapter/zero` 歸零。介面見 [docs/architecture.md](docs/architecture.md) |
| 不經 ROS 直接跑偵測模型、重新訓練 | [ml/rfdetr/README.md](ml/rfdetr/README.md) |
| 改飛控參數 | [firmware/ardusub/README.md](firmware/ardusub/README.md)：實機改完存回 `real.parm`，影響控制的參數同步到 `sitl.parm` |
| 模擬的 log、已知問題、改設定後怎麼生效 | [docs/sim/README.md](docs/sim/README.md) |

## 資料夾

| 路徑 | 內容 |
|---|---|
| `src/` | ROS 2 package（colcon workspace） |
| `src/orca_msgs/` | 自訂介面。所有 package 都依賴它，改介面時相關程式在同一個 commit 一起改 |
| `src/orca_bridge/` | `/auv_control` → mavros → ArduSub；mavros 設定依連線方式分成 `rpi_relay`、`sitl`、`serial` |
| `src/orca_cam_bridge/` | 接收 RPi 轉出的相機影像，發布成跟 `realsense2_camera` 一樣的 topic（只有 RPi 轉接時用） |
| `src/orca_perception/` | 物件偵測節點（RF-DETR），前視、底部相機共用一個模型 |
| `src/orca_mission/` | 行為樹決策層：任務流程寫在 `trees/*.yaml`（旁邊有產生出來的流程圖 `.md`），見 [README](src/orca_mission/README.md) |
| `src/orca_session/` | 場次紀錄的共用部分：紀錄放哪裡、標記（`/session/marker`、`markers.csv`）、飛控參數快照、rosbag 切片。實機的 `deploy/laptop/*.sh` 透過 [`deploy/common.sh`](deploy/common.sh) 使用 |
| `sim/` | Gazebo 世界與模型、SITL 啟動腳本 |
| `firmware/ardusub/` | 飛控參數：實機快照、SITL |
| `ml/` | 模型訓練的 notebook、設定、下載權重的腳本。權重放在 GitHub Releases，資料集不放進 repo |
| `docker/` | 各 image 的 Dockerfile |
| `deploy/` | 每種跑法一個資料夾：compose 與機器上的服務設定。`deploy/common.sh` 是各跑法的腳本共用的函式（等待就緒、場次） |
| `tools/` | 水池測試腳本、鍵盤操控（`teleop/keyboard.py`）等不屬於 ROS package 的工具 |
| `docs/` | 架構、操作、排錯紀錄 |

之後會加的 package（用到時再建）：`orca_safety`（漏水、電池、kill switch、節點看門狗）、`orca_localization`、`orca_actuators`（電磁鐵、投放）、`orca_description`（URDF、TF）、`orca_bringup`（同一台機器上多個節點的 launch）。

## 開發

- `src/` 會掛進容器，而且 workspace 用 `--symlink-install` build，改 `.py` 之後重開該服務就生效。改了 `.msg`、`package.xml`、`setup.py` 或 Dockerfile 才要 `docker compose build`。
- 同一台電腦上的 ROS 節點都用 `ROS_LOCALHOST_ONLY=1`。要跟別的 ROS 系統隔開（例如同時有人在跑），設不同的 `ROS_DOMAIN_ID`。
- 實機與模擬的 `fc_adapter` 都綁 UDP 14551，不能同時跑。

## 授權

[Apache-2.0](LICENSE)
