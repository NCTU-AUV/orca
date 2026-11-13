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

## 使用方法

需求：Linux、Docker（含 compose plugin）、NVIDIA 驅動與 NVIDIA Container Toolkit。主機不用裝 ROS。

這一節列出所有用法，每項的細節見連結的文件或程式檔頭。以下 `docker compose` 指令都在對應的 `deploy/<跑法>/` 底下執行。

### 跑法

| 情境 | 位置 | 詳細說明 |
|---|---|---|
| 實機：ROS 全部跑在筆電，RPi 只當轉接（**現行做法**） | [`deploy/laptop/`](deploy/laptop/)、[`deploy/rpi-relay/`](deploy/rpi-relay/) | [docs/laptop-rpi-relay.md](docs/laptop-rpi-relay.md) |
| 模擬：Gazebo + ArduSub SITL | [`deploy/sim/`](deploy/sim/) | [docs/sim/README.md](docs/sim/README.md) |

兩種跑法的 bridge 都綁 UDP 14551，不能同時跑。

**實機**

```bash
# RPi 端（只要做一次，在 deploy/rpi-relay/ 執行；RPi 的 ssh 名稱預設 rpi，可用 RPI_HOST 改）
./deploy.sh                    # 安裝 cam-relay 並設成開機自動跑（取代 realsense-web）
./deploy.sh revert             # 改回 realsense-web
scp mavproxy-udpin.sh rpi: && ssh -t rpi sudo bash mavproxy-udpin.sh   # mavproxy 加 udpin 14560

# 筆電端（在 deploy/laptop/ 執行）
../../ml/rfdetr/fetch_weights.sh            # 第一次：下載偵測模型權重
docker compose up -d                        # cam_bridge、detector、bridge、recorder（錄 rosbag）
docker compose up -d cam_bridge detector    # 只跑影像與偵測（飛控沒接時）
docker compose logs -f detector             # 每 5 秒印 fps 與延遲
docker compose exec bridge bash -ic 'ros2 topic echo --once /mavros/state'   # 上岸檢查飛控，要看到 connected: true
docker compose down                         # 全部關掉
```

**模擬**

```bash
# 在 deploy/sim/ 執行
./run.sh                       # build（第一次）並啟動 sim、bridge，前景執行，Ctrl-C 結束
GUI=0 ./run.sh                 # 不開 Gazebo 視窗
WORLD=<檔名> ./run.sh          # 換 sim/worlds/ 底下的世界檔
./gate_pass.sh                 # 過門：有門、相機、偵測的世界（orca_qgate.sdf），見 docs/sim/README.md
```

QGroundControl 開著就會自動連上（UDP 14550），joystick 要關掉，否則跟 mavros 的搖桿指令打架。

### 功能

以下在實機與模擬都能用，除非特別註明。同一時間只能有一個程式發 `/auv_control`（鍵盤、測試腳本、過門擇一）。

| 功能 | 指令 | 說明 |
|---|---|---|
| 過門（自主） | `./gate_pass.sh` | 起服務、等飛控／相機／偵測就緒、解鎖前等你按 Enter（模擬不等），然後下潛 → 對準 → 前進 → 盲走過門 → 上鎖。可加參數，例如 `./gate_pass.sh --depth 0.3 --surge 0.2 --pass-s 5`，全部參數見 [`mission/gate_pass.py`](mission/gate_pass.py) 的 `Params` |
| 水池測試：定深 | `docker compose exec bridge bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'` | 預設 0.15 → 0.20 → 0.10 m。要改就把深度接在後面，例如 `water_depth.py 0.1 0.2` |
| 水池測試：定向 | 同上，檔名換成 `water_heading.py` | 定深 0.15 m，預設 90 → 0 → −90 → 0°。要改就把角度接在後面，例如 `water_heading.py 45 0` |
| 水池測試：小任務 | 同上，檔名換成 `water_mission.py` | 下潛 → 直走 → 轉 180° → 走回來 |
| 鍵盤操控 | `docker compose run --rm tools python3 /orca/tools/teleop/keyboard.py` | pygame 視窗，按鍵與安全機制見 [`keyboard.py`](tools/teleop/keyboard.py) 檔頭。可加 `--gain 0.3`（初始推力）、`--max-depth 0.25`（定深上限） |
| 看相機畫面（只限實機） | `docker compose run --rm tools ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw` | 原始影像。topic 換成 `/detections/image` 是畫好偵測框的畫面，視窗左上的下拉選單也能切換 |
| 過門流程測試（不需要 ROS） | `python3 mission/test_gate_pass.py` | 用簡化模型檢查流程邏輯與正負號，CI 也會跑 |

- 開視窗的功能（鍵盤操控、看畫面）要先在主機跑 `xhost +SI:localuser:root`；模擬的 `./run.sh` 已經跑過。
- 測試腳本和過門都會先列檢查清單、歸零（以浮在水面時的深度與航向為 0）。任何時候 Ctrl-C 都會上鎖，船浮上來。
- 紀錄 CSV 存在 `deploy/<跑法>/logs/bridge/`。
- 實機跑 `docker compose up -d` 或 `./gate_pass.sh` 時，`recorder` 會把所有 topic（mavros、`/auv_control`、`/detections`、相機 JPEG、`/rosout` 的節點 log）錄成 rosbag，存在 `deploy/laptop/logs/bag/<啟動時間>/`，每 5 分鐘切一個 `.mcap` 檔，約 3 GB／小時。原始影像與 `/detections/image` 不錄。用 Foxglove 直接開 `.mcap`，或 `docker compose run --rm tools ros2 bag play /bags/<資料夾>` 回放（先 `docker compose down`）。

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
| `src/orca_perception/` | 物件偵測節點（RF-DETR） |
| `sim/` | Gazebo 世界與模型、SITL 啟動腳本 |
| `firmware/ardusub/` | 飛控參數：實機快照、SITL |
| `ml/` | 模型訓練的 notebook、設定、下載權重的腳本。權重放在 GitHub Releases，資料集不放進 repo |
| `docker/` | 各 image 的 Dockerfile |
| `deploy/` | 每種跑法一個資料夾：compose 與機器上的服務設定 |
| `tools/` | 水池測試腳本、鍵盤操控（`teleop/keyboard.py`）等不屬於 ROS package 的工具 |
| `mission/` | 自主流程：偵測結果 → `/auv_control`。目前有過門（`gate_pass.py`），在 `bridge` 容器裡執行 |
| `docs/` | 架構、操作、排錯紀錄 |

之後會加的 package（用到時再建）：`orca_mission`（行為樹或 FSM 決策）、`orca_safety`（漏水、電池、kill switch、節點看門狗）、`orca_localization`、`orca_actuators`（電磁鐵、投放）、`orca_description`（URDF、TF）、`orca_bringup`（同一台機器上多個節點的 launch）。

## 開發

- `src/` 會掛進容器，而且 workspace 用 `--symlink-install` build，改 `.py` 之後重開該服務就生效。改了 `.msg`、`package.xml`、`setup.py` 或 Dockerfile 才要 `docker compose build`。
- 同一台電腦上的 ROS 節點都用 `ROS_LOCALHOST_ONLY=1`。要跟別的 ROS 系統隔開（例如同時有人在跑），設不同的 `ROS_DOMAIN_ID`。
- 實機的 bridge 與模擬的 bridge 都綁 UDP 14551，不能同時跑。

## 授權

[Apache-2.0](LICENSE)
