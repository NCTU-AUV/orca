# Orca 模擬：ArduSub SITL + Gazebo

在電腦上跑一套跟實機相同的控制鏈，讓實機的測試程式（`water_depth.py`、`water_heading.py` 等）不必修改就能對模擬執行：

```
Gazebo（泳池 + Orca）⇄ ArduSub 4.7.2（SITL）→ mavproxy → mavros → auv_adapter.py → /auv_control
```

飛控韌體、飛控參數、mavros 設定、adapter 程式都與實機相同；船的物理（推進器、浮力、阻力）由 Gazebo 模擬。設計與規格見 [plan.md](plan.md)，開發中遇到的特別事項見 [notes.md](notes.md)。

## 快速開始

指令都在 `deploy/sim/` 底下執行（`cd deploy/sim`）。第一次會 build image、下載偵測模型權重，要等比較久；主機需求見[需求](#需求)。

**跑任務（行為樹）**：`./mission.sh` 一個指令從起模擬、等就緒、錄 rosbag 到跑完行為樹都包了，不用先跑 `./run.sh`。

```bash
./mission.sh                                         # 過門：資格賽門（orca_qgate.sdf），開 Gazebo 視窗
GUI=0 ./mission.sh                                   # 不開 Gazebo 視窗
./mission.sh gate_pass -p depth=0.5 -p pass_s=6      # -p 覆寫樹的 vars（depth、surge、pass_s、min_score、center_tol、ready_timeout）
WORLD=orca_gate.sdf ./mission.sh gate_pass -p depth=0.9   # 導航門：橫桿在水下 0.6 m，要潛深一點
WORLD=orca_pool.sdf ./mission.sh smoke_test          # 動作測試（下潛、轉向、前進），不需要門和相機
RECORD=0 ./mission.sh                                # 不錄 rosbag
./gate_pass.sh                                       # 舊版過門（狀態機 mission/gate_pass.py）
```

- 每次執行都會重開 `simulator`，船回到水面、原點；其他服務沿用已經在跑的。要重跑就再執行一次。
- 任何時候 Ctrl-C：上鎖、船浮上來。rosbag 從樹開始錄，樹結束、船浮上水面後自動停止並收尾（最多等 30 秒，再按一次 Ctrl-C 不等）。
- 紀錄：`logs/mission/<樹>_<時間>.csv`、`logs/bag/<樹>_<時間>/`。bag 在 mission 結束後不到 1 秒就收尾好，接著就能用 Foxglove 開裡面的 `.mcap`。
- 細節見[過門](#過門有門相機偵測)；樹的寫法與參數見 [`src/orca_mission`](../../src/orca_mission/README.md)。

**只起模擬（跑水池測試程式、鍵盤操控）**：

```bash
./run.sh                       # simulator + fc_adapter，前景執行，Ctrl-C 結束
GUI=0 ./run.sh                 # 不開 Gazebo 視窗
WORLD=orca_qgate.sdf ./run.sh  # 換 sim/worlds/ 底下的世界檔（預設 orca_pool.sdf）
```

起來約 5 秒後 EKF 對準，就可以另開終端機（同樣在 `deploy/sim/`）：

```bash
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'   # 定深測試，模擬裡檢查清單直接按 Enter
docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py                  # 鍵盤操控
```

同一時間只能有一個程式發 `/auv_control`（測試程式、鍵盤、行為樹擇一）。

**跑的時候看狀態**（另開終端機，在 `deploy/sim/`）：

```bash
docker compose --profile mission run --rm mission py-trees-tree-watcher   # 即時看整棵樹每個節點的狀態
docker compose logs -f fc_adapter         # mavros、auv_adapter
docker compose logs -f object_detector    # 偵測，每 5 秒印 fps 與推論時間
```

主機上開 QGroundControl 會自動連上（UDP 14550），joystick 要關掉。

**全部關掉**：

```bash
docker compose --profile vision down
```

## 需求

| 項目 | 說明 |
|---|---|
| 系統 | Linux（在 Ubuntu 24.04 上驗證過），桌面已登入（GUI 要 X 或 XWayland） |
| 顯示卡 | NVIDIA，驅動已裝好（`nvidia-smi` 能用）。`simulator` 服務設了 `gpus: all`，不開 GUI 也要有 |
| Docker | Docker Engine + compose v2，目前使用者在 `docker` 群組 |
| NVIDIA Container Toolkit | 讓容器用 GPU。`docker info` 的 Runtimes 要有 `nvidia`，`./run.sh` 會檢查 |
| 硬碟 | 兩個 image 合計約 5 GB（`orca-sim` 3.6 GB、`orca-base` 2.7 GB，其中 1.2 GB 是兩者共用的 `ros:humble` 基底） |
| 網路 | 第一次 build 要下載 Ubuntu、ROS、Gazebo 套件並編譯 ArduSub。12 核的筆電上約 8 分鐘（不含下載） |

主機上不需要裝 ROS、Gazebo 或 ArduPilot。QGroundControl 是選用的。

## 怎麼跑

以下指令都在 `deploy/sim/` 底下執行。

```bash
./run.sh            # build（第一次）並啟動，前景執行，Ctrl-C 結束
GUI=0 ./run.sh      # 不開 Gazebo 視窗
```

`./run.sh` 會依序：允許容器連到 X（`xhost +SI:localuser:root`）、確認有 nvidia runtime、`docker compose up --build`。起來的東西：

| 服務 | 內容 |
|---|---|
| `simulator` | Gazebo 泳池與 Orca、ArduSub SITL、mavproxy（MAVLink 轉送到 UDP 14550 給 QGC、UDP 14551 給 mavros） |
| `fc_adapter` | mavros 與 `auv_adapter.py`，跟實機用同一個 image，只差 mavros 設定（`mavros_sitl.yaml`） |

開機後約 5 秒 EKF 對準完成（`docker compose logs fc_adapter` 出現 `MAG0 initial yaw alignment complete`），之後才能解鎖。

背景執行：`docker compose up -d`（`GUI=1` 時要先自己跑 `xhost +SI:localuser:root`），停止：`docker compose down`。

### 跑實機的測試程式

另開終端機進 fc_adapter 容器：

```bash
docker compose exec fc_adapter bash
python3 /orca/tools/pool_tests/water_depth.py     # 定深 0.15 → 0.20 → 0.10 m
python3 /orca/tools/pool_tests/water_heading.py   # 定深 0.15 m，航向 +90 → 0 → −90 → 0°
```

- 程式開頭的檢查清單（繩子、kill switch 等）是給實機的，模擬裡直接按 Enter。auv_adapter 已經在容器裡跑著，不用另開。
- 程式會歸零（以船浮在水面時的深度、航向為 0），所以每次都在船浮著、上鎖時開始。上一個測試上鎖後，等幾秒讓船浮回水面。
- 同一時間只跑一支測試程式。兩支同時跑會互相切模式、解鎖、上鎖。
- 測試程式的深度上限 0.25 m 是實機測試池的深度（`water_common.py` 的 `MAX_DEPTH`）；模擬的泳池深 1.6 m。
- 結果 CSV 在 `deploy/sim/logs/fc_adapter/`。mavros 與 auv_adapter 的輸出用 `docker compose logs -f fc_adapter` 看。
- `/auv_control` 的用法與各欄位意思見 [`AuvControl.msg`](../../src/orca_msgs/msg/AuvControl.msg) 的註解。

### 鍵盤操控

```bash
docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py   # --gain 0.3、--max-depth 0.25
```

開一個 pygame 視窗，視窗有焦點時：W/S、A/D、R/F、Q/E 按住出力（放開就停），1/2/3 切模式，Enter 解鎖、空白鍵上鎖，Z 歸零（上鎖時），H 定深開關（開啟時 R/F 改成移動目標深度），`-`/`=` 調推力大小。視窗失去焦點時全部歸零；關閉視窗會上鎖。完整說明在 [`keyboard.py`](../../tools/teleop/keyboard.py) 的檔頭。不要跟測試程式同時跑。

`./run.sh` 已經跑過 `xhost +SI:localuser:root`；`GUI=0` 時要自己跑一次，視窗才開得起來。

### 過門（有門、相機、偵測）

行為樹版（[`src/orca_mission`](../../src/orca_mission/README.md)，流程寫在 `trees/gate_pass.yaml`）：

```bash
./mission.sh                                      # 資格賽門，開 Gazebo 視窗
GUI=0 ./mission.sh gate_pass -p pass_s=6          # 不開視窗；-p 覆寫樹的 vars
WORLD=orca_gate.sdf ./mission.sh gate_pass -p depth=0.9   # 導航門
WORLD=orca_pool.sdf ./mission.sh smoke_test       # 動作測試（下潛、轉向、前進），不需要門
```

一個指令：沒有權重就先下載 → build image → 起 `fc_adapter`、`sim_cameras`、`object_detector`（`vision` profile）→ 重開 `simulator`（船回到水面、原點）→ 等 EKF 對準、飛控連線、偵測都就緒 → 開始錄 rosbag → 用 `mission` 服務跑行為樹（`--no-checklist`）→ 樹結束（含 Ctrl-C）、上鎖、船浮上水面後停止錄影。

每次執行的紀錄同名，`<樹>_<時間>`：

- `deploy/sim/logs/mission/<樹>_<時間>.csv`：行為樹的紀錄（20 Hz）。
- `deploy/sim/logs/bag/<樹>_<時間>/`：rosbag（MCAP），用 [Foxglove](https://foxglove.dev) 直接開裡面的 `.mcap`。錄了 mavros、`/auv_control`、`/detections`、`/mission/status`（目前的節點與回饋）、`/rosout` 等全部 topic；**影像只錄畫好框的 JPEG**（`/detections/image/compressed`、`/detections/bottom_image/compressed`），相機的原始影像不錄。一次過門（約 30 秒）約 12 MB。
- 不想錄：`RECORD=0 ./mission.sh`。錄影設定在 `compose.yaml` 的 `recorder` 服務。
- Foxglove 的 layout：[`tools/foxglove/orca_mission.json`](../../tools/foxglove/orca_mission.json)（Foxglove 右上角的 layout 選單 → Import from file）。分頁「過門」有前視、底部畫好框的影像，行為樹節點／飛控模式／解鎖／有沒有看到門，門的偏離與框寬、深度、推力；分頁「飛控、log」有航向、`/mavros/state`、`/rosout`。`/mission/status` 是 JSON 字串，layout 裡帶的 User Script（`tools/foxglove/*.ts`）會把它和 `/detections` 拆成欄位。改了之後用 `python3 tools/foxglove/make_layout.py` 重新產生。

舊版（狀態機 [`mission/gate_pass.py`](../../mission/gate_pass.py)，參數直接交給它）：

```bash
./gate_pass.sh                              # 資格賽門
GUI=0 ./gate_pass.sh --pass-s 6
WORLD=orca_gate.sdf ./gate_pass.sh --depth 0.9
```

流程一樣：起 `fc_adapter`、`sim_cameras`、`object_detector` → 重開 `simulator` → 等就緒 → 在 `fc_adapter` 容器裡執行 `mission/gate_pass.py`。模擬裡不列實機的檢查清單，不用按 Enter。跑完容器繼續跑，再執行一次就是重跑；全部關掉用 `docker compose --profile vision down`。

| 世界 | 內容 |
|---|---|
| `orca_qgate.sdf`（預設） | 資格賽門（`models/q_gate`），從水面垂到池底，任何深度都能過；在船頭前方 6 m、偏右 1 m |
| `orca_gate.sdf` | 導航門（`models/gate`），立在池底、高 1 m，橫桿在水下 1.6 − 1.0 = 0.6 m：船要潛到約 0.9 m 才不會從橫桿上方經過 |

- 兩個世界的船都是 `models/orca_cam`：`models/orca` 原封不動併入，多一顆前視相機（640×480、15 Hz、水平視角 55.7°，跟實機 D435i 彩色的 `camera_info` 一致）和一顆朝下的底部相機（640×480、10 Hz、水平視角 70°，影像上方 = 船頭；實機規格還沒定）。
- 門的模型取自 [SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation) 依 2026 規則書做的版本。
- `sim_cameras` 服務（[`sim/gz_camera_bridge.py`](../../sim/gz_camera_bridge.py)）把 Gazebo 影像轉成跟實機一樣的 `/camera/camera/color/image_raw`、`camera_info`，底部是 `/bottom_camera/image_raw`、`camera_info`；`object_detector` 跟實機是同一個 RF-DETR 節點，兩台相機輪流推論。不需要 `ros_gz`。
- 2026-11-12 實測（`orca_qgate.sdf`、`GUI=0`）：相機約 11～14 fps；RF-DETR 把資格賽門認成 `gate`（分數 0.6～0.95，流程兩類都接受）；連跑兩次都走完，Gazebo 真值顯示通過門平面時離門中心 1.3 cm。
- **畫面比真實水下乾淨**：沒有水的衰減、色偏、浮游物（Harmonic 的 ogre2 不支援 fog），偵測會比實機容易。適合驗證流程與方向，不代表實機的偵測距離。
- 模擬裡 surge 0.2 約 0.8 m/s，盲走 8 秒會衝過門約 6 m，離池壁不到 2 m（現在 `gate_pass.py` 的預設是 5 秒）。

### QGroundControl（選用）

在主機上開 QGC 就會自動連上（UDP 14550），可以看姿態、模式、參數。**QGC 的 joystick 要關掉**：QGC 與 mavros 的 system id 都是 255，兩邊的搖桿指令會打架。

### 改設定

| 改了什麼 | 怎麼生效 |
|---|---|
| repo 的 `sim/` 底下的世界、模型、`start.sh`，或 `firmware/ardusub/sitl.parm` | `docker compose up -d --force-recreate simulator`（直接掛進容器，不用重 build） |
| `tools/pool_tests/*.py` | 重新執行該程式 |
| `src/` 底下的 `.py`、mavros 設定 | `docker compose restart fc_adapter`（`src/` 掛進容器，不用重 build） |
| Dockerfile、`orca_msgs`、`package.xml`、`setup.py` | `docker compose build` 後再啟動 |

`compose.yaml` 可用的環境變數：`GUI`（1 / 0）、`WORLD`（`sim/worlds/` 底下的世界檔，預設 `orca_pool.sdf`）、`DISPLAY`。

飛控每次開機都從 `sitl.parm` 重新載入參數（不保留上次改的值）。執行中用 QGC 或 mavros 改的參數，重開 sim 就會消失；要保留就寫進 `firmware/ardusub/sitl.parm`。

## Log

| 位置 | 內容 |
|---|---|
| `deploy/sim/logs/sim/ardusub/ardusub.out` | ArduSub 的終端機輸出 |
| `deploy/sim/logs/sim/ardusub/logs/*.BIN` | 飛控的 DataFlash log（高頻姿態、控制器目標與出力），用 pymavlink 或 MAVExplorer 讀 |
| `deploy/sim/logs/sim/mavproxy/` | mavproxy 的 tlog |
| `deploy/sim/logs/fc_adapter/*.csv` | 測試程式、舊版過門的紀錄 |
| `deploy/sim/logs/mission/*.csv` | 行為樹的紀錄 |
| `deploy/sim/logs/bag/<樹>_<時間>/` | `./mission.sh` 每次執行錄的 rosbag（MCAP，影像只有畫好框的 JPEG），跟 CSV 同名 |

## 已知問題與限制

**模擬與實機的差異**

- 船的質量、浮力、阻力、推進器位置沿用 [NCTU-AUV/SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation) 的估計值，而且模型沒有推進器的反應延遲。結果是模擬比實機「好控」：定深、轉向的過衝比實機小很多（轉 90° 過衝約 1.4°，實機 9～13°），定深時抵銷浮力的垂直出力約 −13 µs（實機約 −50 µs）。**模擬適合驗證流程、方向與邏輯，不適合拿來調 PID。**
- 模擬推進器的 PWM → 推力是線性的，但 `sitl.parm` 沿用實機的 `MOT_THST_EXPO` 0.65（用來補償 T200 的非線性），所以小指令時模擬的推力比實機大。
- 船的 yaw 阻尼很小：MANUAL 下 r = 300 約 170°/s。受控模式（STABILIZE、DEPTH_HOLD）有 `ATC_RATE_WPY_MAX` 60°/s 限住，跟實機一樣。
- 飛控讀到的深度比 Gazebo 的真實深度多約 2.5%：SITL 用海水密度算水壓，飛控用淡水換回深度。
- 飛控回報的深度以約 1 cm 為單位跳動。
- 船頭朝 Gazebo 的 +x（東）時，飛控航向約 77°，不是 90°：沒有 GPS 就沒有 EKF 原點，飛控不做磁偏角補償，報的是磁北航向（實機也一樣）。測試程式都用相對歸零時的航向，不受影響。

**看起來像錯誤、其實正常的訊息**

| 訊息 | 原因 |
|---|---|
| `Compass performance degraded`（解鎖時） | 沒有 GPS → 沒有 EKF 原點 → 不能查磁偏角表。實機同樣沒有 GPS |
| `GPS 1: detected u-blox` | 模擬的 GPS 設成永遠不定位（`SIM_GPS1_ENABLE 0`），但仍會被偵測到；EKF 一樣沒有水平位置 |
| `GP: No GPS fix`、`CMD: Unexpected command 410, result 4`（mavros） | 沒有 GPS，mavros 要不到 home 位置 |
| `Lost manual control`（只跑 sim、沒跑 adapter 時） | 沒有程式在送搖桿；adapter 起來後就不會出現 |

**操作上**

- `logs/` 底下的檔案由容器以 root 寫入，主機上刪除要 `sudo rm -rf logs/*`，或借容器刪：`docker run --rm -v "$PWD/logs:/l" orca-sim bash -c 'rm -rf /l/*'`（在 `deploy/sim/` 底下）。
- 兩個服務都用 host network，佔用本機的 UDP 9002、14550、14551 與 TCP 5760～5763。同一台電腦不能同時跑另一套 ArduPilot SITL 或 mavros。
- fc_adapter 在跑的時候，不要讓其他程式綁 UDP 14551。**筆電上的實機 fc_adapter（`deploy/laptop`）也綁 14551，兩者不能同時跑**，否則 SITL 的 MAVLink 會送進接著實機的 mavros：同一個埠只會有一個程式收到封包，mavros 會斷線。要直接讀 MAVLink，就在 QGC 沒開的時候聽 14550。
- ROS 2 Humble 的官方搭配是 Gazebo Fortress；這裡用 Harmonic。之後要接 ROS 橋接時用 `ros-humble-ros-gzharmonic`。

## 檔案

| 路徑（repo 根目錄起算） | 內容 |
|---|---|
| `deploy/sim/run.sh`、`deploy/sim/compose.yaml` | 啟動 |
| `docker/sim.Dockerfile` | ROS 2 Humble + Gazebo Harmonic + ArduSub Sub-4.7.2 SITL + [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) 外掛 + MAVProxy |
| `sim/start.sh` | 依序啟動 gz sim、ardusub、mavproxy |
| `firmware/ardusub/sitl.parm` | 飛控參數（取自實機）與 SITL 模擬硬體參數 |
| `sim/worlds/orca_pool.sdf` | 泳池世界（25 × 16 m、深 1.6 m，水面 z = 0） |
| `sim/worlds/orca_qgate.sdf`、`orca_gate.sdf` | 泳池 + 有相機的 Orca + 資格賽門／導航門 |
| `sim/models/pool/`、`sim/models/orca/` | 泳池、Orca（船體、浮力、阻力、8 顆推進器、IMU、ArduPilotPlugin） |
| `sim/models/orca_cam/`、`gate/`、`q_gate/` | Orca + 前視相機、導航門、資格賽門 |
| `sim/gz_camera_bridge.py` | Gazebo 前視、底部相機 → ROS 影像（`sim_cameras` 服務） |
| `deploy/sim/mission.sh` | 一個指令跑行為樹（預設過門） |
| `tools/foxglove/` | 看 rosbag 用的 Foxglove layout（`orca_mission.json`，由 `make_layout.py` 產生） |
| `deploy/sim/gate_pass.sh` | 一個指令跑舊版過門 |
| `src/orca_bridge/` | mavros + `auv_adapter.py`，跟實機共用 |
| `tools/pool_tests/` | 水池測試腳本 |

## 之後要加東西

- 比賽道具：另寫一個世界檔 `<include>` 泳池與 `orca_cam`（要相機）或 `orca`，用 `WORLD=<檔名>` 載入。過門的兩個世界檔可以當範本。
- 水下成像（衰減、色偏）：SAUVC-Simulation 的 `underwater_camera_node.py` 用深度圖做，可以搬到 `gz_camera_bridge.py`。
- ROS 橋接、視覺、任務層：在 `deploy/sim/compose.yaml` 新增服務（ROS 2 Humble、host network、`ROS_LOCALHOST_ONLY=1`），不改 `simulator`、`fc_adapter` 的埠與 topic。
