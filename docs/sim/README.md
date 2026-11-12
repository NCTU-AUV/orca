# Orca 模擬：ArduSub SITL + Gazebo

在電腦上跑一套跟實機相同的控制鏈，讓實機的測試程式（`water_depth.py`、`water_heading.py` 等）不必修改就能對模擬執行：

```
Gazebo（泳池 + Orca）⇄ ArduSub 4.7.2（SITL）→ mavproxy → mavros → auv_adapter.py → /auv_control
```

飛控韌體、飛控參數、mavros 設定、adapter 程式都與實機相同；船的物理（推進器、浮力、阻力）由 Gazebo 模擬。設計與規格見 [plan.md](plan.md)，開發中遇到的特別事項見 [notes.md](notes.md)。

## 需求

| 項目 | 說明 |
|---|---|
| 系統 | Linux（在 Ubuntu 24.04 上驗證過），桌面已登入（GUI 要 X 或 XWayland） |
| 顯示卡 | NVIDIA，驅動已裝好（`nvidia-smi` 能用）。`sim` 服務設了 `gpus: all`，不開 GUI 也要有 |
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
| `sim` | Gazebo 泳池與 Orca、ArduSub SITL、mavproxy（MAVLink 轉送到 UDP 14550 給 QGC、UDP 14551 給 mavros） |
| `bridge` | mavros 與 `auv_adapter.py`，跟實機用同一個 image，只差 mavros 設定（`mavros_sitl.yaml`） |

開機後約 5 秒 EKF 對準完成（`docker compose logs bridge` 出現 `MAG0 initial yaw alignment complete`），之後才能解鎖。

背景執行：`docker compose up -d`（`GUI=1` 時要先自己跑 `xhost +SI:localuser:root`），停止：`docker compose down`。

### 跑實機的測試程式

另開終端機進 bridge 容器：

```bash
docker compose exec bridge bash
python3 /orca/tools/pool_tests/water_depth.py     # 定深 0.15 → 0.20 → 0.10 m
python3 /orca/tools/pool_tests/water_heading.py   # 定深 0.15 m，航向 +90 → 0 → −90 → 0°
```

- 程式開頭的檢查清單（繩子、kill switch 等）是給實機的，模擬裡直接按 Enter。auv_adapter 已經在容器裡跑著，不用另開。
- 程式會歸零（以船浮在水面時的深度、航向為 0），所以每次都在船浮著、上鎖時開始。上一個測試上鎖後，等幾秒讓船浮回水面。
- 同一時間只跑一支測試程式。兩支同時跑會互相切模式、解鎖、上鎖。
- 測試程式的深度上限 0.25 m 是實機測試池的深度（`water_common.py` 的 `MAX_DEPTH`）；模擬的泳池深 1.6 m。
- 結果 CSV 在 `deploy/sim/logs/bridge/`。mavros 與 auv_adapter 的輸出用 `docker compose logs -f bridge` 看。
- `/auv_control` 的用法與各欄位意思見 [`AuvControl.msg`](../../src/orca_msgs/msg/AuvControl.msg) 的註解。

### 鍵盤操控

```bash
docker compose run --rm tools python3 /orca/tools/teleop/keyboard.py   # --gain 0.3、--max-depth 0.25
```

開一個 pygame 視窗，視窗有焦點時：W/S、A/D、R/F、Q/E 按住出力（放開就停），1/2/3 切模式，Enter 解鎖、空白鍵上鎖，Z 歸零（上鎖時），H 定深開關（開啟時 R/F 改成移動目標深度），`-`/`=` 調推力大小。視窗失去焦點時全部歸零；關閉視窗會上鎖。完整說明在 [`keyboard.py`](../../tools/teleop/keyboard.py) 的檔頭。不要跟測試程式同時跑。

`./run.sh` 已經跑過 `xhost +SI:localuser:root`；`GUI=0` 時要自己跑一次，視窗才開得起來。

### QGroundControl（選用）

在主機上開 QGC 就會自動連上（UDP 14550），可以看姿態、模式、參數。**QGC 的 joystick 要關掉**：QGC 與 mavros 的 system id 都是 255，兩邊的搖桿指令會打架。

### 改設定

| 改了什麼 | 怎麼生效 |
|---|---|
| repo 的 `sim/` 底下的世界、模型、`start.sh`，或 `firmware/ardusub/sitl.parm` | `docker compose up -d --force-recreate sim`（直接掛進容器，不用重 build） |
| `tools/pool_tests/*.py` | 重新執行該程式 |
| `src/` 底下的 `.py`、mavros 設定 | `docker compose restart bridge`（`src/` 掛進容器，不用重 build） |
| Dockerfile、`orca_msgs`、`package.xml`、`setup.py` | `docker compose build` 後再啟動 |

`compose.yaml` 可用的環境變數：`GUI`（1 / 0）、`WORLD`（`sim/worlds/` 底下的世界檔，預設 `orca_pool.sdf`）、`DISPLAY`。

飛控每次開機都從 `sitl.parm` 重新載入參數（不保留上次改的值）。執行中用 QGC 或 mavros 改的參數，重開 sim 就會消失；要保留就寫進 `firmware/ardusub/sitl.parm`。

## Log

| 位置 | 內容 |
|---|---|
| `deploy/sim/logs/sim/ardusub/ardusub.out` | ArduSub 的終端機輸出 |
| `deploy/sim/logs/sim/ardusub/logs/*.BIN` | 飛控的 DataFlash log（高頻姿態、控制器目標與出力），用 pymavlink 或 MAVExplorer 讀 |
| `deploy/sim/logs/sim/mavproxy/` | mavproxy 的 tlog |
| `deploy/sim/logs/bridge/*.csv` | 測試程式的紀錄 |

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
- bridge 在跑的時候，不要讓其他程式綁 UDP 14551。**筆電上的實機 bridge（`deploy/laptop`）也綁 14551，兩者不能同時跑**，否則 SITL 的 MAVLink 會送進接著實機的 mavros：同一個埠只會有一個程式收到封包，mavros 會斷線。要直接讀 MAVLink，就在 QGC 沒開的時候聽 14550。
- ROS 2 Humble 的官方搭配是 Gazebo Fortress；這裡用 Harmonic。之後要接 ROS 橋接時用 `ros-humble-ros-gzharmonic`。

## 檔案

| 路徑（repo 根目錄起算） | 內容 |
|---|---|
| `deploy/sim/run.sh`、`deploy/sim/compose.yaml` | 啟動 |
| `docker/sim.Dockerfile` | ROS 2 Humble + Gazebo Harmonic + ArduSub Sub-4.7.2 SITL + [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) 外掛 + MAVProxy |
| `sim/start.sh` | 依序啟動 gz sim、ardusub、mavproxy |
| `firmware/ardusub/sitl.parm` | 飛控參數（取自實機）與 SITL 模擬硬體參數 |
| `sim/worlds/orca_pool.sdf` | 泳池世界（25 × 16 m、深 1.6 m，水面 z = 0） |
| `sim/models/pool/`、`sim/models/orca/` | 泳池、Orca（船體、浮力、阻力、8 顆推進器、IMU、ArduPilotPlugin） |
| `src/orca_bridge/` | mavros + `auv_adapter.py`，跟實機共用 |
| `tools/pool_tests/` | 水池測試腳本 |

## 之後要加東西

- 比賽道具、相機：另寫一個世界檔 `<include>` 泳池與 Orca，用 `WORLD=<檔名> ./run.sh` 載入；相機做成獨立的附掛模型，不改 `models/orca`。
- ROS 橋接、視覺、任務層：在 `deploy/sim/compose.yaml` 新增服務（ROS 2 Humble、host network、`ROS_LOCALHOST_ONLY=1`），不改 `sim`、`bridge` 的埠與 topic。
