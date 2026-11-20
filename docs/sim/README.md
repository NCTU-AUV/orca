# Orca 模擬：ArduSub SITL + Gazebo

在電腦上跑一套跟實機相同的控制鏈，讓實機的測試程式（`water_depth.py`、`water_heading.py` 等）不必修改就能對模擬執行：

```
Gazebo（泳池 + Orca）⇄ ArduSub 4.7.2（SITL）→ mavproxy → mavros → auv_adapter.py → /auv_control
```

飛控韌體、飛控參數、mavros 設定、adapter 程式都與實機相同；船的物理（推進器、浮力、阻力）由 Gazebo 模擬。開發中遇到的坑與原因見 [notes.md](notes.md)，最初的設計與規格見 [archive/sim-plan.md](../archive/sim-plan.md)。

場地照 SAUVC 2027 規則（Rulebook 2027.3.2）：資格賽與決賽主場地兩個世界、水下的畫面、碰撞與計分事件、偵測驗證，見 [world-2027.md](world-2027.md)。

日常的指令（跑任務、只起模擬、鍵盤、看畫面、全部關掉）在 [README 的「模擬」](../../README.md#模擬)。這份寫其他的指令、各部分怎麼運作，以及排錯。以下指令都在 `deploy/sim/` 底下執行。

## 需求

| 項目 | 說明 |
|---|---|
| 系統 | Linux（在 Ubuntu 24.04 上驗證過），桌面已登入（GUI 要 X 或 XWayland） |
| 顯示卡 | NVIDIA，驅動已裝好（`nvidia-smi` 能用）。`simulator` 服務設了 `gpus: all`，不開 GUI 也要有 |
| Docker | Docker Engine + compose v2，目前使用者在 `docker` 群組 |
| NVIDIA Container Toolkit | 讓容器用 GPU。`docker info` 的 Runtimes 要有 `nvidia`（或裝了 `nvidia-container-runtime-hook`），`./run.sh` 會檢查 |
| 硬碟 | `orca-sim`、`orca-base` 合計約 5 GB（3.8 GB、2.7 GB，其中 1.2 GB 是共用的 `ros:humble` 基底）；有相機的世界另外要 `orca-perception`（約 12 GB）、`orca-mission`（2.7 GB，大部分跟 `orca-base` 共用） |
| 網路 | 第一次 build 要下載 Ubuntu、ROS、Gazebo 套件並編譯 ArduSub。12 核的筆電上約 8 分鐘（不含下載） |

主機上不需要裝 ROS、Gazebo 或 ArduPilot。QGroundControl 是選用的。第一次跑會 build image、下載偵測模型權重，要等比較久。

## 世界

`sim/worlds/` 底下的世界檔，用環境變數 `WORLD=<檔名>` 選。**各腳本的預設不一樣**：`run.sh` 是 `orca_pool.sdf`，`mission.sh` 是 `sauvc_qual.sdf`。

| 世界 | 內容 | 船的出發位置 | 相機、偵測 | 計分 |
|---|---|---|---|---|
| `sauvc_qual.sdf` | SAUVC 2027 資格賽：50 × 25 m 泳池、三條資格賽道。約 9.6 m 外是自水面吊掛 1 m 的資格賽門（偏右 0.2 m），池底在門那裡約 1.62 m | 第 2 道的起點標記，船尾貼著牆、船頭朝門 | 有 | 資格賽 |
| `sauvc_final.sdf` | SAUVC 2027 決賽主場地：約 10.8 m 外是立在 2 m 池底的導航門（高 1.5 m），還有橘色 flare、紅／黃／藍 flare、綠墊與四個 drum、Surfacing Square，船底掛著一顆球 | 起始區中心、水面，船頭朝導航門 | 有 | 決賽 |
| `orca_pool.sdf` | 舊的空池（25 × 16 m、深 1.6 m），沒有道具，給動作測試用 | 原點 | 沒有 | 沒有（只記下潛、出發這類一般事件） |

- 每個世界的原點都是船的出發位置（水面 z = 0，往下為負，船頭 +x）。道具的擺放、尺寸、計分事件見 [world-2027.md](world-2027.md)。
- 有道具的世界，船都是 `models/orca_cam`：`models/orca` 原封不動併入，多一顆前視相機（640×480、15 Hz、水平視角 55.7°，跟實機 D435i 彩色的 `camera_info` 一致）、一顆朝下的底部相機（640×480、10 Hz、水平視角 70°，影像上方 = 船頭；實機規格還沒定），以及碰撞外框（只用來判斷碰撞，不算浮力）。腳本看世界檔有沒有 `model://orca_cam` 決定要不要起相機和偵測。
- `sim_cameras` 服務（[`sim/gz_camera_bridge.py`](../../sim/gz_camera_bridge.py)）把 Gazebo 影像轉成跟實機一樣的 `/camera/camera/color/image_raw`、`camera_info`，底部是 `/bottom_camera/image_raw`、`camera_info`，轉之前先做水下成像的後處理（[`sim/underwater.py`](../../sim/underwater.py)）；`object_detector` 跟實機是同一個 RF-DETR 節點，兩台相機輪流推論。不需要 `ros_gz`。

## 服務

| 服務 | 內容 | 誰會起 |
|---|---|---|
| `simulator` | Gazebo 泳池與 Orca、ArduSub SITL、mavproxy（MAVLink 轉送到 UDP 14550 給 QGC、UDP 14551 給 mavros） | 全部腳本 |
| `fc_adapter` | mavros 與 `auv_adapter.py`，跟實機用同一個 image，只差 mavros 設定（`mavros_sitl.yaml`） | `run.sh`、`mission.sh` |
| `sim_referee` | 模擬裁判（[`sim/referee.py`](../../sim/referee.py)）：碰撞、過門、掉球、上浮 → `/sim/referee/events`、分數。只有模擬有，任務程式不要訂閱 | 全部腳本 |
| `sim_dropper` | 放球指令 `/ball_dropper/release`（`std_msgs/Empty`，**暫定介面**）→ Gazebo 放開掛在船底的球（只有 `sauvc_final.sdf` 有球） | 全部腳本 |
| `sim_cameras` | Gazebo 前視、底部相機 → ROS 影像，含水下後處理（`vision` profile） | 有相機的世界 |
| `object_detector` | 跟實機同一個 RF-DETR 節點 → `/detections`、畫好框的影像（`vision` profile） | 有相機的世界 |
| `recorder` | rosbag（MCAP）（`record` profile） | `mission.sh` |
| `mission` | 行為樹（`mission` profile），一次性容器，跑完就刪 | `mission.sh` |
| `gui_tools` | 鍵盤操控、看畫面等臨時工具（`tools` profile），用 `docker compose run --rm` 開 | 自己開 |
| `sim_sweep`、`detect_eval` | 偵測驗證的拍照與推論（`sweep` profile） | `detect_sweep.sh` |

## 跑行為樹（`mission.sh`）

```bash
cd deploy/sim
WORLD=orca_pool.sdf ./mission.sh smoke_test    # 動作測試，空池，不需要相機
SEED=7 WORLD=sauvc_final.sdf ./mission.sh      # 決賽主場地的隨機佈局（SEED=random 隨機挑；同一個 SEED 佈局一樣），見 world-2027.md
UNDERWATER=0 ./mission.sh                      # 關掉水下效果（乾淨明亮的畫面）
UNDERWATER_STRENGTH=0.5 ./mission.sh           # 水下效果的強度，預設 1.0；0.5 更清澈、2 更混
RECORD=0 ./mission.sh                          # 不錄 rosbag
docker compose --profile mission run --rm mission mission gate_pass --print-tree   # 只印樹的結構，不執行
```

| 樹 | 內容 | 需要 |
|---|---|---|
| `gate_pass`（預設） | 過門：航向從解鎖起就鎖住（歸零時的船頭方向），對準只用 sway 平移 | 有門、有相機的世界 |
| `gate_pass_yaw` | 過門：下潛後先原地轉 yaw 對準門、鎖住當下的航向，之後跟 `gate_pass` 一樣 | 同上 |
| `smoke_test` | 動作測試：下潛 → 右轉 90° → 往前 → 轉回 0° → 浮上 → 上鎖 | 不需要相機，配 `WORLD=orca_pool.sdf` |

樹寫在 `src/orca_mission/trees/<樹>.yaml`，旁邊的 `<樹>.md` 是流程圖。`-p` 能覆寫的參數就是 YAML 裡 `vars` 的那些（`gate_pass` 有 `depth`、`surge`、`pass_s`、`min_score`、`center_tol`、`ready_timeout` 等）。樹的寫法、節點見 [`src/orca_mission`](../../src/orca_mission/README.md)。

`mission.sh` 一個指令做完這些：沒有權重就先下載 → build image → 起 `fc_adapter`、`sim_dropper`（有相機的世界加 `sim_cameras`、`object_detector`）→ 重開 `simulator` 與 `sim_referee`（船回到水面、出發位置，從 0 分開始）→ 等 EKF 對準、飛控連線、偵測都就緒 → 開始錄 rosbag → 用 `mission` 服務跑行為樹（`--no-checklist`）→ 樹結束（含 Ctrl-C）、上鎖、船浮上水面後停止錄影，印出裁判的事件。

- 每次執行都會重開 `simulator`，其他服務沿用已經在跑的（省掉 mavros、偵測模型重新載入的時間）。要重跑就再執行一次。
- 任何時候 Ctrl-C：上鎖、船浮上來。rosbag 錄到船浮上水面（最多等 30 秒，再按一次 Ctrl-C 不等）。
- 跑完服務繼續跑，全部關掉用 `docker compose --profile vision down`。
- 2026-11-12 實測（舊的 `orca_qgate.sdf`、`GUI=0`）：相機約 11～14 fps；RF-DETR 把資格賽門認成 `gate`（分數 0.6～0.95，流程兩類都接受）；連跑兩次都走完，Gazebo 真值顯示通過門平面時離門中心 1.3 cm。2027 世界的結果見 [world-2027.md](world-2027.md#驗收紀錄)。
- 模擬裡 surge 0.2 約 0.8 m/s，盲走 8 秒會衝過門約 6 m（`gate_pass.yaml` 的 `pass_s` 預設是 7 秒）。

## 只起模擬（`run.sh`）

```bash
cd deploy/sim
./run.sh -d                                          # 背景執行（其他參數都交給 docker compose up）
WORLD=sauvc_final.sdf ./run.sh -d --force-recreate   # 重開，船回到出發位置（要帶同一個 WORLD，不然會換回空池）
SEED=7 WORLD=sauvc_final.sdf ./run.sh                # 決賽主場地的隨機佈局
```

`./run.sh` 會依序：允許容器連到 X（`xhost +SI:localuser:root`）、確認有 nvidia runtime、有相機的世界先確認偵測權重在（沒有就下載）、`docker compose up --build`（有相機的世界加 `--profile vision`）。

開機後約 5 秒 EKF 對準完成（`docker compose logs simulator` 出現 `MAG0 initial yaw alignment complete`），之後才能解鎖。

不經 `run.sh` 直接 `docker compose up -d` 也可以，但 `GUI=1` 時要先自己跑 `xhost +SI:localuser:root`，有相機的世界要加 `--profile vision`。

## 跑實機的測試程式

先 `./run.sh`（空池就夠），另開終端機：

```bash
cd deploy/sim
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'          # 定深 0.15 → 0.20 → 0.10 m
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_depth.py 0.1 0.2'  # 自訂深度
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_heading.py'        # 定深 0.15 m，航向 +90 → 0 → −90 → 0°
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_heading.py 45 0'   # 自訂角度
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_mission.py'        # 下潛 → 直走 → 轉 180° → 走回來
```

- 程式開頭的檢查清單（繩子、kill switch 等）是給實機的，模擬裡直接按 Enter。auv_adapter 已經在容器裡跑著，不用另開。
- 程式會歸零（以船浮在水面時的深度、航向為 0），所以每次都在船浮著、上鎖時開始。上一個測試上鎖後，等幾秒讓船浮回水面。
- 同一時間只跑一支測試程式。兩支同時跑會互相切模式、解鎖、上鎖。
- 測試程式的深度上限 0.25 m 是實機測試池的深度（`water_common.py` 的 `MAX_DEPTH`）；模擬的泳池深 1.6～2 m。
- 結果 CSV 在 `deploy/sim/logs/fc_adapter/`。
- `/auv_control` 的用法與各欄位意思見 [`AuvControl.msg`](../../src/orca_msgs/msg/AuvControl.msg) 的註解。

## 執行中常用的操作

模擬起來之後（`./run.sh` 或 `./mission.sh` 都可以），另開終端機在 `deploy/sim/` 執行。ROS 指令借 `fc_adapter` 容器跑，所有服務都在同一個 ROS domain。

| 要做什麼 | 指令 |
|---|---|
| 看相機畫面 | `docker compose run --rm gui_tools ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw`。topic 換成 `/bottom_camera/image_raw` 是底部相機，`/detections/image`、`/detections/bottom_image` 是畫好偵測框的畫面，視窗左上的下拉選單也能切換 |
| 即時看裁判的事件、分數 | `docker compose exec fc_adapter bash -ic 'ros2 topic echo /sim/referee/events'`（分數與狀態在 `/sim/referee/state`），或 `tail -f logs/referee/<名稱>.jsonl` |
| 重新計分（不重開模擬） | `docker compose exec fc_adapter bash -ic 'ros2 topic pub --once /sim/referee/reset std_msgs/msg/Empty'` |
| 放球（只有 `sauvc_final.sdf` 有球） | `docker compose exec fc_adapter bash -ic 'ros2 topic pub --once /ball_dropper/release std_msgs/msg/Empty'` |
| 開關水下效果、調強度 | `docker compose exec fc_adapter bash -ic 'ros2 param set /sim_cameras underwater false'`；強度：`ros2 param set /sim_cameras underwater_strength 0.5`。只影響相機影像，世界裡的水面與水體霧要用 `UNDERWATER=0` 重開 |
| 船回到出發位置 | 重開 `simulator`：`WORLD=<世界> ./run.sh -d --force-recreate`，或直接再跑一次 `./mission.sh`。只想搬船不重開，見[排錯](#排錯gazebo-指令)的 `set_pose` |
| 單獨錄 rosbag | `BAG_NAME=名稱 docker compose --profile record up -d recorder`，停止用 `docker compose --profile record stop recorder`（送 SIGINT 讓它把檔案收尾，不要用 `down` 或 `kill`）。存在 `logs/bag/<名稱>/` |
| 偵測的 fps 與推論時間 | `docker compose logs -f object_detector`（每 5 秒印一次） |

## 驗收與偵測驗證

**規則事件**（`referee_check.sh`）：每個世界各起一次模擬（不開視窗），用外力把船推去撞道具，檢查裁判發出的事件與分數，以及該擋住的有擋住、該撞得動的撞得動。細節見 [`sim/tests/referee_check.py`](../../sim/tests/referee_check.py)。

```bash
cd deploy/sim
./referee_check.sh                    # sauvc_final.sdf、sauvc_qual.sdf 都跑，約 6 分鐘
./referee_check.sh sauvc_qual.sdf     # 只跑一個
```

結果印在畫面上（每一項 PASS／FAIL），也存到 `logs/referee/check_<世界>.json`；有 FAIL 時結束碼非 0。跑完會把 `simulator`、`sim_referee`、`sim_dropper` 停掉。

**偵測驗證**（`detect_sweep.sh`）：用現有的 RF-DETR 權重跑模擬畫面，量每個道具在多遠、什麼角度、什麼深度偵測得到，水下效果開與關各一輪。相機位姿是程式擺的（不用開船），每次跑結果一樣。不需要先起模擬。

```bash
cd deploy/sim
./detect_sweep.sh                         # 全部道具，約 1500 張影像，約 5 分鐘
QUICK=1 ./detect_sweep.sh                 # 試跑：每個道具 3 個距離
PROPS=gate,q_gate ./detect_sweep.sh       # 只跑這些道具（gate q_gate orange_flare flare_red flare_yellow flare_blue drum_red drum_blue surfacing_square）
UNDERWATER_STRENGTH=1.5 ./detect_sweep.sh # 水下效果的強度（「開」那一輪）
RUN=名稱 ./detect_sweep.sh                # 結果資料夾的名稱（預設 sweep_<時間>）
```

結果在 `logs/detect/<RUN>/`：`summary.md`（報告，跑完會印出來）、`summary.json`、`results.csv`、`samples/`、`frames/`（影像，約 100 MB）。

- 比較兩次：`python3 ../../tools/sim_detect/compare.py logs/detect/<A> logs/detect/<B>`
- 只重算推論、不重拍（例如換了權重）：`docker compose --profile sweep run --rm detect_eval python3 -u /orca/tools/sim_detect/evaluate.py /out/<RUN>`

## 哪些可以同時跑

- `run.sh`、`mission.sh`、`referee_check.sh` 用同一套容器，**一次只跑一個**。後跑的會重開 `simulator`；`referee_check.sh` 跑完還會把模擬停掉。
- `detect_sweep.sh` 自己起一個 gz server（另一個 gz partition），可以跟上面的同時跑，只是會搶 GPU。
- 跟筆電上的實機 `deploy/laptop` 不能同時跑：兩邊的 `fc_adapter` 都綁 UDP 14551。
- 同一時間只能有一個程式發 `/auv_control`（測試程式、鍵盤、行為樹擇一）。

## QGroundControl（選用）

在主機上開 QGC 就會自動連上（UDP 14550），可以看姿態、模式、參數。**QGC 的 joystick 要關掉**：QGC 與 mavros 的 system id 都是 255，兩邊的搖桿指令會打架。

## 改設定

| 改了什麼 | 怎麼生效 |
|---|---|
| repo 的 `sim/` 底下的世界、模型、`start.sh`，或 `firmware/ardusub/sitl.parm` | `docker compose up -d --force-recreate simulator`（直接掛進容器，不用重 build） |
| `sim/plugins/` 底下的 gz 外掛（例如推力曲線 `thrust_curve`） | 重新 build `simulator` 的 image（外掛在 image 裡編譯）再重開它。`run.sh`、`mission.sh` 每次都會 build，直接重跑就好 |
| `sim/referee.py`、`sim/gz_camera_bridge.py`、`sim/underwater.py` | `docker compose restart sim_referee`（或 `sim_cameras`） |
| 道具的尺寸、顏色、池子的貼圖（`tools/sim/gen_assets.py` 產生的那些） | 改 `gen_assets.py`，`python3 tools/sim/gen_assets.py` 重新產生 |
| `tools/pool_tests/*.py` | 重新執行該程式 |
| `src/` 底下的 `.py`、mavros 設定 | `docker compose restart fc_adapter`（`src/` 掛進容器，不用重 build） |
| 行為樹 `trees/*.yaml`、`config/` | 下次 `./mission.sh` 就生效。改了 YAML 要重新產生流程圖（見 [`src/orca_mission`](../../src/orca_mission/README.md#寫一棵樹)） |
| Dockerfile、`orca_msgs`、`package.xml`、`setup.py` | `docker compose build` 後再啟動（`mission.sh` 每次都會 build，沒改的話幾秒） |

`compose.yaml` 可用的環境變數：`GUI`（1 / 0）、`WORLD`（`sim/worlds/` 底下的世界檔）、`UNDERWATER`（1 / 0，水下的樣子：世界裡的水面與水體霧、相機影像的後處理）、`UNDERWATER_STRENGTH`（相機影像水下效果的強度，預設 1.0；0.5 更清澈、2 更混）、`DISPLAY`、`ROS_DOMAIN_ID`（預設 0）。`SEED` 由腳本處理（`seed.sh` 產生隨機佈局的世界檔，再把 `WORLD` 換成它）。

飛控每次開機都從 `sitl.parm` 重新載入參數（不保留上次改的值）。執行中用 QGC 或 mavros 改的參數，重開 sim 就會消失；要保留就寫進 `firmware/ardusub/sitl.parm`。

## 紀錄

`./mission.sh` 每次執行的紀錄同名，`<樹>_<時間>`：

- `logs/mission/<樹>_<時間>.csv`：行為樹的紀錄（20 Hz）。
- `logs/bag/<樹>_<時間>/`：rosbag（MCAP），用 [Foxglove](https://foxglove.dev) 直接開裡面的 `.mcap`（mission 結束後不到 1 秒就收尾好）。錄了 mavros、`/auv_control`、`/detections`、`/mission/status`（目前的節點與回饋）、`/sim/referee/events`、`/rosout` 等全部 topic；**影像只錄畫好框的 JPEG**（`/detections/image/compressed`、`/detections/bottom_image/compressed`），相機的原始影像不錄。一次過門（約 30 秒）約 12 MB。錄影設定在 `compose.yaml` 的 `recorder` 服務。
- `logs/referee/<樹>_<時間>.jsonl`：裁判的事件，一行一則 JSON（過門、碰撞、扣分、上浮…，見 [world-2027.md](world-2027.md#事件與計分)），樹跑完會印出來。

Foxglove 的 layout：[`tools/foxglove/orca_mission.json`](../../tools/foxglove/orca_mission.json)（Foxglove 右上角的 layout 選單 → Import from file）。分頁「過門」有前視、底部畫好框的影像，行為樹節點／飛控模式／解鎖／有沒有看到門，門的偏離與框寬、深度、推力；分頁「飛控、log」有航向、`/mavros/state`、`/rosout`。`/mission/status` 是 JSON 字串，layout 裡帶的 User Script（`tools/foxglove/*.ts`）會把它和 `/detections` 拆成欄位。改了之後用 `python3 tools/foxglove/make_layout.py` 重新產生。

全部的紀錄位置（都在 `deploy/sim/` 底下）：

| 位置 | 內容 |
|---|---|
| `logs/sim/ardusub/ardusub.out` | ArduSub 的終端機輸出 |
| `logs/sim/ardusub/logs/*.BIN` | 飛控的 DataFlash log（高頻姿態、控制器目標與出力），讀法見[排錯](#排錯gazebo-指令) |
| `logs/sim/mavproxy/` | mavproxy 的 tlog |
| `logs/fc_adapter/*.csv` | 水池測試程式的紀錄 |
| `logs/mission/*.csv` | 行為樹的紀錄 |
| `logs/bag/<名稱>/` | rosbag（MCAP） |
| `logs/referee/*.jsonl` | 裁判的事件與分數（`./mission.sh` 跑的跟 CSV 同名，其他用啟動時間）；`check_*.json` 是 `./referee_check.sh` 的結果 |
| `logs/detect/<RUN>/` | `./detect_sweep.sh` 的影像與結果（`summary.md`） |

`logs/` 底下的檔案由容器以 root 寫入，主機上刪除要 `sudo rm -rf logs/*`，或借容器刪：`docker run --rm -v "$PWD/logs:/l" orca-sim bash -c 'rm -rf /l/*'`。

## 排錯（Gazebo 指令）

以下在 `simulator` 容器裡執行（`docker compose exec simulator bash`，`GZ_PARTITION` 等環境變數已經設好）。`<世界>` 是世界檔名去掉 `.sdf`（`sauvc_qual`、`sauvc_final`、`orca_pool`）。

- **船的真實位姿**：`gz model -m orca -p`（位置 m，姿態 RPY rad，Gazebo 世界座標，原點是出發位置）。
- **把船搬到某個位姿**（不重開模擬）。船沒出力時會照浮力浮回水面，所以 z 只是起始值；`set_pose` 不會把速度歸零：

  ```bash
  gz service -s /world/sauvc_qual/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 2000 --req 'name: "orca", position: {x: 2, z: -0.3}'
  ```

- **推船（模擬被撞）**：`sauvc_qual.sdf`、`sauvc_final.sdf` 的世界檔已經載入 ApplyLinkWrench，直接送：

  ```bash
  # 沿世界 +x 推 20 N，持續到清除為止（改成 torque: {z: 40} 是繞 z 軸逆時針 40 N·m）
  gz topic -t /world/sauvc_qual/wrench/persistent -m gz.msgs.EntityWrench \
    -p 'entity: {name: "orca::base_link", type: LINK}, wrench: {force: {x: 20}}'
  gz topic -t /world/sauvc_qual/wrench/clear -m gz.msgs.Entity -p 'name: "orca::base_link", type: LINK'
  ```

  `orca_pool.sdf` 沒有載入，要先動態加。回應常常比逾時晚，但其實已經載入，不要重送（重送會載入第二份，力加倍）：

  ```bash
  gz service -s /world/orca_pool/entity/system/add --reqtype gz.msgs.EntityPlugin_V \
    --reptype gz.msgs.Boolean --timeout 3000 --req 'entity: {id: 1, type: WORLD},
    plugins: {name: "gz::sim::systems::ApplyLinkWrench", filename: "gz-sim-apply-link-wrench-system"}'
  ```

  不要用 `set_pose` 瞬移來模擬被推：瞬移時陀螺儀量不到轉動，只有磁力計會跳，EKF 的反應跟真的被推不一樣。
- **看飛控內部（控制器的目標與出力）**：MAVLink 遙測只有 4 Hz，而且沒有控制器的內部值。飛控的 DataFlash log 在 `logs/sim/ardusub/logs/*.BIN`（容器裡是 `/logs/ardusub/logs/`），用 pymavlink 讀：

  ```python
  from pymavlink import mavutil
  f = mavutil.mavlink_connection('/logs/ardusub/logs/00000001.BIN')
  while (m := f.recv_match(type=['ATT', 'RATE', 'CTUN', 'BARO'])) is not None:
      print(m)
  ```

  常用的訊息：`ATT`（`DesYaw` 目標航向、`Yaw` 實際航向）、`RATE`（`YDes` 目標角速度、`YOut` 控制器出力）、`CTUN`（`DAlt` 目標深度、`Alt` EKF 深度、`BAlt` 深度計）、`BARO`（原始水壓）。PID 各分量（`PIDY` 等）預設不記，要在解鎖前把 `LOG_BITMASK` 加上第 12 位元（4096），只對這次開機有效。
- **`mission.sh` 卡在「等偵測」**：第一次要載入模型，最多等 300 秒。逾時就看 `docker compose logs object_detector`；權重檔在 `ml/rfdetr/weights/checkpoint_best_total.pth`。
- **`mission.sh` 卡在「等飛控 EKF 對準」**：看 `docker compose logs simulator`。
- **鍵盤、`rqt_image_view` 的視窗開不起來**：主機跑一次 `xhost +SI:localuser:root`。

## 已知問題與限制

**模擬與實機的差異**

- 船的外觀、推進器位置取自 CAD（ORCA AUV 5.0 的「外殼重作版組合件」）；質量、慣量、浮力、阻力仍沿用 [NCTU-AUV/SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation) 的估計值，質心假設在 CAD 外框的中心（都還沒實測），而且模型沒有推進器的反應延遲。結果是模擬比實機「好控」：定深、轉向的過衝比實機小很多（轉 90° 過衝約 1.4°，實機 9～13°），定深時抵銷浮力的垂直出力約 −13 µs（實機約 −50 µs）。**模擬適合驗證流程、方向與邏輯，不適合拿來調 PID。**
- 推進器的 PWM → 推力查 T200 的公開推力表（自寫的 `orca::ThrustCurve` 外掛，16 V），有跟實機一樣的死區（約 1472～1528 µs 推力為 0）。鎖航向、定深的誤差小時 PWM 會落在死區，收斂比直線換算時慢。電壓要跟實機下水時的電池電壓一致（`model.sdf` 的 `<voltage>`）。細節見 [notes.md](notes.md)。
- **過門的平移對準（CenterGate）大多推不動船**：sway 只有約 0.03，換成 PWM 只偏離中位約 12 µs，落在 T200 的死區裡，推力是 0；大多 40 秒逾時照樣往前，過門靠盲走。實機很可能一樣，還沒修。細節與修法見 [notes.md](notes.md) 的「CAD 船體、推力死區」。
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
| `Lost manual control`（只跑 simulator、沒跑 fc_adapter 時） | 沒有程式在送搖桿；fc_adapter 起來後就不會出現 |

**操作上**

- 所有服務都用 host network，佔用本機的 UDP 9002、14550、14551 與 TCP 5760～5763。同一台電腦不能同時跑另一套 ArduPilot SITL 或 mavros。
- fc_adapter 在跑的時候，不要讓其他程式綁 UDP 14551。**筆電上的實機 fc_adapter（`deploy/laptop`）也綁 14551，兩者不能同時跑**，否則 SITL 的 MAVLink 會送進接著實機的 mavros：同一個埠只會有一個程式收到封包，mavros 會斷線。要直接讀 MAVLink，就在 QGC 沒開的時候聽 14550。
- ROS 2 Humble 的官方搭配是 Gazebo Fortress；這裡用 Harmonic。之後要接 ROS 橋接時用 `ros-humble-ros-gzharmonic`。

## 檔案

| 路徑（repo 根目錄起算） | 內容 |
|---|---|
| `deploy/sim/run.sh`、`deploy/sim/mission.sh`、`deploy/sim/compose.yaml` | 啟動、一個指令跑行為樹 |
| `deploy/sim/seed.sh`、`tools/sim/layout.py` | 決賽主場地的隨機佈局 |
| `docker/sim.Dockerfile` | ROS 2 Humble + Gazebo Harmonic + ArduSub Sub-4.7.2 SITL + [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) 外掛 + 自寫的 gz 外掛 + MAVProxy |
| `sim/start.sh` | 依序啟動 gz sim、ardusub、mavproxy |
| `sim/plugins/thrust_curve/` | 推力曲線外掛 `orca::ThrustCurve`：PWM 查 T200 推力表（`sim/models/orca/t200_bollard.csv`）再送給 Thruster |
| `firmware/ardusub/sitl.parm` | 飛控參數（取自實機）與 SITL 模擬硬體參數 |
| `sim/worlds/orca_pool.sdf` | 空池世界（25 × 16 m、深 1.6 m，水面 z = 0） |
| `sim/worlds/sauvc_qual.sdf`、`sauvc_final.sdf` | SAUVC 2027 資格賽、決賽主場地（見 [world-2027.md](world-2027.md)） |
| `sim/models/pool/`、`sim/models/orca/` | 舊泳池、Orca（船體、浮力、阻力、8 顆推進器、IMU、ArduPilotPlugin；外觀是 `meshes/*.stl`） |
| `tools/cad/sldasm_mesh.py` | SolidWorks 組合件 → `sim/models/orca/meshes/*.stl` 與各零件、推進器的姿態（不需要 SolidWorks） |
| `sim/models/orca_cam/` | Orca + 前視、底部相機（含 depth camera）+ 碰撞外框 |
| `sim/models/pool_2027/`、`water/` | 50 × 25 m 泳池、水面與水體霧 |
| `sim/models/` 其他 | 2027 的道具：`gate`、`q_gate`、`orange_flare`、`flare_red/yellow/blue`、`drum_red/blue/red_pinger`、`target_mat`、`surfacing_square`、`start_zone`、`start_marker`、`dropper_ball` |
| `tools/sim/gen_assets.py` | 產生泳池、水、門、flare、drum 的貼圖、mesh、SDF |
| `sim/gz_camera_bridge.py`、`sim/underwater.py` | Gazebo 前視、底部相機 → ROS 影像，含水下後處理（`sim_cameras` 服務） |
| `sim/referee.py`、`sim/gz_dropper_bridge.py` | 模擬裁判（`sim_referee`）、放球指令（`sim_dropper`） |
| `sim/tests/referee_check.py`、`deploy/sim/referee_check.sh` | 規則事件的驗收 |
| `tools/sim_detect/`、`deploy/sim/detect_sweep.sh` | 偵測驗證（`sim_sweep`、`detect_eval`） |
| `tools/foxglove/` | 看 rosbag 用的 Foxglove layout（`orca_mission.json`，由 `make_layout.py` 產生） |
| `src/orca_bridge/` | mavros + `auv_adapter.py`，跟實機共用 |
| `tools/pool_tests/` | 水池測試腳本 |

## 之後要加東西

- CAD 改版（外殼、推進器位置）：從雲端硬碟下載新的 `.SLDASM`，跑 `python3 tools/cad/sldasm_mesh.py <組合件.SLDASM> sim/models/orca/meshes`。它會重寫 STL，並印出各零件在機體座標的 pose、各推進器的中心與軸向，手動更新 `sim/models/orca/model.sdf` 的 `<visual>`、`thruster_N` 的 `<pose>`，以及 `sim/models/orca_cam/model.sdf` 的碰撞外框。推力方向沿用原本 joint 軸的正負號，`ArduPilotPlugin` 的對應與 `<multiplier>` 就不用改。零件名稱換了要用 `--name 零件名=檔名` 指定 STL 檔名。
- 新的世界：`<include>` `pool_2027`、`water`（夾在 `UNDERWATER:BEGIN`／`END` 之間）、道具與 `orca_cam`，Buoyancy 一定要 `<enable>orca::base_link</enable>`（原因見 `models/orca_cam` 的註解）。`sauvc_qual.sdf`、`sauvc_final.sdf` 可以當範本。
- 新的計分規則：`sim/referee.py`，再到 `sim/tests/referee_check.py` 加一項驗收。
- ROS 橋接、視覺、任務層：在 `deploy/sim/compose.yaml` 新增服務（ROS 2 Humble、host network、`ROS_LOCALHOST_ONLY=1`），不改 `simulator`、`fc_adapter` 的埠與 topic。
