# 筆電跑 ROS、RPi 轉接（現行做法）

ROS 2 節點全部跑在筆電，RPi 不跑 ROS，只負責把相機和飛控「接出來」。之後主電腦定案時，整套搬到那台（見 [architecture.md](architecture.md) 的「部署」）。

```
             RPi（不跑 ROS）                         筆電（deploy/laptop，ROS 2 Humble）
 D435i ─USB─> cam-relay  ──TCP 5600（筆電主動連）──> cam_bridge ─> /camera/camera/color/image_raw
                                                                       └> detector ─> /detections
 CUAV V5 ─USB─> mavproxy ──UDP 14560（udpin，筆電主動送）──> fc_adapter（mavros + auv_adapter）
                 └> bcast-relay → QGC（14550 廣播，不變）
```

兩條連線都由筆電發起，筆電不用開防火牆。筆電與 RPi 之間是 100 Mbps 的 USB 網卡，所以影像在 RPi 上先壓成 JPEG。

## RPi 端（只要做一次）

在筆電上執行，`rpi` 是 `~/.ssh/config` 裡 RPi 的名稱（可用 `RPI_HOST` 改）：

```bash
cd deploy/rpi-relay
./deploy.sh                    # 安裝 cam-relay 並設成開機自動跑（取代 realsense-web）
./deploy.sh revert             # 改回 realsense-web
scp mavproxy-udpin.sh rpi: && ssh -t rpi sudo bash mavproxy-udpin.sh   # mavproxy 加 udpin 14560（要 sudo）
```

| 檔案 | 內容 |
|---|---|
| `cam_relay.py` | pyrealsense2 讀彩色影像 → JPEG → TCP。通訊協定寫在檔頭 |
| `cam-relay.service` | RPi 的使用者層級服務，跟 `realsense-web.service` 互斥（搶同一台相機） |
| `deploy.sh` | 部署 cam-relay |
| `mavproxy-udpin.sh` | 在 mavproxy.service 加 `--out=udpin:0.0.0.0:14560`，會先備份 unit |

## 筆電端

```bash
ml/rfdetr/fetch_weights.sh                 # 第一次
cd deploy/laptop
./up.sh                                    # 開場次（或沿用）、起服務、開始全程錄影，等飛控、相機、偵測、錄影就緒；映像不存在或過期才 build，已經在跑的不重開
./up.sh --build                            # 不管指紋，全部重 build；映像有變的容器重開
./down.sh                                  # 收工：場次收尾（停錄影、存終端輸出、參數快照）再全部關掉。不要直接 docker compose down
docker compose up -d front_camera object_detector   # 只跑影像與偵測（飛控沒接時；不開場次、不錄）
docker compose logs -f object_detector             # 每 5 秒印 fps 與延遲

# 看偵測畫面（主機先跑 xhost +SI:localuser:root）
docker compose run --rm gui_tools ros2 run rqt_image_view rqt_image_view /detections/image

# 水池測試腳本（tools/pool_tests/water_*.py），參數接在後面
./pool_test.sh water_depth 0.1 0.2

# 鍵盤操控（主機先跑 xhost +SI:localuser:root），M 插標記。按鍵與安全機制見 tools/teleop/keyboard.py 檔頭
docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py

# 過門（行為樹，src/orca_mission/trees/gate_pass.yaml）：先跑 ./up.sh、再跑行為樹（解鎖前會等你按 Enter）
./mission.sh

# 紀錄：插標記、切出一段（見下面的「紀錄」）
./mark.sh surge 0.4 第 2 趟
./slice.sh --match gate_pass_20261120_101500
```

上岸檢查飛控連線（不要解鎖）：`docker compose exec fc_adapter bash -ic 'ros2 topic echo --once /mavros/state'` 要看到 `connected: true`，QGC 也要同時連得上。

### 啟動與收工流程（`./up.sh`、`./mission.sh`、`./down.sh`）

- `./up.sh`：沒有權重就先下載 → 映像不存在或過期才 build → 決定場次（見「紀錄」）→ 起 `front_camera`、`object_detector`、`fc_adapter`、`recorder`、`marker_log` → 等飛控連線（60 秒）、相機（60 秒）、偵測（300 秒）、開始錄影（30 秒）→ 新場次的話拍飛控參數快照、插「場次開始」標記。已經在跑、映像沒換的容器不重開，mavros 的連線不會斷。不起 `mission`。
- 「過期」是指映像的 `orca.build-hash` label 跟 [`docker/build-hash.sh`](../docker/build-hash.sh) 現在算出來的不同：改了 Dockerfile、`package.xml`、`setup.py`、msg，或 `src/` 新增、刪除、改名檔案才會重 build；改 `.py`、樹的 yaml 不會（`src/` 掛進容器，重跑任務就生效）。要強制重 build 用 `./up.sh --build`。
- 模擬的 `./run.sh`、`./mission.sh` 跟實機共用映像，build 時也會蓋上同樣的指紋，跑過模擬再回來跑實機不會白 build。
- `./mission.sh [樹] [-p 變數=值 ...]`：先跑 `./up.sh`（沿用場次）→ 跑行為樹（列檢查清單、等 Enter）→ 樹結束、上鎖、船浮上水面 → 印出切出這次任務的指令。開始、結束都會插標記。
- `./down.sh`：插「場次結束」標記、停 `recorder` 並確認每個 bag 都有收尾 → 各容器的終端輸出存成 `containers.log` → 拍結束時的飛控參數快照 → （飛控 `.BIN` 還沒自動拉）→ `docker compose down`（所有 profile）→ 記下收尾方式、刪掉 `current`。
- 改了 `compose.yaml` 要套用到已經在跑的容器：`docker compose up -d <服務>`。

## 紀錄

一次下水的全部紀錄放在一個場次（session）資料夾：`./up.sh` 開始，`./down.sh` 結束。中間不管跑行為樹、水池測試、鍵盤操控，還是只在岸上調東西，rosbag 都在錄。

路徑都在 `deploy/laptop/logs/session/` 底下，`current` 是目前場次的名稱（場次名稱是開始時間）：

| 路徑 | 內容 | 誰寫的 |
|---|---|---|
| `<場次>/bag_<時間>/` | 全程 rosbag（MCAP），每 5 分鐘一個 `.mcap`。recorder 每次啟動（包括出錯後自動重開）開一個新資料夾 | `recorder` |
| `<場次>/markers.csv` | 標記：時間、來源、內容 | `marker_log` |
| `<場次>/mission/<樹>_<時間>.csv` | 行為樹的紀錄（20 Hz） | `./mission.sh` |
| `<場次>/pool_tests/<腳本>_<時間>.csv` | 水池測試腳本的紀錄（5 Hz） | `./pool_test.sh` |
| `<場次>/info/start.txt`、`git.txt`、`git.diff` | 開始時間、主機、開機 ID；開始時的 commit、`git status`、未提交的修改 | `./up.sh` |
| `<場次>/info/fc_params_start.parm`、`fc_params_end.parm` | 開始、結束時的飛控參數，格式同 `firmware/ardusub/real.parm`，可以直接 diff | `./up.sh`、`./down.sh` |
| `<場次>/info/closed.txt` | 怎麼收尾的 | `./down.sh`，或下一次 `./up.sh` 補救 |
| `<場次>/containers.log` | 各容器的終端輸出 | `./down.sh` |
| `<場次>/slices/<名稱>/` | 切出來的片段 | `./slice.sh` |

這個版本之前的紀錄留在原處（`logs/bag/`、`logs/mission/`、`logs/fc_adapter/`，更早的在 `logs/bridge/`），不會搬動。

### 場次的開始與結束

- `./up.sh` 會被重複呼叫（`./mission.sh` 每次都先跑它）。已經有場次、`recorder` 還在錄，就沿用。
- 沒有場次就開新的：記開始資訊和程式版本，起服務後拍參數快照、插「場次開始」標記。
- 沒用 `./down.sh` 收尾（直接 `docker compose down`、重開機、斷電）的話，下一次 `./up.sh` 會看到 `current` 還在，但 `recorder` 沒在錄（或開機 ID 不同），就先補做還能補的：停止錄影、確認 bag 收尾，飛控連上後補拍結束時的參數快照，寫進 `closed.txt`，再開新場次。容器的終端輸出這時已經沒了。
- 重開機後，`recorder`、`marker_log`（`restart: unless-stopped`）會自己起來，繼續錄進原本的場次，直到下一次 `./up.sh` 把它收掉、開新場次。

### rosbag 錄什麼

- 除了未壓縮的影像（`sensor_msgs/Image`）以外的全部 topic：mavros、`/auv_control`、`/detections`、`/mission/status`、`/session/marker`、`/rosout`（節點的 log，包括鍵盤操控的模式切換、解鎖、歸零、GAIN 變化），以及原始 MAVLink（`/uas1/mavlink_source`）。
- 相機錄兩路 JPEG：RPi 送來的原始 JPEG（`/camera/camera/color/image_raw/compressed`，不重新壓縮），以及畫好框的 JPEG（`/detections/image/compressed`）。估計約 9 GB／小時：兩路 JPEG 實測每秒約 1.3 MB 和 1.1 MB，其他 topic 很小。
- 要改錄哪些 topic 看 `compose.yaml` 的 `recorder`。
- mavros 的遙測（姿態、深度、馬達輸出）目前只有約 4 Hz，要提高到 20 Hz（待辦，要改 RPi 上的 MAVProxy）。bag 拿來看全貌、找段落、對照影像；系統辨識的主要資料是飛控的 DataFlash（`.BIN`）。

### 標記

- 標記發到 `/session/marker`（`std_msgs/String`，內容是「來源: 文字」），會錄進 bag，同時由 `marker_log` 寫進 `markers.csv`。
- 來源：
  - `./mark.sh <文字>`。
  - 鍵盤操控按 M，內容是編號 `#1`、`#2`…；開始、結束也各一筆。
  - 行為樹和水池測試：開始一筆，帶程式版本（commit 前 7 碼，有未提交的修改加 `+dirty`）；結束一筆，帶結果。
  - `./up.sh` 的場次開始、`./down.sh` 的場次結束。
- 容器裡沒有 `.git`，程式版本是主機上的腳本算好傳進去的。所以水池測試要用 `./pool_test.sh` 跑，直接 `docker compose exec` 跑的話，標記裡的版本是「未知」。

### 看紀錄、切片

- 用 [Foxglove](https://foxglove.dev) 開 `.mcap`，layout 用 [`tools/foxglove/orca_mission.json`](../tools/foxglove/orca_mission.json)。
- 只要某一段（例如一次任務）就先切出來：`./slice.sh --match <文字>` 從 `markers.csv` 找含這段文字的標記，從第一筆切到最後一筆（前後各多 2 秒），存成 `<場次>/slices/<名稱>/` 底下一個獨立的 `.mcap`。
  - 給行為樹的 run 名稱（`<樹>_<時間>`，`./mission.sh` 結束時會印出整行指令）就是整次任務。
  - 其他用法：`./slice.sh --match 'keyboard: #3' --duration 60`、`./slice.sh --start 10:15 --end 10:18:30`。不是目前場次的話，場次名稱放第一個參數。
  - 正在錄的 `.mcap` 最後幾秒還在緩衝區、沒寫進檔案，切到「剛剛」會被拒絕，等一下或收工後再切。確定只要到目前為止的部分就加 `--partial`。
- 回放：先 `./down.sh`，再 `docker compose run --rm gui_tools ros2 bag play /sessions/<場次>/slices/<名稱>`（`gui_tools` 把 `logs/session/` 掛在 `/sessions`）。

### 飛控的 DataFlash（`.BIN`）

- 飛控 SD 卡上的高頻紀錄：姿態、IMU、馬達輸出、控制器內部。`LOG_DISARMED 0`，所以只有解鎖期間有紀錄。
- 還沒自動拉回場次資料夾。拉檔方式、跟 bag 的對時、記錄頻率，都要在實機量過再決定（待辦）。現在要的話，用 QGC 的 Analyze Tools → Log Download，或拔 SD 卡。

### 磁碟

- 估計約 9 GB／小時。筆電還有約 528 GB，一次下水 3 小時的話大約夠 19 個場次。
- 不自動刪除。快滿之前把場次資料夾搬到外接硬碟，確認搬過去的檔打得開，再刪掉筆電上的。`logs/` 底下有容器以 root 寫的檔案，刪除要 `sudo`。

### 模擬的錄法不一樣

模擬（`deploy/sim`）目前還是一次任務錄一個 bag（`deploy/sim/logs/bag/`），沒有場次，標記不會寫成 `markers.csv`（bag 裡還是有 `/session/marker`）。場次相關的腳本（[`deploy/common.sh`](../deploy/common.sh)、[`src/orca_session`](../src/orca_session)）寫成跟跑法無關，之後要統一實機和模擬的格式時，模擬可以直接用。

## 實測數字（2026-11-12）

- `cam_bridge`：29 fps，約 3 Mbit/s（640×480 JPEG 品質 80），拍攝到筆電收到約 3 ms。`camera_info` 內參 fx ≈ 606、cx ≈ 320。D435i 要接在 RPi 的 USB 3。
- `detector`（筆電 RTX 4050，FP32）：約 27 fps，推論 37 ms，拍攝到 `/detections` 約 60 ms。

## 踩過的坑

- **容器之間看得到 topic 卻收不到資料**：Fast DDS 在同一台機器上走 `/dev/shm`，容器各自有 `/dev/shm`。compose 的 ROS 服務都設了 `ipc: host`。
- **推論比相機慢時延遲越積越多**：`qos_profile_sensor_data` 的佇列深度是 5，會排出約 150 ms 的延遲。detector 改用 best effort、深度 1，只處理最新一幀。
- **節點類別不能有叫 `handle` 的方法**：會蓋掉 rclpy 的 `Node.handle`，建構時報 `AttributeError: __enter__`。
- **RF-DETR 的 `class_id`** 是 `model.class_names` 的 0 起算索引；資料集 COCO 標註的 category id 是 1 起算（0 是上層類別 `sauvc-objects`）。
- **rfdetr 會把 setuptools 升到新版**，新版在 colcon build ament_python 時失敗。perception 的 Dockerfile 在另一個 stage build workspace。

## 風險

- 控制要經過這條 USB 網卡和筆電，斷線時只能靠 ArduSub 的 failsafe（`FS_PILOT_TIMEOUT`、GCS failsafe），這些還沒設好。
- RPi 上原本的 mavros 容器停用後，`water_*.py` 改在筆電的 fc_adapter 容器裡跑。
