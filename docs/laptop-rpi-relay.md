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
./up.sh                                    # 一次起下面三個服務並等飛控、相機、偵測就緒；映像不存在或過期才 build，已經在跑的不重開
./up.sh --build                            # 不管指紋，全部重 build；映像有變的容器重開
docker compose up -d front_camera object_detector   # 影像與偵測
docker compose up -d fc_adapter                     # 飛控（RPi 的 mavproxy 要先加 udpin）
./mission.sh                               # 跑行為樹；樹跑的期間才錄 rosbag → logs/bag/<樹>_<時間>/（RECORD=0 不錄）
docker compose logs -f object_detector             # 每 5 秒印 fps 與延遲

# 看偵測畫面（主機先跑 xhost +SI:localuser:root）
docker compose run --rm gui_tools ros2 run rqt_image_view rqt_image_view /detections/image

# 水池測試腳本
docker compose exec fc_adapter bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'

# 鍵盤操控（主機先跑 xhost +SI:localuser:root）。按鍵與安全機制見 tools/teleop/keyboard.py 檔頭
docker compose run --rm gui_tools python3 /orca/tools/teleop/keyboard.py

# 過門（行為樹，src/orca_mission/trees/gate_pass.yaml）：先跑 ./up.sh、再跑行為樹（解鎖前會等你按 Enter）
./mission.sh
# 舊版過門（mission/gate_pass.py）
./gate_pass.sh
```

上岸檢查飛控連線（不要解鎖）：`docker compose exec fc_adapter bash -ic 'ros2 topic echo --once /mavros/state'` 要看到 `connected: true`，QGC 也要同時連得上。

### 啟動流程（`./up.sh`、`./mission.sh`）

- `./up.sh`：沒有權重就先下載 → 映像不存在或過期才 build → 起 `front_camera`、`object_detector`、`fc_adapter` → 等飛控連線（60 秒）、相機（60 秒）、偵測（300 秒）。已經在跑、映像沒換的容器不重開，mavros 的連線不會斷。不起 `recorder`、`mission`。
- 「過期」是指映像的 `orca.build-hash` label 跟 [`docker/build-hash.sh`](../docker/build-hash.sh) 現在算出來的不同：改了 Dockerfile、`package.xml`、`setup.py`、msg，或 `src/` 新增、刪除、改名檔案才會重 build；改 `.py`、樹的 yaml 不會（`src/` 掛進容器，重跑任務就生效）。要強制重 build 用 `./up.sh --build`。
- 模擬的 `./run.sh`、`./mission.sh`、`./gate_pass.sh` 跟實機共用映像，build 時也會蓋上同樣的指紋，跑過模擬再回來跑實機不會白 build。
- `./mission.sh [樹] [-p 變數=值 ...]`：先跑 `./up.sh` → 開始錄 rosbag → 跑行為樹（列檢查清單、等 Enter）→ 樹結束、上鎖、船浮上水面後停止錄影，最後印出 bag 的路徑。`RECORD=0 ./mission.sh` 不錄。
- `./gate_pass.sh`（舊版過門）也是先跑 `./up.sh`，不錄 rosbag。
- 改了 `compose.yaml` 要套用到已經在跑的容器：`docker compose up -d <服務>`。全部關掉：`docker compose down`。

## 紀錄（CSV、rosbag）

路徑都在 `deploy/laptop/` 底下：

| 路徑 | 內容 | 誰寫的 |
|---|---|---|
| `logs/mission/<樹>_<時間>.csv` | 行為樹的紀錄（20 Hz） | `./mission.sh` |
| `logs/bag/<樹>_<時間>/` | rosbag（MCAP），一次任務一個 `.mcap`，跟 CSV 同名 | `./mission.sh`（`RECORD=0` 不錄） |
| `logs/fc_adapter/*.csv` | 水池測試腳本、舊版過門的紀錄 | `water_*.py`、`./gate_pass.sh` |

- rosbag 從樹開始（列檢查清單時）錄到樹結束、船浮上水面（上鎖後最多等 30 秒，再按一次 Ctrl-C 不等）。`docker compose up -d`、`./up.sh`、`./gate_pass.sh`、水池測試腳本都不錄。
- 錄全部 topic（mavros、`/auv_control`、`/detections`、`/mission/status`、`/rosout` 的節點 log）。影像只錄畫好框的 JPEG（`/detections/image/compressed`），相機的影像（原始、JPEG）都不錄，約 4 GB／小時。要改錄哪些 topic 看 `compose.yaml` 的 `recorder`。
- 看 bag：用 [Foxglove](https://foxglove.dev) 開 `.mcap`（還在寫的檔打不開），layout 用 [`tools/foxglove/orca_mission.json`](../tools/foxglove/orca_mission.json)。回放：先 `docker compose down`，再 `docker compose run --rm gui_tools ros2 bag play /bags/<樹>_<時間>`（`gui_tools` 把 `logs/bag/` 唯讀掛在 `/bags`）。
- 不經 `./mission.sh` 單獨錄：`BAG_NAME=<名稱> docker compose --profile record up -d recorder`，存到 `logs/bag/<名稱>/`；停止用 `docker compose --profile record stop recorder`（送 SIGINT，讓它把檔案收尾）。
- `logs/` 底下的檔案由容器以 root 寫入，主機上刪除要 `sudo`。

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
