# 筆電跑 ROS、RPi 轉接（現行做法）

ROS 2 節點全部跑在筆電，RPi 不跑 ROS，只負責把相機和飛控「接出來」。之後主電腦定案時，整套搬到那台（見 [architecture.md](architecture.md) 的「部署」）。

```
             RPi（不跑 ROS）                         筆電（deploy/laptop，ROS 2 Humble）
 D435i ─USB─> cam-relay  ──TCP 5600（筆電主動連）──> cam_bridge ─> /camera/camera/color/image_raw
                                                                       └> detector ─> /detections
 CUAV V5 ─USB─> mavproxy ──UDP 14560（udpin，筆電主動送）──> bridge（mavros + auv_adapter）
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
docker compose up -d cam_bridge detector   # 影像與偵測
docker compose up -d bridge                # 飛控（RPi 的 mavproxy 要先加 udpin）
docker compose up -d recorder              # 所有 topic 錄成 rosbag → logs/bag/（docker compose up -d 會一起起）
docker compose logs -f detector            # 每 5 秒印 fps 與延遲

# 看偵測畫面（主機先跑 xhost +SI:localuser:root）
docker compose run --rm tools ros2 run rqt_image_view rqt_image_view /detections/image

# 水池測試腳本
docker compose exec bridge bash -ic 'python3 /orca/tools/pool_tests/water_depth.py'

# 鍵盤操控（主機先跑 xhost +SI:localuser:root）。按鍵與安全機制見 tools/teleop/keyboard.py 檔頭
docker compose run --rm tools python3 /orca/tools/teleop/keyboard.py

# 過門：一個指令起三個服務、等飛控／相機／偵測就緒、開始 mission/gate_pass.py（解鎖前會等你按 Enter）
./gate_pass.sh
```

上岸檢查飛控連線（不要解鎖）：`docker compose exec bridge bash -ic 'ros2 topic echo --once /mavros/state'` 要看到 `connected: true`，QGC 也要同時連得上。

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
- RPi 上原本的 mavros 容器停用後，`water_*.py` 改在筆電的 bridge 容器裡跑。
