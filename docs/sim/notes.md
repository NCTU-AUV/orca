# 開發過程的特別事項

> 這份是建立模擬時的紀錄（2026-11-10），寫的是搬進 orca repo 之前的檔案配置。對照：
> `adapter/` → `src/orca_bridge/`（程式、mavros 設定）與 `tools/pool_tests/`（測試腳本）；
> `sim/params/orca.parm` → `firmware/ardusub/sitl.parm`；`sim/Dockerfile` → `docker/sim.Dockerfile`；
> `run.sh`、`compose.yaml` → `deploy/sim/`；compose 的 `adapter` 服務 → `bridge`；`logs/adapter/` → `deploy/sim/logs/bridge/`。


建置這套模擬（M0～M5，見 [plan.md](plan.md) 的里程碑）時遇到的坑、不直觀的設定，以及排錯的方法。之後要改這套模擬、或發現行為怪怪的時候，先看這裡。

每一項的格式：現象 → 原因 → 處理。檔案裡有對應註解的，也標出位置。

## M0：可行性（容器、GPU、套件）

- **ROS 2 Humble 的官方搭配是 Gazebo Fortress，不是 Harmonic。** Harmonic 從 OSRF 的 apt 套件庫裝進 `ros:humble`（Ubuntu 22.04）可以用；之後要接 ROS 橋接時，`ros-humble-ros-gzharmonic` 在 apt 解析得到。
- **混合顯卡筆電（NVIDIA PRIME on-demand）**：容器要帶 `__NV_PRIME_RENDER_OFFLOAD=1`、`__GLX_VENDOR_LIBRARY_NAME=nvidia` 與 `NVIDIA_DRIVER_CAPABILITIES=all`，`glxinfo -B` 才會顯示 NVIDIA。
- **Wayland 桌面**：容器裡的 Gazebo 視窗走 XWayland（`DISPLAY`），要先 `xhost +SI:localuser:root` 允許容器的 root 連線，`run.sh` 會做。

## M1：Gazebo 世界與船

- **SAUVC-Simulation 是 Fortress（Ignition）時期的寫法**：外掛檔名與命名空間要改成 Harmonic 的 `gz-sim-*-system`、`gz::sim::systems::*`。
- **船靜止時浮在 z = −0.210 m（模型原點），傾斜後約 3 秒回正**：浮力體中心在質心上方 0.115 m，浮心高於重心，所以會自己回正。
- **ArduPilotPlugin 一定要有一顆 IMU**：送給 SITL 的陀螺儀與加速度取自它。IMU 繞 x 轉 180°，讀數才是 ArduPilot 的機體座標 FRD（`models/orca/model.sdf`）；世界檔要載入 `gz-sim-imu-system`，IMU 才會更新。原本的規格寫「不放任何感測器」，已改。

## M2：ArduSub SITL ⇄ Gazebo

- **ArduPilotPlugin 的 `no_time_sync` 預設值會讓飛控不跑 EKF**：預設 `1` 時 SITL 會把 `AHRS_EKF_TYPE` 改成 10（直接用模擬真值），在 `orca.parm` 設參數也蓋不掉。模型裡設 `<no_time_sync>0</no_time_sync>`，飛控才跟實機一樣跑 EKF3。**不要拿掉。**
- **關掉模擬的 GPS（`SIM_GPS1_ENABLE 0`）**：實機沒有 GPS，關掉後 EKF 才跟實機一樣沒有水平位置。副作用見下面 M3 的「看起來像錯誤的訊息」。
- **`GZ_IP=127.0.0.1`**（`compose.yaml`）：讓 Gazebo 的通訊只走本機，不與同網段其他電腦的 Gazebo 互通。
- **ArduPilot 淺層複製（`--depth 1`）後要自己 `git submodule update --init --recursive --depth 1`**：交給 `./waf` 自動補子模組不可靠。
- **ArduPilot 的建置需要 `empy` 3.x**，裝到 4.x 會失敗（`sim/Dockerfile` 固定 3.3.4）。
- **MAVProxy 用 `pip install --no-deps` 裝**：只做 MAVLink 轉送，不需要它的地圖、繪圖功能帶進來的 OpenCV、VTK、matplotlib，image 小很多。
- **推進器方向**：`MOT_n_DIRECTION` 照實機設定（−1, 1, −1, 1, 1, −1, −1, 1）。實機用它補償螺旋槳正反轉與接線，模擬沒有這些差異，所以在外掛的推力倍率乘上相同的正負號抵銷（`models/orca/model.sdf` 的 `<multiplier>`）。這樣 PWM 的意義、測試程式裡的 `VERT_SIGN`、`YAW_SIGN` 都與實機一致。
- **馬達對應**經 MANUAL 下 12 個方向（前後、左右、上下、左右轉、pitch、roll 各正負）逐一實測確認，表在 PLAN.md。
- **航向的數值**：船頭朝 Gazebo 的 +x（東）時，飛控報約 77°，不是 90°。沒有 EKF 原點時飛控不做磁偏角補償，報的是磁北航向；差的 13° 是模擬地點的磁偏角。實機一樣沒有原點。
- **MAVLink 測試的陷阱**（直接用 pymavlink 送指令時）：
  - ArduSub 只收 system id = `MAV_GCS_SYSID`（255）送的 `MANUAL_CONTROL`，其他的被默默丟掉。
  - 解鎖後要持續送 `MANUAL_CONTROL`，停超過 `FS_PILOT_TIMEOUT`（3 秒）就自己上鎖。
  - roll、pitch 要用擴充欄位：`manual_control_send(sys, x, y, z, r, 0, 0, 3, s, t)`。
  - 解鎖前要等 EKF 對準：`EKF_STATUS_REPORT.flags` = `0x0a7`，沒有 `0x400`。
  - 量 yaw 要邊轉邊取樣、累加相鄰兩點的角度差。只比頭尾兩點的話，轉超過 180° 會繞回去而誤判方向。
  - **同一個 UDP 埠只能有一個程式收**：mavros（adapter）在跑時，不要讓其他程式綁 14551，封包會被分走（實機也出過這個事故）。

## M3：鎖航向與定深

- **定深完全失效：船被全力推到池底，飛控卻以為深度一直是 0。**
  - 原因：ArduSub 的模擬深度計把 SITL 的「海拔高度」直接當成水深來算水壓，而 SITL 預設的起始位置是一座海拔 584 m 的機場，算出的水壓是負的（log 裡 `BARO.Press` ≈ −5.75×10⁶ Pa）。深度計失效，EKF 估不到下潛，ALT_HOLD 就一直往下推。
  - 處理：`orca.parm` 設 `SIM_OPOS_ALT 0`（起始海拔 = 水面）。上游 [bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) 也是用海拔 0 的地點啟動。
  - M2 沒發現，是因為 MANUAL 模式用不到深度計。
- **修好之後飛控深度還在亂跳（約 50 cm），和真實深度對不上。**
  - 原因：模擬深度計的雜訊 `SIM_BARO_RND` 預設 0.2 m，是給空中載具氣壓計用的值。
  - 處理：設成 0.0003（±0.3 mm，實機 Bar02 靜止時的量級），第二顆 `SIM_BAR2_RND` 也一樣。
- **飛控深度比 Gazebo 真實深度多約 2.5%**：SITL 用海水密度 1024 kg/m³ 算水壓，飛控用 9800 Pa/m（淡水）換回深度。0.5 m 時差約 1.2 cm。沒有修正（要改就得動飛控參數 `BARO_SPEC_GRAV`，與實機不同）。
- **「被推偏」不能用 `set_pose` 瞬移來模擬**：瞬移時陀螺儀量不到轉動，只有模擬的磁力計會跳，跟真的被推不一樣，EKF 可能拒絕磁力計的新值。改成對船施加一段時間的力矩，見下面「排錯方法」。
- **推回原航向後停在偏 3～5° 的地方，要十幾秒才慢慢回來。**
  - 這是控制器真正的行為，不是模擬錯誤。打開 PID 記錄看分量：推開的那段時間，yaw 角速度迴路的積分項累積到約 +0.08；船回來之後，比例項（−0.028）剛好被積分項（+0.028）抵銷，有誤差卻幾乎沒出力，只能等積分慢慢消掉（`ATC_RAT_YAW_I` 只有 0.018）。這叫積分飽和（integrator windup）。
  - 結果：推開 75° 時 15 秒回到 ±2°，推開 32° 時 6.5 秒；30 秒後都在 1° 以內。實機岸上放手後的紀錄是 −0.8～+1.4°。
- **看起來像錯誤、其實正常的訊息**：
  - `Compass performance degraded`（解鎖時）：ArduSub 解鎖時發現沒有 EKF 原點，不能查磁偏角表，只是警告。實機同樣沒有 GPS、沒有原點。
  - `GPS 1: detected u-blox`：`SIM_GPS1_ENABLE 0` 在 SITL 的意思是「GPS 還在，但永遠不定位」，所以偵測得到、沒有定位。

## M4：adapter（mavros + auv_adapter.py）

- **adapter 程式、`mavros.yaml`、Dockerfile 都不用改**：實機的 `fcu_url` 本來就是 `udp://127.0.0.1:14551@`，模擬的 mavproxy 往同一個埠送。唯一的差別是部署方式，記在 `adapter/CHANGES.md`。
- **遙測約 4 Hz**（姿態、深度、PWM），adapter 送搖桿 20 Hz，與實機相同。
- **mavros 的 `CMD: Unexpected command 410, result 4`、`GP: No GPS fix`**：沒有 GPS 就沒有 home 位置，可忽略。
- **深度以約 1 cm 為單位跳動**（例如 0.143 ↔ 0.154 m），看起來是飛控回報高度的解析度。實機紀錄裡約 1 cm 的「波動」可能也是同一個原因。
- **結果與實機比較**：每一段都在約 2 秒內到達，轉 90° 約 1.5 秒（被 `ATC_RATE_WPY_MAX` 60°/s 限住，與實機相符）。但模擬的過衝小很多（轉向約 1.4°，實機 9～13°；定深最多 0.8 cm，實機最多 6.2 cm），抵銷浮力的垂直出力約 −13 µs（實機約 −50 µs）。原因是船的物理參數是估計值、模型也沒有推進器的反應延遲。**模擬適合驗證流程與方向，不適合調 PID。**
- **同一時間只能有一支測試程式**：開發時曾有兩支 `water_heading.py` 同時對同一套模擬執行，一支解鎖後另一支判定「飛控是解鎖狀態」而退出、或被中止。測試程式彼此不知道對方存在，會互相切模式、解鎖、上鎖。

## M5：交付

- **從零重建（`docker compose build --no-cache --pull`）可以成功**，在 12 核筆電上約 7.5 分鐘（`ros:humble` 基底已在本機）。build log 裡兩則看起來像錯誤的訊息是正常的：
  - `ln: failed to create symbolic link '/etc/resolv.conf': Device or resource busy`：裝 Gazebo 時順帶裝了 systemd，它的安裝腳本想換掉 `/etc/resolv.conf`，但 build 時那個檔案由 Docker 掛進來，換不掉。不影響安裝。
  - pip 的 `dependency conflicts`（MAVProxy 缺 matplotlib、opencv-python 等）：MAVProxy 刻意用 `--no-deps` 裝，見 M2。
- **Gazebo 視窗啟動時的 `libEGL warning: egl: failed to create dri2 screen`**：Mesa 嘗試用內顯失敗的訊息。視窗實際跑在 NVIDIA 上（`nvidia-smi` 看得到 `gz sim -g`），可忽略。
- **乾淨重跑的結果與 M4 一致**：`water_depth.py` 三段 0.8～1.0 s 到達、過衝 0.4～0.8 cm；`water_heading.py` 四段 1.7～1.9 s 到達、過衝 1.4～1.6°。同樣的設定每次跑的結果幾乎相同，可以拿來做回歸測試（改了程式後確認行為沒變）。
- **確認有沒有測試程式在跑**：`docker compose exec adapter pgrep -af 'python3 /work/water_'`。不要寫成 `bash -c 'pgrep -af water_'`，`pgrep` 會比對到這行指令本身而誤報。

## 排錯方法

以下指令都在 `sim` 容器裡跑（`docker compose exec sim bash`）。

- **船的真實位姿**：`gz model -m orca -p`（位置 m，姿態 RPY rad，Gazebo 世界座標 ENU）。
- **把船擺到某個位姿**：
  ```bash
  gz service -s /world/orca_pool/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 2000 --req 'name: "orca", position: {z: -0.3}'
  ```
- **對船施加力矩（模擬被推）**：世界檔沒有載入施力的系統，執行中臨時加：
  ```bash
  gz service -s /world/orca_pool/entity/system/add --reqtype gz.msgs.EntityPlugin_V \
    --reptype gz.msgs.Boolean --timeout 3000 --req 'entity: {id: 1, type: WORLD},
    plugins: {name: "gz::sim::systems::ApplyLinkWrench", filename: "gz-sim-apply-link-wrench-system"}'
  # 繞 z 軸逆時針 40 N·m，持續到清除為止
  gz topic -t /world/orca_pool/wrench/persistent -m gz.msgs.EntityWrench \
    -p 'entity: {name: "orca::base_link", type: LINK}, wrench: {torque: {z: 40}}'
  gz topic -t /world/orca_pool/wrench/clear -m gz.msgs.Entity -p 'name: "orca::base_link", type: LINK'
  ```
- **看飛控內部（控制器的目標與出力）**：MAVLink 遙測只有 4 Hz，而且沒有控制器的內部值。飛控的 DataFlash log 在 `logs/sim/ardusub/logs/*.BIN`，用 pymavlink 讀：
  ```python
  from pymavlink import mavutil
  f = mavutil.mavlink_connection('/logs/ardusub/logs/00000001.BIN')
  while (m := f.recv_match(type=['ATT', 'RATE', 'CTUN', 'BARO'])) is not None:
      print(m)
  ```
  常用的訊息：`ATT`（`DesYaw` 目標航向、`Yaw` 實際航向）、`RATE`（`YDes` 目標角速度、`YOut` 控制器出力）、`CTUN`（`DAlt` 目標深度、`Alt` EKF 深度、`BAlt` 深度計）、`BARO`（原始水壓）。PID 各分量（`PIDY` 等）預設不記，要在解鎖前把 `LOG_BITMASK` 加上第 12 位元（4096），只對這次開機有效。
