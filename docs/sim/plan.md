# Orca ArduSub SITL + Gazebo 模擬：計畫

> 這份是建立模擬時的紀錄（2026-11-10），寫的是搬進 orca repo 之前的檔案配置。對照：
> `adapter/` → `src/orca_bridge/`（程式、mavros 設定）與 `tools/pool_tests/`（測試腳本）；
> `sim/params/orca.parm` → `firmware/ardusub/sitl.parm`；`sim/Dockerfile` → `docker/sim.Dockerfile`；
> `run.sh`、`compose.yaml` → `deploy/sim/`；compose 的 `adapter` 服務 → `bridge`；`logs/adapter/` → `deploy/sim/logs/bridge/`。


## 目標

在一台有 NVIDIA 顯示卡的 Ubuntu 電腦上，用 `./run.sh` 一個指令起出：

- Gazebo Harmonic 的泳池場景與 Orca 載具；
- 跑在 SITL 的 ArduSub（版本與實機飛控相同），由 Gazebo 提供物理；
- 與實機相同的 mavproxy → mavros → `auv_adapter.py` 指令鏈，使實機的測試腳本（`water_depth.py`、`water_heading.py` 等）不必修改就能對模擬執行。

原則：

- **最小可運行**：只做讓上面這條鏈跑起來所需的東西。感測器、相機、比賽道具、水下成像、水動力校準都不在這次範圍。
- **可擴充**：結構上讓之後加相機、ROS 橋接、比賽道具、其他 ROS 服務時，只需要新增檔案或服務，不必重寫現有部分。

## 架構

```
QGroundControl（主機上，選用） ◀── UDP 14550 ──┐
                                                │
[adapter 服務]  ros:humble                      │
    auv_adapter.py + mavros                     │
    fcu_url = udp://127.0.0.1:14551@            │
                     ▲ UDP 14551                │
[sim 服務]  ros:humble + Gazebo Harmonic，NVIDIA GPU
    mavproxy ───────────────────────────────────┘   分送 MAVLink，角色同實機 RPi 上的 mavproxy.service
       │ TCP 5760
    ardusub（Sub-4.7.2，--model JSON，載入 orca.parm）
       │ JSON over UDP 9002
    gz sim（泳池 + orca 模型 + ArduPilotPlugin）
```

- 物理全在 Gazebo。Gazebo 模型只放一顆 IMU：ArduPilotPlugin 必須有它，送給 SITL 的陀螺儀與加速度取自這顆 IMU；磁力計、深度計由 SITL 從 Gazebo 回傳的位姿自行模擬。
- Gazebo 與 SITL 以 lockstep 同步：Gazebo 變慢時整個模擬一起變慢，控制行為不失真。
- 兩個服務都用 host network，並設 `ROS_LOCALHOST_ONLY=1`，避免同一網段上的其他 ROS 系統互相干擾。

## 檔案配置

```
orca-sim/
├── README.md            需求、怎麼跑、已知問題
├── NOTES.md             開發過程遇到的坑、不直觀的設定、排錯方法
├── PLAN.md              本文件
├── run.sh               xhost 授權 → 檢查 nvidia runtime → docker compose up
├── compose.yaml         sim、adapter 兩個服務
├── adapter/             auv_adapter.py、orca_msgs、測試腳本、mavros.yaml、Dockerfile
│   └── CHANGES.md       與實機版 adapter 的差異
├── sim/
│   ├── Dockerfile       ros:humble + gz-harmonic + ardupilot_gazebo + ArduSub Sub-4.7.2 + MAVProxy
│   ├── start.sh         依序啟動 gz sim、ardusub、mavproxy
│   ├── params/orca.parm
│   ├── worlds/orca_pool.sdf
│   └── models/
│       ├── pool/        泳池地板與貼圖
│       └── orca/        船體、浮力、阻力、8 顆推進器、ArduPilotPlugin
└── logs/                adapter 的 /logs 與 ArduSub log（不納入版本控制）
```

## 各部分規格

### 版本

| 元件 | 版本 |
|---|---|
| 容器基底 | `ros:humble`（Ubuntu 22.04） |
| Gazebo | Harmonic（OSRF apt） |
| ArduSub | `Sub-4.7.2` tag（與實機飛控相同） |
| ArduPilot Gazebo 外掛 | [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) `main`，`GZ_VERSION=harmonic` |
| mavros | `ros-humble-mavros` |

### 世界：`orca_pool.sdf`

- 取自 [NCTU-AUV/SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation) 的 `water_world.sdf` 與 `pool_ground` 模型：25 × 16 m、深 1.6 m 的泳池，水面在 z = 0，池壁、地板貼圖。
- 該 repo 是 Gazebo Fortress（Ignition）時期的寫法，外掛檔名與命名空間改為 Harmonic 的 `gz-sim-*-system` / `gz::sim::systems::*`。
- 保留 Physics、UserCommands、SceneBroadcaster、Buoyancy（graded：z > 0 為空氣）。
- 不放比賽道具。泳池與道具分開，之後道具以 `<include>` 加入。

### 載具：`models/orca`

- 幾何沿用 SAUVC-Simulation 的 `orca_auv`：

  | 模型推進器 | 用途 | 位置（x 前、y 左，m） | 方向 |
  |---|---|---|---|
  | 0、1、2、3 | 垂直 | (0.15, 0.237)、(0.15, −0.237)、(−0.15, 0.237)、(−0.15, −0.237) | 推力沿 z |
  | 4、5、6、7 | 水平 | (0.2965, 0.213)、(0.2965, −0.213)、(−0.2965, 0.213)、(−0.2965, −0.213) | 與 x 軸夾 45° |

- 質量、慣量、浮力體、Hydrodynamics 阻力係數沿用原模型；不放相機、高度計，IMU 只放 ArduPilotPlugin 需要的那一顆。
- 用 Harmonic 的 Thruster 系統接收推力指令，ArduPilotPlugin 把 SITL 的 8 路 PWM（1100–1900 µs，中位 1500）換成推力。
- 座標轉換照 ardupilot_gazebo 範例：模型 FLU → FRD、世界 ENU → NED。
- 相機之後以獨立的附掛模型加入，由另一個世界檔（例如 `orca_pool_cameras.sdf`）載入，不改動本模型。

### 馬達對應

ArduSub `VECTORED_6DOF` 的馬達 1–4 為水平、5–8 為垂直。依 `AP_Motors6DOF.cpp` 的混控係數推得、經 M2 逐軸實測確認的對應：

| ArduSub 馬達 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| 位置 | 右前 | 左前 | 右後 | 左後 | 右前 | 左前 | 右後 | 左後 |
| 模型推進器 | 5 | 4 | 7 | 6 | 1 | 0 | 3 | 2 |

`MOT_n_DIRECTION` 照實機設定。實機的方向值用來補償螺旋槳正反轉與接線，模擬沒有這些差異，因此在外掛的推力倍率乘上相同的正負號抵銷。這樣 PWM 的意義、測試腳本裡的 `VERT_SIGN`、`YAW_SIGN` 都與實機一致。

### SITL 參數：`orca.parm`

只收錄影響控制行為的參數，值取自實機飛控（2026-11-06 設定）：

| 類別 | 參數 |
|---|---|
| 機架與馬達 | `FRAME_CONFIG` = 2；`MOT_1~8_DIRECTION` = −1, 1, −1, 1, 1, −1, −1, 1；`MOT_PWM_MIN/MAX` = 1100/1900；`MOT_THST_*`、`MOT_SPIN_*` |
| 搖桿 | `JS_GAIN_DEFAULT` = 1.0 |
| 姿態 | `ATC_*`，其中 `ATC_ANG_RLL_P` = 6、`ATC_RAT_RLL_I` = 0、`ATC_RAT_PIT_I` = 0；`ACRO_YAW_P` |
| 定深 | `PSC_D_*` |
| Failsafe | `FS_PILOT_INPUT` = 2、`FS_PILOT_TIMEOUT` = 3；`FS_GCS_ENABLE` = 2、`FS_GCS_TIMEOUT` = 5 |
| 地面站 | `MAV_GCS_SYSID` = 255 |

實體感測器的校正值（`AHRS_ORIENTATION`、`INS_*OFFS`、`COMPASS_OFS_*`、`BARO_EXT_BUS`）與 `BATT_*` 不帶入；解鎖前檢查用 SITL 預設。

另有幾個 SITL 模擬硬體的參數（`SIM_*`，不是飛控參數），讓模擬的感測器與實機一致：

| 參數 | 值 | 理由 |
|---|---|---|
| `SIM_GPS1_ENABLE` | 0 | 實機沒有 GPS；EKF 與實機一樣沒有水平位置 |
| `SIM_OPOS_ALT` | 0 | ArduSub 的模擬深度計把起始海拔直接當水深換算水壓，非 0 會算出錯誤的水壓，ALT_HOLD 失效 |
| `SIM_BARO_RND`、`SIM_BAR2_RND` | 0.0003 | 深度計雜訊 ±0.3 mm，同實機 Bar02；預設 0.2 m 是空中載具氣壓計的值 |

### adapter

- `adapter/` 是實機版 adapter 的複本，Dockerfile、`orca_msgs`、`auv_adapter.py`、測試腳本都在裡面。
- `mavros.yaml` 的 `fcu_url` 已是 `udp://127.0.0.1:14551@`，sim 服務的 mavproxy 往這個埠送，不必改。
- 任何修改都記在 `adapter/CHANGES.md`，以便回推到實機。

### `run.sh` 與 `compose.yaml`

- `run.sh`：`xhost +SI:localuser:root` → 確認 `docker info` 有 `nvidia` runtime，沒有就印出安裝方式並結束 → `docker compose up --build`。
- sim 服務：`gpus: all`、`NVIDIA_DRIVER_CAPABILITIES=all`、`__NV_PRIME_RENDER_OFFLOAD=1`、`__GLX_VENDOR_LIBRARY_NAME=nvidia`、掛 `/tmp/.X11-unix`、傳 `DISPLAY`。
- `start.sh` 以環境變數決定是否開 GUI（預設開）。
- adapter 服務：host network、`ROS_LOCALHOST_ONLY=1`；`adapter/` 唯讀掛到 `/work`，`logs/adapter/` 掛到 `/logs`；容器內同時跑 mavros 與 `auv_adapter.py`，測試程式用 `docker compose exec adapter` 執行。
- 之後的服務（ROS 橋接、水下成像、視覺、任務層）以新增 compose 服務的方式加入，都用 Humble。

## 里程碑

| # | 內容 | 驗收 |
|---|---|---|
| M0 | 可行性 | `ros:humble` 容器帶上述 GPU 設定，`glxinfo -B` 顯示 NVIDIA；EGL 算圖可用；`ros-humble-ros-gzharmonic` 在 apt 可解析 |
| M1 | Gazebo | `orca_pool.sdf` 載入；船靜止時浮著且保持正立；RTF 約 1 |
| M2 | SITL ↔ Gazebo | 收到 HEARTBEAT、EKF 正常；MANUAL 解鎖後前後、左右、上下、yaw 各推一次，方向全部正確；馬達對應表定稿 |
| M3 | 內迴路 | STABILIZE 被推偏後回到原航向；ALT_HOLD 定在 0.5 m 深，記錄誤差 |
| M4 | adapter | adapter 服務連上，`/auv_adapter/zero` 後 `water_depth.py`、`water_heading.py` 行為與實機紀錄大致相符 |
| M5 | 交付 | README 完成；從乾淨狀態 `./run.sh` 重跑一次 M4 |

## 已知限制

- 推進器位置沿用 SAUVC-Simulation 的模型，可能與目前船體不同。
- 質量、浮力、阻力是估計值，M3、M4 只驗證行為正確，不驗證數值。
- SITL 用海水密度（1024 kg/m³）算水壓，飛控用 9800 Pa/m 換回深度，飛控讀到的深度比 Gazebo 真值多約 2.5%。
- Humble + Harmonic 不是 ROS 官方預設組合（預設是 Fortress），ROS 橋接要用 `ros-humble-ros-gzharmonic`。
- ArduPilot 建置需要 `empy` 3.x，不能用 4.x。
