# orca_actuators：致動器

目前只有放球的電磁鐵。電磁鐵的驅動模組（MOSFET 或繼電器）接 Jetson 40-pin 的一支 GPIO，
數位訊號，通電吸住、斷電放開。

| 檔案 | 內容 |
|---|---|
| `orca_actuators/gpio.py` | 40-pin 的一支 GPIO 輸出（Linux GPIO character device），不 import ROS、不需要 libgpiod |
| `orca_actuators/ball_dropper.py` | `ball_dropper` 節點：ROS 介面 → GPIO |
| `config/ball_dropper.yaml` | 腳位、極性、啟動時吸住 |
| [`tools/hw_tests/magnet.py`](../../tools/hw_tests/magnet.py) | 岸上的吸放測試，直接在 Jetson 主機上跑（不經 ROS） |

## 介面

| 名稱 | 型別 | 內容 |
|---|---|---|
| `/ball_dropper/set_hold` | `std_srvs/SetBool` | `true` 吸住、`false` 放開。腳位設好才回覆；`success: false` 表示沒做到 |
| `/ball_dropper/holding` | `std_msgs/Bool` | 目前是不是吸住。變化時馬上發、之後每秒一次，transient local |

每次改變都插一筆標記（`/session/marker`）。模擬的 `sim_dropper`（[sim/gz_dropper_bridge.py](../../sim/gz_dropper_bridge.py)）
提供同一套介面，任務層（`orca_mission` 的 `DropBall`、`WaitReady dropper: true`）不用分實機、模擬。

手動操作（`fc_adapter` 這類有 ROS 的容器裡）：

```bash
ros2 service call /ball_dropper/set_hold std_srvs/srv/SetBool "{data: false}"   # 放開
ros2 topic echo --once /ball_dropper/holding
```

## 腳位

建議用 **pin 29（PQ.05）**，備用 pin 31（PQ.06）：這兩支在開發套件的腳位圖上就是一般 GPIO，沒有跟
UART、SPI、I2S、PWM 共用。`python3 tools/hw_tests/magnet.py pins` 列出全部可用的腳和目前狀態。

- Jetson 的 GPIO 是 3.3 V、只能推幾 mA，一定要經過驅動模組，模組的輸入要吃得下 3.3 V 邏輯；線圈要有飛輪二極體。
- 驅動模組的輸入要有下拉（或上拉）電阻：Jetson 開機、節點還沒起來的時候腳位是輸入（浮接），沒有電阻的話電磁鐵的狀態不確定。
- JetPack 6 有些腳的 pinmux 預設不是 GPIO，要在 Jetson 螢幕前跑 `sudo /opt/nvidia/jetson-io/jetson-io.py` 設定（要 sudo）。
  `gpio.py` 讀回來的是「要求輸出的值」，不是腳上的電壓，第一次一定要用電錶量。

接線、極性、放掉腳位後的狀態都用 `magnet.py` 實測（步驟在它的檔頭），確定之後改 `config/ball_dropper.yaml`，
結果記到 `docs/test-logs/`。

## 在 Jetson 上跑

`ball_dropper` 要在 Jetson 的 compose 裡當常駐服務（`restart: unless-stopped`，任務中別重開：節點結束時只放掉腳位，
放掉之後的電位沒有保證）。容器要看得到 `/dev/gpiochip0`：

```yaml
  ball_dropper:
    <<: [*ros, *base]
    restart: unless-stopped
    devices:
      - /dev/gpiochip0
    group_add: ["999"]            # 主機 gpio 群組的 GID（getent group gpio），容器以 root 跑時可省略
    volumes:
      - /etc/localtime:/etc/localtime:ro
      - ../../src:/opt/orca_ws/src:ro
    command: [ros2, run, orca_actuators, ball_dropper, --ros-args, --params-file,
              /opt/orca_ws/src/orca_actuators/config/ball_dropper.yaml]
```

`magnet.py` 和 `ball_dropper` 不能同時用同一支腳（第二個會拿到 EBUSY）：岸上測試前先 `docker compose stop ball_dropper`。
