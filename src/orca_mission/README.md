# orca_mission：行為樹決策層

偵測結果加上飛控狀態，換算成 `/auv_control`。整個任務流程寫在 `trees/*.yaml`，旁邊有自動產生的流程圖（`trees/*.md`，GitHub、VS Code 都能直接顯示）。

| 樹 | 內容 | 流程圖 |
|---|---|---|
| `gate_pass` | 過門：航向從解鎖起就鎖住（歸零時的船頭方向），對準只用 sway 平移 | [gate_pass.md](trees/gate_pass.md) |
| `gate_pass_yaw` | 過門：下潛後先原地轉 yaw 對準門、鎖住當下的航向，之後跟 `gate_pass` 一樣 | [gate_pass_yaw.md](trees/gate_pass_yaw.md) |
| `smoke_test` | 動作測試（下潛、轉向、前進、上浮），不需要相機 | [smoke_test.md](trees/smoke_test.md) |

## 跑

平常用 `deploy/laptop/mission.sh`、`deploy/sim/mission.sh` 跑，指令見 [README 的「日常指令」](../../README.md#日常指令)，模擬的其他選項（`smoke_test`、隨機佈局等）見 [docs/sim/README.md](../../docs/sim/README.md#跑行為樹missionsh)。兩個腳本都是把參數交給容器裡的 `mission`：

```
mission <樹> [-p 名稱=值 ...] [--no-checklist] [--print-tree] [--run-name 名稱]
```

- `-p` 覆寫樹的 `vars`，例如 `-p depth=0.5 -p pass_s=6`。
- `--no-checklist`：不列檢查清單、不等 Enter（只在模擬用）。`--print-tree`：只印出樹的結構，不執行。`--run-name`：CSV 的檔名與標記裡的名稱，預設 `<樹>_<時間>`。

無論樹成功或失敗、流程中飛控自己上鎖、程式出錯、Ctrl-C、Ctrl-Z、`docker stop`，都會送 MANUAL、全 0、DISARM，並等飛控上鎖。adapter 的看門狗在 0.5 秒沒收到 `/auv_control` 時只會改送中位，不切模式、不上鎖（ALT_HOLD 下船會停在水裡維持深度），所以上鎖一定要由這裡做。

上鎖之後會等船浮上水面（深度 ≤ 0.05 m 持續 1 秒，最多 30 秒）才結束，CSV 跟著記到上浮的過程。rosbag 也錄到上浮為止：模擬的 `mission.sh` 在 `mission` 結束後才停止錄影，實機是全程錄影、結束的標記在上浮之後才發。等的時候再按一次 Ctrl-C 就不等；`docker stop` 不等。

## 看狀態

- 終端機每 0.5 秒印一行：目前的節點、深度、航向、指令、執行器的回饋。
- `/mission/status`（`std_msgs/String`，JSON，2 Hz）。
- 即時看整棵樹每個節點的狀態：`docker compose --profile mission run --rm mission py-trees-tree-watcher`。
- CSV（20 Hz）：實機在 `deploy/laptop/logs/session/<場次>/mission/<樹>_<時間>.csv`，模擬在 `deploy/sim/logs/mission/<樹>_<時間>.csv`（`--run-name` 可以指定檔名）。
- 標記（`/session/marker`）：開始時一筆，帶程式版本（`ORCA_GIT`，`mission.sh` 傳進來）；結束（上浮後）一筆，帶結果。兩筆都含 run 的名稱。
- rosbag（MCAP，Foxglove 直接開）：模擬每次 `deploy/sim/mission.sh` 錄一個，在 `deploy/sim/logs/bag/<樹>_<時間>/`，跟 CSV 同名，影像只有畫好框的 JPEG；實機是場次的全程錄影，用 `./slice.sh --match <樹>_<時間>` 切出這次任務（見 [docs/laptop-rpi-relay.md](../../docs/laptop-rpi-relay.md#紀錄)）。

## 架構

```
/mavros/state、rel_alt、imu/data ─> VehicleState ─┐
/detections + 各相機 camera_info ─> Perception ───┴─> Snapshot（每次 tick 一份，唯讀）
                                                         │
        行為樹 10 Hz（任務層：順序、重試、逾時、條件、就緒檢查、歸零、解鎖）
                                                         │ start／status／cancel
        TaskRunner 20 Hz（一次只跑一個執行器，經 Shaper 限幅、限轉速）──> /auv_control
```

| 檔案 | 內容 |
|---|---|
| `orca_mission/node.py` | ROS 節點：訂閱、計時器、收尾（`mission` 指令） |
| `orca_mission/vehicle.py` | `VehicleState`（mavros → 深度、航向；自己記零點；航向歷史內插）、`Command`、`Shaper` |
| `orca_mission/perception.py` | `/detections` → `Sighting`：方位角、仰角、偏移、框寬、拍攝當下的航向、絕對方位；`last_seen` |
| `orca_mission/task_runner.py` | `TaskRunner`：`/auv_control` 唯一的主人 |
| `orca_mission/tasks/` | 執行器：`Hold`、`Dive`、`TurnTo`、`Cruise`（placeholder），`AlignTo`、`ApproachGate`（過門） |
| `orca_mission/bt/behaviours.py` | 行為樹的葉節點：`TaskAction`（包執行器），`WaitReady`、`Confirm`、`Zero`、`Arm`、`Finish`、`Seen` |
| `orca_mission/bt/loader.py` | YAML → py_trees，載入時檢查（節點種類、參數名稱、必填、型別） |
| `orca_mission/bt/render.py` | YAML → Mermaid 流程圖（`render_tree` 指令） |
| `config/orca.yaml` | 相機（frame_id、camera_info topic、安裝角度）、指令上限 |
| `test/` | 單元測試，以及用平面模型跑完整過門流程的測試（不需要 ROS） |

規則：

- 控制只經 `/auv_control`，mavros 的 topic 只讀，`orca_bridge`（adapter）不改。
- 模式固定 DEPTH_HOLD，`use_depth_target`、`use_yaw_target` 全程 true，不切回 yaw 轉速。原因是切回去之後的 5 秒內，飛控對 yaw 輸入沒有反應。
- 目標航向每秒最多變 `limits.yaw_rate` 度（預設 30°/s）。大角度轉向會自動拆成小步，所以不會出現「相鄰兩筆目標差 180°、飛控不知道往哪邊轉」的情形。
- 執行器只決定目標深度、目標航向和正規化的 surge／sway，閉迴路交給飛控。歸零、解鎖、切模式只有任務層節點能做。
- 執行器結束後，保持它最後的目標深度和航向，推力歸 0。

## 寫一棵樹

格式說明在 [`orca_mission/bt/loader.py`](orca_mission/bt/loader.py) 開頭，可用的節點和參數看 `tasks/`、`bt/behaviours.py`。重點：

- 組合節點：`Sequence`、`Selector`（`memory` 預設 true）、`Parallel`（`policy: all | one`）。裝飾節點：`Timeout`、`Retry`、`Inverter`、`ForceSuccess`。
- 任何節點都可以加 `timeout: 秒`、`retry: 次數`（每次重試各自計時）、`name`。
- `vars` 裡的值用 `$名稱` 引用，指令列可以用 `-p` 覆寫。
- 參數名字寫錯、缺了必填參數，載入時就會報錯並指出在樹裡的哪個位置。

改完 YAML 要重新產生流程圖，CI 會檢查兩者是否一致：

```bash
docker compose --profile mission run --rm mission render_tree                      # 寫 trees/*.md
docker compose --profile mission run --rm mission render_tree gate_pass --svg /logs/gate_pass.svg
```

新增執行器：在 `tasks/` 寫一個 `Task` 子類別（`Params` dataclass 加上 `step()`），然後加進 `tasks/__init__.py` 的 `TASKS`。

## 測試

```bash
cd src/orca_mission && python3 -m pytest test        # 要有 py_trees；或在 mission 容器裡跑
```

## 已知限制

- **遙測只有約 4 Hz**（經 mavproxy）。影像的絕對方位用航向歷史內插；轉向中內插誤差會變大，所以要用絕對方位時，先讓船停止轉向再量。對準（`AlignTo`）用的是「拍攝當下的航向 + 方位角」。
- **水平方向沒有定位**（沒有 DVL、GPS），前進距離只能用時間估（`Cruise`）。
- **內參是空氣中的**：平面視窗在水下會把角度放大約 1.33 倍。閉迴路會收斂，只是等效增益變小。
- **模擬比實機容易**：畫面乾淨，偵測比實機穩定，只適合驗證流程和正負號。
- **底部相機**：模擬有，實機還沒接。RPi 轉接沒有這一路；Jetson 上用 driver 發 `/bottom_camera/image_raw`、`camera_info`（frame_id `bottom_camera_optical_frame`）即可。
