# SAUVC 2027 模擬場地

依 [SAUVC 2027 規則書](https://sauvc.org/rulebook/) 做的兩個世界，資格賽和決賽分開：

| 世界 | 內容 | 船的起點 |
|---|---|---|
| `sim/worlds/sauvc27_qualification.sdf` | 三條資格賽道：池壁上的起點標記、10 m 外的資格賽門 | Arena 2 的起點，船尾貼牆、船頭朝門（+x） |
| `sim/worlds/sauvc27_finals.sdf` | 決賽主場地：導航門、橘色 flare、target zone 與四個桶、紅黃藍 flare、Surfacing Square、出發區（資格賽的門也在池裡） | 出發區水面，船頭朝 +y |

兩個世界的船都是 `model://orca_cam`（前視、底部相機），所以 `deploy/sim/run.sh`、`mission.sh` 都會自動起相機和偵測。

```bash
cd deploy/sim
WORLD=sauvc27_finals.sdf ./run.sh -d                                 # 只起決賽環境，不跑任務
WORLD=sauvc27_qualification.sdf ./run.sh -d                          # 只起資格賽環境
WORLD=sauvc27_qualification.sdf ./mission.sh gate_pass               # 資格賽：過資格賽門
WORLD=sauvc27_finals.sdf ./mission.sh gate_pass -p depth=1.0         # 決賽：過導航門
WORLD=generated/sauvc27_finals_seed7.sdf ./mission.sh gate_pass -p depth=1.0   # 隨機佈局（先產生，見下面）
```

## 座標

兩個世界共用：原點在**淺端、出發區那一側的池角水面**。x 沿 50 m 長邊（0 = 淺端池壁，資格賽起點那面牆），y 沿 25 m 短邊（0 = 決賽出發區那一側），z 向上（水面 0）。規則書的距離（「離池邊 4–8 m」「約 12 m 的線」）都是 y，可以直接對照。

```
y=25 ┌──────────┬─────────────────────────────────────────────────────┐
     │ ▮──── ⊓  │              ▔▔▔ target zone（綠墊 8×2 m，4 桶）▔▔▔   │
     │ Arena 3  │                                                     │
     │          │       ● 黃            ● 紅               ● 藍 flare  │
     │ ▮──── ⊓  │                                                     │
y=12 │ Arena 2  │       ━━━━━━━━━━ 導航門可在這條線上任意 x ━━━━━━━━━━   │
     │      ┌─┐ │                                                     │
     │      └─┘Surfacing Square      ◍ 橘色 flare（y 4–8 m 帶狀區）    │
     │ ▮──── ⊓  │                               ▢ 出發區              │
y=0  └──────────┴─────────────────────────────────────────────────────┘
     x=0      x=10        x=22（主場地 22–50 m，深區水深 2 m）        x=50
     淺端 1.28 m ─ 斜坡（x 1.8 → 19.3 m）→ 2.0 m
```

## 尺寸與出處

| 項目 | 模擬 | 規則書 |
|---|---|---|
| 泳池 | 50 × 25 m；淺端 1.28 m 深、平台 1.8 m，斜坡水平 17.5 m，之後 2.0 m | §8 Figure 5 側視圖 |
| 資格賽門 | 寬 1.5 m、從水面垂下 1.0 m、柱寬 4 cm 橘色、頂上白色浮桿；x = 10 m | §8.1、Figure 8 |
| 資格賽起點 | 池壁上 1 m 寬的標記；三條賽道 y = 2.5、12.5、22.5 m | §8.1、Figure 5 |
| 導航門 | 1.5 × 1.5 m、立在池底；左舷紅黑、右舷綠黑條紋、黑色橫桿；y = 12 m 線上 | §9.1、Figure 11 |
| 橘色 flare | 1.5 m 高、約 15 cm 寬；y 4–8 m 帶狀區、x 在主場地內 | §9.1、Figure 12 |
| Target zone | 綠色墊 8 × 2 m，中心 (31, 23) | §9.2、Figure 5 |
| 桶 | 直徑 60 cm、深 30 cm、開口朝上；一藍三紅，其中一個紅桶有 pinger | §9.2 |
| 紅黃藍 flare | 80 cm 高、直徑 1.6 cm，頂上一顆高爾夫球（動態，撞到會掉） | §9.4、Figure 17 |
| Surfacing Square | 2 × 2 m 浮框，四角掛藍色標記 30 cm 深；中心 (11.5, 7) | §9.5、Figure 18 |
| 出發區 | 1.4 × 1.4 m 水面浮框；中心 (36, 2) | §8.2 |

規則書說「誤差 ±5%」，場地圖標「不按比例」，所以位置是近似值。

### 規則書沒寫清楚、這裡自己假設的

- **主場地的 x 範圍**：圖上標 28 m，假設是深端的最後 28 m（x 22–50）。
- **出發區、target zone、flare 的位置**：從場地圖量的近似值。
- **導航門的線**：文字寫「離出發區約 12 m」，圖上標的是「離池邊約 12 m」。這裡用圖上的（y = 12 m；出發區中心在 y = 2 m，所以離出發區約 10 m）。
- **紅黃藍 flare 的範圍**：文字寫「主場地內任意位置」，隨機佈局只在圖上畫的帶狀區（y 13.5–20.5 m）裡抽。
- **4 個桶**在墊子上沿長邊等距 2 m 排一列。
- **Surfacing Square 的藍色標記**：側視圖只看得到兩根，假設四個角各一根。
- **導航門的白色 PVC 底座**：規則書圖沒畫，比賽照片（2022）上有，偵測模型是用照片訓練的，所以保留。
- **池壁高出水面 0.4 m**（沿用 2026 模擬）。

## 跟 2026 模擬（SAUVC-Simulation）不同的地方

| | 2026 模擬 | 2027 |
|---|---|---|
| 泳池 | 25 × 16 m、平底 1.6 m | 50 × 25 m、淺端 1.28 m 斜到 2.0 m |
| 前進方向 | 沿長邊（x） | 決賽沿短邊（+y）；資格賽沿長邊（+x） |
| 資格賽門 | 從水面一直到池底、白色 | 從水面垂下約 1 m、橘色 |
| 導航門 | 1.0 m 高 | 1.5 m 高（池底 2 m 深，橫桿在水下 0.5 m） |
| 導航門的位置 | 離起點牆 16 m | 離池邊約 12 m |
| 新道具 | — | Surfacing Square |
| flare 桿子 | 視覺放大 5 倍（偵測用） | 照規則書 1.6 cm |

## 隨機佈局

比賽時門的位置、橘色 flare、桶的順序、pinger、flare 位置和撞擊順序都會變。產生一組：

```bash
python3 sim/arena/sauvc2027.py finals --seed 7         # → sim/worlds/generated/sauvc27_finals_seed7.sdf
python3 sim/arena/sauvc2027.py qualification --lane 3  # 船換到 Arena 3
```

`sim/worlds/generated/` 不進 git。同一個 seed 一定產生同一個場地。

## 真值（layout.json）

每個世界旁邊有 `<世界>.layout.json`：每個道具的模型、位置、yaw，船的起點，決賽另外有 flare 順序、哪個桶有 pinger。之後要寫裁判或評分程式（過門判定、碰撞、浮出水面的位置）可以直接讀它，配合 Gazebo 的 `/world/<世界>/dynamic_pose/info`（船、flare、高爾夫球的即時位置）。

## 改場地

所有尺寸都在 [`sim/arena/sauvc2027.py`](../../sim/arena/sauvc2027.py) 開頭的常數。改完重新產生：

```bash
python3 sim/arena/sauvc2027.py              # 模型 sim/models/sauvc27_* 和兩個預設世界
python3 sim/arena/make_pool_textures.py     # 泳池尺寸或標線改了才要（需要 numpy、Pillow）
```

產生出來的 `model.sdf`、世界檔不要手改，下次產生會被蓋掉。模型掛載進容器，改完 `docker compose up -d --force-recreate simulator` 就生效，不用重 build。

## 實測（2026-11-13，`GUI=0`，整套流程：Gazebo + ArduSub SITL + fc_adapter + sim_cameras + RF-DETR + orca_mission）

| 世界 | 指令 | 行為樹 | Gazebo 真值 |
|---|---|---|---|
| `sauvc27_qualification.sdf` | `./mission.sh gate_pass` | SUCCESS；門的分數 0.93～0.95，對準後鎖航向 −0.1° | 穿過門平面時離門中心 1.7 cm，z = −0.51 m（門的範圍 0 ～ −1.0 m）；全程左右偏離不到 6 cm |
| `sauvc27_finals.sdf` | `./mission.sh gate_pass -p depth=1.0` | SUCCESS；門在出發時的左前方約 24°，對準後鎖航向 −24.1° | 穿過門時離門中心 11 cm，z = −1.19 m（開口 −2.0 ～ −0.5 m）；離橘色 flare 最近 1.69 m |
| `generated/sauvc27_finals_seed7.sdf` | 同上 | SUCCESS；鎖航向 −23.4° | 離門中心 4.5 cm；三顆高爾夫球全程留在桿頂 |

- 決賽要用 `-p depth=1.0`：導航門的橫桿在水下 0.5 m，`gate_pass` 預設的 0.3 m 會從橫桿上面過去。
- `gate_pass` 只過一次門。資格賽要的「過門 → U-turn → 反向再過一次」還沒有對應的行為樹。
- `AlignTo` 不會自己轉頭找門：門一開始就要在前視相機的視野內（水平 ±27.9°）。預設佈局和 seed 7 剛好都在視野內（約 24°），其他 seed 不一定。
- flare：單獨測過不碰時球停在桿頂 30 秒不動；在 flare 半高處施加 20 N、0.3 秒的側向力，球會掉到池底。

## 限制

- **沒有水下成像**：沒有衰減、色偏、浮游物，偵測比實機容易，也看得比實機遠。
- **沒有裁判**：過門、碰到門／池底／池壁、浮出水面在不在 Surfacing Square 裡，都還沒有自動判定。
- **pinger 只有外觀**，沒有聲學訊號。
- **沒有投球／撿球機構**：桶是空心的，可以放球，但船還沒有投放器的模型。
- **flare 撞擊順序**不會傳給船（比賽時是完成導航後由隊伍用通訊設備告知），只寫在 layout.json。
- **flare 沒有水的阻力**：被撞後會在池底滑得比實際遠（上面的測試滑了約 1.6 m）。
- **相機在水面附近時，上半部看到的是空氣**（天空色背景），因為沒有水面的模型。下潛之後就正常。
