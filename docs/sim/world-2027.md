# SAUVC 2027 模擬世界

依 **SAUVC 2027 Rulebook 2027.3.2（2026-10-30 更新）** 重建的 Gazebo 世界：資格賽與決賽主場地、像水下的畫面、碰撞與計分事件，以及偵測驗證。需求見 [world-2027-brief.md](world-2027-brief.md)，官方參考圖見 [reference/](reference/README.md)。怎麼跑模擬見 [README.md](README.md)。

指令都在 `deploy/sim/` 底下執行。

```bash
./mission.sh                                # 資格賽世界跑過門的行為樹（預設 WORLD=sauvc_qual.sdf）
WORLD=sauvc_final.sdf GUI=1 ./run.sh        # 只起決賽主場地，開 Gazebo 視窗看
UNDERWATER=0 ./mission.sh                   # 關掉水下效果（A/B 比較）
./referee_check.sh                          # 驗收：每一種規則事件都做得出來、讀得到（約 10 分鐘）
./detect_sweep.sh                           # 偵測驗證：每個道具在多遠偵測得到（約 6 分鐘）
```

## 兩個世界

| 世界 | 內容 | 船的出發位置 |
|---|---|---|
| `sim/worlds/sauvc_qual.sdf` | 50 × 25 m 泳池、三條資格賽道（池壁上 1 m 寬的起點標記、10 m 外的資格賽門）。主場地的道具不放 | 第 2 道的起點標記，船尾貼著牆，船頭朝門 |
| `sim/worlds/sauvc_final.sdf` | 同一個泳池，主場地的全部道具：起始區、導航門、橘色 flare、紅／黃／藍 flare、綠墊與四個 drum、Surfacing Square，加上三條資格賽道的門與標記（場地本來就有，相機也看得到） | 起始區中心、水面，船頭朝導航門 |
| `sim/worlds/orca_pool.sdf` | 舊的空池（25 × 16 m），沒動，動作測試用 | 原點 |

**座標**：每個世界的原點都是船的出發位置（水面 z = 0，往下為負，船頭 +x），所以「船從水面、原點出發」的習慣、飛控歸零、既有程式都不受影響。道具則用另一個座標 `arena` 擺：原點在泳池的角落，x = 0～50 沿長邊（x = 0 是資格賽起點那面牆），y = 0～25 沿短邊，就是 [`arena-2027.png`](reference/arena-2027.png) Top View 的左下角為原點。世界檔裡道具的 pose 都寫 `relative_to="arena"`，可以直接對照官方圖。要換出發位置，改世界檔裡 `<frame name="arena">` 的 pose（世界檔的註解有算法）。

**池底**：照官方側視圖的標註，牆邊 1.28 m 深的平台長 1.8 m，接水平長 17.5 m 的斜坡，x ≥ 19.3 m 是 2.0 m 平底。主場地（x = 22～50）整片是 2.0 m。資格賽門（x = 10）處約 1.62 m。

> rulebook 的圖說寫「資格賽門處 1.28 m」，跟圖上的標註對不上（照標註算是 1.62 m）。2026-11-17 決定照圖上的標註。資格賽門只往下吊 1 m，兩種讀法都碰不到池底。

## 道具

| 道具 | 規則（2027.3.2） | 模擬 | 模型 |
|---|---|---|---|
| 泳池 | 50 × 25 m | 50 × 25 m，牆頂高出水面 0.3 m；磁磚、10 條水道線、牆上的十字照實拍畫 | `pool_2027` |
| 資格賽門 | 寬 150 cm、自水面往下約 100 cm，橘色標記，柱 4 cm | 柱距 150 cm、柱徑 4 cm、長 100 cm，水面上一根白色浮桿（5 cm）。static | `q_gate` |
| 起點標記 | 池壁上 1 m 寬 | 1 m 寬、從水面上 10 cm 到水面下 40 cm 的土黃色標記（高度規則沒寫） | `start_marker` |
| 起始區 | 水面上 140 × 140 cm | 內緣 140 cm 的白色浮框（管徑 4 cm），有碰撞：沒下潛就出不去 | `start_zone` |
| 導航門 | 150 × 150 cm，立在池底，port 紅黑、starboard 綠黑條紋 | 同左，柱徑 4 cm，條紋「色 20／黑 10 cm」，黑色橫桿、白色 PVC 地面框（實拍有）。static | `gate` |
| 橘色 flare | 高約 150 cm、直徑約 15 cm，配重壓底、靠浮力立起 | 浮管 140 cm + 配重盤，萬向接頭加彈簧：撞歪會自己回正 | `orange_flare` |
| 紅／黃／藍 flare | 高約 80 cm、直徑約 1.6 cm，頂端一顆高爾夫球 | 硬桿插在 13 cm 的同色圓底盤上，**沒有固定、沒有浮力**：撞了會晃、會倒、會被推走，不會回正。球擱在 6 顆小球排成的球座上，桿子歪約 10° 球就掉 | `flare_red`、`flare_yellow`、`flare_blue` |
| 綠墊 | 8 × 2 m | 同左，偏黃的亮綠色（實拍）。沒有碰撞（貼著池底，碰到等於碰池底） | `target_mat` |
| drum | 直徑 60 cm、深 30 cm，一藍三紅，一個紅的裝 45 kHz pinger | 上寬下窄的塑膠盆（盆口 60、盆底 52 cm），碰撞是一圈 box，球掉得進去、留得住。pinger 只有外觀 | `drum_blue`、`drum_red`、`drum_red_pinger` |
| 球 | 直徑 3.5～4.5 cm、200 g 以內 | 直徑 4 cm、200 g，掛在船底（電磁鐵），放開就沉 | `dropper_ball` |
| Surfacing Square | 200 × 200 cm 浮框，下方約 30 cm 掛藍色標記 | 外緣 200 cm 的白色浮框，四個角各一根 30 cm 的藍色標記（官方只有側視圖，放在四角是假設）。static | `surfacing_square` |

泳池、水、門、flare、drum 由 [`tools/sim/gen_assets.py`](../../tools/sim/gen_assets.py) 產生（貼圖、mesh、SDF）；要改尺寸、顏色就改那支再重跑，不要手改產生出來的 SDF。

## 擺放與比賽會隨機的東西

位置照 [`arena-2027.png`](reference/arena-2027.png)。官方圖不是按比例畫的，用圖上標的數字推回去：資格賽區 x = 0～10、主場地 x = 22～50（28 m）。

| 道具 | arena 座標 (x, y) | 依據 |
|---|---|---|
| 起始區 | (35.7, 1.2) | 主場地靠起點那一側，離牆約 1.2 m |
| 導航門 | (31.4, 12.0)，兩柱沿 x | 距起始區那面牆約 12 m 的線上 |
| 橘色 flare | (32.6, 6.3) | y = 4～8 m 的帶狀區、門之前 |
| 黃／紅／藍 flare | (28.6, 17.9)、(39.6, 18.2)、(45.6, 16.2) | 門與標靶區之間的帶狀區（y ≈ 14.4～19.3） |
| 綠墊、drum | 墊子中心 (31.1, 22.9)，drum 在 x = 28.1、30.1、32.1、34.1 | 靠主場地另一端，長邊沿泳池長邊 |
| Surfacing Square | (11.0, 8.5) | 貼著資格賽區那一側的邊界 |
| 資格賽門 | (10, 2.6)、(10, 12.0)、(10, 22.6)，起點標記在對應的 (0, y) | 三條道並排在 x = 0～10 |

上表是**固定佈局**（`sauvc_final.sdf` 本身，不設 `SEED` 時用的就是它；驗收的數字都是它）。比賽會隨機的東西有兩種改法：用下面的[隨機佈局](#隨機佈局)，或直接改 `sauvc_final.sdf` 裡標了「隨機」的那幾行：

- **導航門的位置**：`model://gate` 的 x（沿 y = 12 的線移動）。
- **橘色 flare 的位置**：`model://orange_flare` 的 x、y（y 在 4～8）。
- **三支 flare 的位置**：`model://flare_*` 的 x、y。
- **哪個 drum 是藍的、pinger 在哪個紅 drum**：`drum_1`～`drum_4` 四個位置固定，換 `<uri>`（`drum_blue`、`drum_red_pinger`、`drum_red`）。裁判用 uri 決定分數。
- **flare 的指定順序**：`<orca:referee mode="final" flare_order="red blue yellow"/>`。

### 隨機佈局

規則上門、橘色 flare、drum 的顏色與 pinger、flare 的順序在每次 attempt 之間都可能不同。模擬每跑一次 `./mission.sh` 就重開一次 Gazebo（等於一次 attempt），所以設 `SEED` 就在開模擬時照規則抽一個佈局：

```bash
SEED=7 WORLD=sauvc_final.sdf ./mission.sh gate_pass_yaw -p depth=1.0    # 固定用 seed 7 的佈局（同一個 seed 每次都一樣）
SEED=random WORLD=sauvc_final.sdf ./mission.sh gate_pass_yaw -p depth=1.0   # 隨機挑一個 seed（會印出來，之後用它重現）
SEED=7 WORLD=sauvc_final.sdf GUI=1 ./run.sh          # 只起模擬看看這個佈局
SEED=7 ./referee_check.sh sauvc_final.sdf            # 在這個佈局上跑事件的驗收
SEED=7 ./detect_sweep.sh                             # 在這個佈局上做偵測驗證
python3 tools/sim/layout.py sauvc_final.sdf --seed 7 # 只產生世界檔、看佈局，不開模擬
```

- 不設 `SEED` 就是上表的固定佈局，跟以前一樣。目前只有決賽主場地有隨機佈局，其他世界設了 `SEED` 會報錯。
- [`tools/sim/layout.py`](../../tools/sim/layout.py) 以 `sauvc_final.sdf` 為範本，只改下表那幾項，產生 `sim/worlds/generated/sauvc_final_seed<N>.sdf`（不進 git）與旁邊的 `.layout.json`（這次佈局的真值，arena 座標）。裁判會把真值寫進那次紀錄（`logs/referee/` 第一則 `reset` 事件的 `layout`），所以事後查得到 seed 與佈局。
- 只隨機規則明說會變的（2026-11-19 決定）：

| 項目 | 怎麼抽 | 規則 |
|---|---|---|
| 導航門 | 沿 y = 12 m 的線，x 在 23.5～48.5（門柱離主場地邊界至少 0.7 m） | §9.1：平行池邊、距起始區約 12 m 的線上任意位置 |
| 橘色 flare | x 在 22.5～49.5、y 在 4～8 m | §9.1：門之前、離場地側邊 4–8 m 的矩形區 |
| 紅／黃／藍 flare | x 在 22.5～49.5、y 在 14.4～19.3 m，彼此至少隔 2 m | 場地圖的帶狀區。§9.4 的文字是「主場地內任意位置」，先照圖 |
| drum | 四個位置不動，哪個是藍的、pinger 在哪個紅的隨機 | §9.2 |
| flare 的順序 | 紅黃藍的排列 | §9.4；過門之後才告知，見[事件與計分](#事件與計分)的 `/comms/flare_order` |

起始區、綠墊、Surfacing Square、資格賽道、船的出發位置不隨機。道具的方向、浮框漂移、船放進起始區的誤差這類「小擾動」目前沒有做。

驗證（2026-11-19）：`sim/tests/test_layout.py` 500 個 seed 都守規則；`SEED=7 ./referee_check.sh sauvc_final.sdf` 34／34（順序 Y-B-R、第 3 個 drum 是普通紅的）。`gate_pass_yaw -p depth=1.0` 在 seed 7（門在起始區左前方，轉 −20.5° 對準）和 seed 68718（門在右前方，轉 +16.1°）都穿過導航門、沒碰到，過門時 `/comms/flare_order` 發布了 Y-B-R／R-B-Y。

## 事件與計分

`sim_referee` 服務（[`sim/referee.py`](../../sim/referee.py)）讀 Gazebo 的碰撞與真值位姿，照規則發事件：

| ROS topic | 型別 | 內容 |
|---|---|---|
| `/sim/referee/events` | `std_msgs/String`（JSON） | 一個事件一則，latched（晚訂閱也拿得到之前的）。也寫進 `logs/referee/<名稱>.jsonl` |
| `/sim/referee/state` | `std_msgs/String`（JSON） | 2 Hz 的現況：分數、正在碰什麼、碰池底／池壁幾次幾秒、過門了沒、掉了哪些球… |
| `/sim/referee/reset` | `std_msgs/Empty` | 重新開始計分（模擬重開時也會自動重來） |
| `/comms/flare_order` | `std_msgs/String`（latched） | 決賽時船第一次穿過導航門就發布這次的 flare 順序，例如 `R-B-Y`（空字串 = 還沒告知）。扮演「Game Master 告訴隊伍、隊伍用通訊設備傳給 AUV」，**任務可以訂閱這一個**。名稱與型別是暫定的，實機的通訊方式定了改 `sim/referee.py` 的 `FLARE_ORDER_TOPIC` |

**`/sim/*` 只有模擬有，任務程式不要訂閱**（等於偷看答案）。用途是驗證模擬與事後看紀錄；rosbag 會錄到。

```bash
docker compose exec sim_referee bash -ic 'ros2 topic echo /sim/referee/events'
```

事件（每則都有 `t` 模擬時間、`event`、`points` 這一則的分數、`score` 累計，其他欄位依事件而定）：

| 事件 | 規則上的意義 | 怎麼判斷 |
|---|---|---|
| `flare_ball_dropped` | 紅／黃／藍 flare 的球掉下來，+20（過門前不算分） | 球離開球座 5 cm 以上。附 `colour`、`order`、`touched`（船有沒有碰到 flare）、`vehicle_distance` |
| `flare_order_bonus` | 三支都掉了、順序跟 `flare_order` 一樣，+60 | 第三支掉的時候判斷 |
| `flare_order_announced` | 告知 flare 的順序（不計分） | 第一次穿過導航門時，同時發布 `/comms/flare_order` |
| `abort`（`reason: orange_flare`） | 碰到橘色 flare，立即 abort | 船的任何部位碰到它 |
| `penalty_gate` | 碰到導航門，−2（每一次） | 0.5 秒內的斷續接觸算同一次 |
| `penalty_floor_wall` | 碰到池底或池壁，−5（每一次） | 同上；碰池底的常常是掛在船底的球，球還吸在船上時算船的 |
| `abort`（`floor_wall_touches`／`floor_wall_time`） | 池底／池壁累計 5 次，或累計超過 10 秒 | |
| `qualification_failed` | 資格賽：碰到門的任何一邊（兩柱、浮桿）、池底、池壁，或出發時沒碰牆 | 出發前貼著牆不算 |
| `start_wall_touch` | 資格賽：出發時船貼著起點標記範圍內的牆 | 船的外緣離牆 3 cm 內、中心在標記的 1 m 內 |
| `gate_passed` | 決賽 +15；資格賽去程 1 分、迴轉穿回來共 2 分 | 船的中心穿過門的平面、在開口內，再往前走完船身長度。從旁邊、上面、下面過去發 `gate_missed` |
| `ball_released` | 放球機構的球離開船 | |
| `ball_in_drum` | 球停在 drum 裡：藍 30、裝 pinger 的紅 50、其他紅 10（只算第一顆） | 球靜止後在 drum 內側的圓柱範圍內；停在外面發 `ball_missed` |
| `surfaced` | 上浮 +5，這次結束；過門後在 Surfacing Square 內上浮且整趟沒碰到框，再 +20 | 船下潛過、又在水面停 1 秒。附 `inside_square`、`square_touched` |
| `submerged`、`left_start_zone` | 決賽：完全下潛、離開起始區（`submerged: false` 表示沒先下潛就走） | |
| `contact_begin`、`contact_end` | 每一段接觸，含碰到的模型、collision、船的哪個部位 | |
| `run_ended` | 這次結束（`surfaced`、`abort`、`failed`、`completed`），之後的事件 `points` 都是 0 | 資格賽附 `qual_points`（這次的資格賽分數） |

規則沒寫清楚、這裡自己決定的：資格賽過門後上浮當成「結束、保留分數」；「過門之前做其他任務不算分」事件照發但 `points` 是 0、`counted: false`；`left_start_zone` 只回報，不扣分（規則沒有罰則）。

**驗收**：`./referee_check.sh` 把兩個世界各起一次，用外力把船推去撞每一樣東西，檢查事件、分數，以及該擋住的有擋住（門、池壁、起始區的框）、該撞得動的撞得動（flare 被撞倒、橘色 flare 回正）。細節見 [`sim/tests/referee_check.py`](../../sim/tests/referee_check.py)。

## 放球

只做電磁鐵：球用 DetachableJoint 固定在船底，收到指令就放開，自己沉下去（200 g、排水約 34 g）。

| | topic | 型別 |
|---|---|---|
| 任務層 → 模擬 | `/ball_dropper/release` | `std_msgs/Empty` |
| `sim_dropper` → Gazebo | `/orca/ball_dropper/release`（gz） | `gz.msgs.Empty` |

**ROS 這一側的介面是暫定的**（實機的放球機構還沒定案）。實機定了之後改 [`sim/gz_dropper_bridge.py`](../../sim/gz_dropper_bridge.py) 開頭的 `ROS_TOPIC`、`ROS_TYPE` 兩行。放開是一次性的，要再試就重開模擬。

## 水下的畫面

**相機影像**：Harmonic 的 ogre2 沒有 fog，改成在 `sim_cameras` 轉成 ROS 之前做後處理（[`sim/underwater.py`](../../sim/underwater.py)）。每顆彩色相機旁邊有一顆同位置的 depth camera，用每個像素的距離套簡化的水下成像模型：直接光隨距離衰減（紅色先不見）、後向散射讓遠處趨近水體的藍、距離越遠越模糊、加一點噪點。topic 名稱、`frame_id`、影像大小與編碼、`camera_info` 都不變。

預設值對著隊上的實拍（`ml/rfdetr` 驗證集）與官方照片 `gate+flare-2022.jpg` 調：近處的顏色對隊上實拍，遠處往官方照片的藍靠；10 m 外剩約 6 成對比，25 m 外的對面池壁還看得到。量法與數字見 `underwater.py` 的檔頭。

**Gazebo 視窗**：世界裡的 `model://water`，水面的底面是自發光的波紋（相機在水下往上看也是它），頂面是半透明的水色，水體裡是一層很淡的大顆 particle 疊出「越遠越藍」。彩色相機用 visibility mask 把 particle 排除，所以水體霧只影響視窗，不會跟相機的後處理重複。

**開關**：

| | 世界的水（水面、水體霧） | 相機影像的後處理 |
|---|---|---|
| `UNDERWATER=0` | 不載入（`sim/start.sh` 把世界檔的 `UNDERWATER:BEGIN`～`END` 拿掉） | 關，影像原封不動 |
| `UNDERWATER_STRENGTH=0.5`（預設 1） | — | 強度：衰減、散射、模糊、噪點一起乘 |
| 執行中 | — | `docker compose exec sim_cameras bash -ic 'ros2 param set /sim_cameras underwater false'`（`underwater_strength` 也可以改） |

## 偵測驗證

`./detect_sweep.sh` 用現有的 RF-DETR 權重（`ml/rfdetr/weights/`，8 類）跑模擬畫面：

1. `sim_sweep`（[`tools/sim_detect/capture.py`](../../tools/sim_detect/capture.py)）自己起一個 gz server（不需要 ArduSub，也不跟正在跑的模擬衝突），在決賽世界裡放一組跟船上前視相機規格一樣的相機，擺到每個道具前面的固定位姿：距離 1～20 m × 角度 −45°／0°／45° × 深度 0.5／1.0／1.5 m，相機水平、對著道具。每個位姿拍「水下效果開」（世界有水 + 後處理）與「關」（移除水、不處理）各一張，並用 segmentation camera 算真值框（扣掉被遮住的部分）。
2. `detect_eval`（[`tools/sim_detect/evaluate.py`](../../tools/sim_detect/evaluate.py)）用跟實機偵測節點同一套 rfdetr 與權重推論，算每個道具的偵測率、平均信心、最遠偵測距離，依距離、深度、角度細分，以及被認成什麼。
3. 結果在 `logs/detect/<RUN>/summary.md`（另有 `summary.json`、`results.csv`、`samples/<道具>.jpg`）。兩次比較：`python3 tools/sim_detect/compare.py logs/detect/<A> logs/detect/<B>`。換了權重只要重跑第 2 步（`evaluate.py` 的檔頭有指令）。

### 結果（2026-11-18，`sweep_20261118`）

權重 `rfdetr-v2`（`checkpoint_best_total.pth`），門檻 0.4，764 個位姿 × 水下效果關／開。偵測率只算「看得到」的影像（真值框 ≥ 20 px）。

| 道具 | 偵測率（關 → 開） | 平均信心（關 → 開） | 最遠偵測距離 m（關 → 開） | 偵測率都 ≥ 50% 的距離 m（關 → 開） |
|---|---|---|---|---|
| 導航門（150 cm 高） | 93% → 94% | 0.93 → 0.88 | 15 → 15 | 2～15 → 2～15 |
| 資格賽門（吊掛 100 cm） | 52% → 92% | 0.82 → 0.88 | 10 → 12 | 2～10 → 2～12 |
| 資格賽門，認成 `gate` 也算 | 92% → 92% | | | |
| 橘色 flare | 100% → 60% | 0.81 → 0.64 | 20 → 10 | 1～20 → 1～8 |
| 紅 flare | 87% → 39% | 0.62 → 0.63 | 12 → 5 | 1～12 → 1～4 |
| 黃 flare | 84% → 64% | 0.64 → 0.62 | 12 → 8 | 1～12 → 1～5 |
| 藍 flare | 98% → 73% | 0.70 → 0.68 | 12 → 8 | 1～12 → 1～6 |
| 紅 drum | 100% → 92% | 0.94 → 0.87 | 20 → 20 | 1～20 → 1～15 |
| 藍 drum | 99% → 99% | 0.90 → 0.82 | 20 → 20 | 1～20 → 1～20 |
| Surfacing Square | 0% → 0%（沒有這一類） | | | |

讀法與要注意的地方：

- **門的新外形沒有讓偵測變差**：導航門從 100 cm 變 150 cm、資格賽門從落地變吊掛，模型（用 2026 外形訓練）在 2～15 m 都穩定偵測得到。1 m 時門超出畫面，偵測率低是正常的。資格賽門在乾淨畫面常被認成 `gate`（25 張），過門的行為樹兩類都接受，所以不影響。
- **細的 flare 受水下效果影響最大**：紅 flare 開效果後 5 m 外幾乎認不到（紅光衰減最多，1.6 cm 的桿子在遠處只剩一兩個像素又被模糊）。黃、藍約 5～6 m。要撞 flare 的任務要假設 5 m 內才看得到。
- **橘色 flare 開效果後有 29 張被認成 `yellow_flare`**：模擬裡橘色浮管在陰影面本來就偏暗，加上紅光衰減，顏色往黃綠偏。官方實拍裡近距離的橘色 flare 仍是鮮橘色，所以這可能是模擬誇大了，要用隊上的實拍確認。如果實機也會這樣，任務層不能只靠類別分辨橘色（碰到就 abort）與黃 flare（要撞的目標）。
- **Surfacing Square 在乾淨畫面會被認成 `gate`**（21 張，10 m 外分數到 0.85）。偵測模型沒有它與藍色標記這一類，用視覺找 bonus 的位置目前做不到。
- drum 從 20 m 外都偵測得到（前視相機、水平看過去；太近時 drum 在畫面下緣之外，不列入）。
- 模擬的道具外觀、光線、水況都只是近似，這些數字適合拿來比較（換權重、換水下參數之後變好還是變壞），不代表實機的偵測距離。

## 驗收紀錄

2026-11-17～18，RTX 3060 Laptop，brief 的六項驗收：

| # | 項目 | 結果 |
|---|---|---|
| 1 | 兩個世界開得起來，沒有 SDF 解析錯誤、沒有缺貼圖或 mesh 的警告；`GUI=0` 也能跑 | 通過。下面所有測試都是 `GUI=0`；`GUI=1` 用 Xvfb 開過視窗截圖。log 裡唯一的警告是水體霧的三行 `ParticleEmitter SetColorRange is currently disabled`（Gazebo 的限制，見 [notes.md](notes.md)） |
| 2 | 船從起始區出發姿態穩定，浮力沒被改壞 | 通過。[`float_check.py`](../../sim/tests/float_check.py)：水面靜置深度 舊空池 −0.2101 m／資格賽 −0.2104／決賽 −0.2118（多了 200 g 的球），傾角 ≤ 0.1°、不漂；壓到 0.8 m 放開，上浮速度 0.183／0.177／0.125 m/s，自己浮回水面、傾角 ≤ 0.33°。決賽慢是因為球讓淨浮力從約 5.7 N 降到 4.1 N，比例相符。過門任務全程深度 0.29～0.31 m、航向 ±0.0° |
| 3 | 行為要求的每個事件都做得出來、讀得到 | 通過。`./referee_check.sh`：決賽 32／32、資格賽 13／13 項 |
| 4 | 水下效果開與關的影像明顯不同，`camera_info` 完全相同 | 通過。[`camera_ab_check.py`](../../sim/tests/camera_ab_check.py)（資格賽世界）：前視、底部的 `camera_info`、`frame_id`、影像格式完全相同；開與關平均每個像素差 46.5／40.8（8-bit），同一條件前後幀的差是 0 |
| 5 | 偵測驗證跑得完並產出結果檔 | 通過。`./detect_sweep.sh`：1528 張，拍攝約 4 分鐘、推論約 1 分鐘，結果見[上面](#結果2026-11-18sweep_20261118) |
| 6 | `./mission.sh smoke_test` 等既有流程不壞 | 通過。`WORLD=orca_pool.sdf ./mission.sh smoke_test`：行為樹 SUCCESS。`./mission.sh`（gate_pass，資格賽世界，水下效果開）：行為樹 SUCCESS，9.6 m 外就認得到資格賽門；裁判記到「出發碰牆 → 下潛 → 去程穿過資格賽門（離中心 0.8 cm、沒碰到）→ 上浮」，1 分。合併 `feat/gate-blind-after-lost`（盲走到完全看不到門、`gate_pass_yaw`）後重跑：`gate_pass`、`gate_pass_yaw`、`smoke_test` 都 SUCCESS，兩棵過門樹離門中心都不到 1 cm、沒碰到 |

gate_pass 的 `depth` 是飛控歸零後的深度（船浮著時為 0），船的原點實際在水面下 `depth + 0.21` m：預設 0.3 時船在 0.51 m，正好在資格賽門開口（0～1 m）的中間。決賽的導航門橫桿在水面下約 0.54 m，船要在 `depth` 約 0.6～1.6 之間才從門框裡過（建議 1.0）；用預設 0.3 會從橫桿上方過去，裁判會發 `gate_missed`。

## 已知限制

- 門、資格賽門、Surfacing Square、起始區的框是 static：撞到會擋住，但不會像實際的吊掛／浮框那樣被撞得晃或漂開。
- 橘色 flare 用彈簧代替浮力，模擬裡沒有水流，所以它靜止時完全直立，不會隨水流擺動。
- 紅／黃／藍 flare 的質量與底盤大小是估的（扣掉浮力的等效值）。
- pinger 只有外觀，沒有模擬聲學。
- 水下效果是後處理，不是真的光學模擬：沒有水面的光紋投在池底（caustics）、沒有懸浮顆粒。
- Gazebo 視窗的水下感是近似：有水面、遠處被水體霧藍化，但 2 m 內的東西不會偏藍（Gazebo 沒有 fog）。看相機實際拍到的畫面要看 ROS 影像（`/detections/image`、Foxglove）。
- 偵測模型沒有 Surfacing Square 這一類，也沒有它下方的藍色標記。
