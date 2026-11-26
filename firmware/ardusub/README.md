# ArduSub 參數

姿態、定深的控制器和推力分配都在 ArduSub 裡，所以這些參數就是 Orca 的控制器設定，跟程式一樣要進版本控制。

| 檔案 | 內容 |
|---|---|
| `real.parm` | 實機（AET-H743-Basic、ArduSub 4.7.2）的完整參數快照。檔頭記了讀取日期，以及快照之後已知的修改 |
| `sitl.parm` | 模擬用：從實機挑出影響控制行為的參數，加上 SITL 的模擬硬體參數（`SIM_*`）。`deploy/sim` 每次開機都從這份重新載入 |
| `archive/cuav-v5.parm` | 舊飛控 CUAV V5 的最後一份快照（2026-11-19），換飛控後封存 |

格式是 QGC／Mission Planner 的 `.parm`（一行 `NAME VALUE`），可以直接在 QGC 載入。

## 規則

- 改了實機參數（不管是用 QGC 還是 mavros），就重新讀一次存成 `real.parm`，commit 訊息寫改了什麼、為什麼。
- 影響控制行為的修改（`ATC_*`、`PSC_*`、`MOT_*`、`JS_*`、`FS_*` 等）也要同步到 `sitl.parm`，模擬才會跟實機一致。
- 感測器校正值（羅盤、加速度計的 offset）只屬於那一塊飛控，不要帶進 `sitl.parm`。

## AET-H743-Basic

2026-11-26 換上的新飛控，USB 接在 Jetson Orin 上。USB 有兩個序列埠（`…-if00`、`…-if02`）都是 MAVLink，mavros 用 `if00`（`SERIAL0`，對應 `MAV1_*`）。

### 我們改過的參數

裝機的人設好之後，2026-11-26 由我們改了下面 12 個，都改回舊飛控（CUAV V5）上的值。其他參數維持裝機時的設定。

| 參數 | 裝機時 | 改成 | 理由 |
|---|---|---|---|
| `JS_GAIN_DEFAULT` | 0.5 | 1 | `auv_adapter` 假設 effort 1.0 是全力；0.5 時全力只出一半 |
| `MAV1_RAW_SENS`、`MAV1_EXT_STAT`、`MAV1_RC_CHAN` | 2 | 20 | USB 通道的遙測 20 Hz：場次紀錄、系統辨識都照 20 Hz 設計 |
| `MAV1_POSITION`、`MAV1_EXTRA3` | 3 | 20 | 同上 |
| `MAV1_EXTRA1`、`MAV1_EXTRA2` | 10 | 20 | 同上 |
| `MAV1_OPTIONS` | 0 | 4 | 忽略 `REQUEST_DATA_STREAM`，頻率只由上面的參數決定，不會被 QGC 等程式的請求蓋掉 |
| `EK3_ALT_M_NSE` | 10（預設） | 0.1 | 舊飛控上調過的深度量測雜訊，定深才跟之前一致。換了深度計之後可能要重調 |
| `LOG_FILE_DSRMROT` | 0 | 1 | 上鎖時關檔，每次解鎖一個 `.BIN` |
| `LOG_BITMASK` | 176126 | 176127 | 加上 ATTITUDE_FAST，`.BIN` 的 ATT、RATE 從 10 Hz 變 25 Hz |

改的理由的細節（舊飛控時寫的）見 [docs/laptop-rpi-relay.md](../../docs/laptop-rpi-relay.md#紀錄)。

### 還沒處理

- **深度計**：只認到一顆氣壓計（I2C 位址 0x76，讀數是大氣壓）。舊飛控是內建＋外接深度計兩顆、以外接為主（`BARO_PRIMARY 1`）；這塊 `BARO_PRIMARY 0`，`BARO_EXT_BUS 1` 上也沒找到東西。下水前要確認深度計有接上、被認到，並設成主要的那顆。
- **漏水偵測**：`FS_LEAK_ENABLE 1`，但 `LEAK1_PIN -1`（沒指定腳位），實際上沒有作用。
- **`sitl.parm`**：還是照舊飛控挑的。新飛控上有些控制參數跟舊的不同（例如 `ATC_RAT_PIT_I`、`ATC_RAT_RLL_I` 0 → 0.09、`ATC_THR_MIX_MAN` 0.5 → 0.1、`ACRO_YAW_P` 4.5 → 3.375），要不要同步到模擬還沒決定。
