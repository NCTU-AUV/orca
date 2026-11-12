# ArduSub 參數

姿態、定深的控制器和推力分配都在 ArduSub 裡，所以這些參數就是 Orca 的控制器設定，跟程式一樣要進版本控制。

| 檔案 | 內容 |
|---|---|
| `real.parm` | 實機（CUAV V5、ArduSub 4.7.2）的完整參數快照。檔頭記了讀取日期，以及快照之後已知的修改 |
| `sitl.parm` | 模擬用：從實機挑出影響控制行為的參數，加上 SITL 的模擬硬體參數（`SIM_*`）。`deploy/sim` 每次開機都從這份重新載入 |

格式是 QGC／Mission Planner 的 `.parm`（一行 `NAME VALUE`），可以直接在 QGC 載入。

## 規則

- 改了實機參數（不管是用 QGC 還是 mavros），就重新讀一次存成 `real.parm`，commit 訊息寫改了什麼、為什麼。
- 影響控制行為的修改（`ATC_*`、`PSC_*`、`MOT_*`、`JS_*`、`FS_*` 等）也要同步到 `sitl.parm`，模擬才會跟實機一致。
- 感測器校正值（羅盤、加速度計的 offset）只屬於那一塊飛控，不要帶進 `sitl.parm`。
