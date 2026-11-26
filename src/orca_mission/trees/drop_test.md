# drop_test

<!-- 由 render_tree 從 drop_test.yaml 產生，不要手改 -->

歸零（浮在水面）→ 解鎖 → 下潛 → 停 hold_s 秒 → 放球 → 停 → 上鎖。

| 變數（-p 覆寫） | 預設 |
|---|---|
| `depth` | `0.5` |
| `hold_s` | `3` |
| `ready_timeout` | `60` |

```mermaid
flowchart TD
  n0["→ 水下放球測試"]:::seq
  n1{{"⏱ 60 s"}}:::dec
  n0 -->|1| n1
  n2["<b>等就緒（WaitReady）</b><br/>dropper = True"]:::mission
  n1 --> n2
  n3["<b>檢查清單（Confirm）</b><br/>檢查清單 4 項"]:::mission
  n0 -->|2| n3
  n4["<b>歸零（Zero）</b>"]:::mission
  n0 -->|3| n4
  n5["<b>解鎖（Arm）</b><br/>depth = 0.5"]:::mission
  n0 -->|4| n5
  n6{{"⏱ 90 s"}}:::dec
  n0 -->|5| n6
  n7["→ 下水之後"]:::seq
  n6 --> n7
  n8{{"⏱ 40 s"}}:::dec
  n7 -->|1| n8
  n9(["<b>下潛（Dive）</b><br/>depth = 0.5<br/>tol = 0.05"]):::task
  n8 --> n9
  n10(["<b>停（Hold）</b><br/>seconds = 3"]):::task
  n7 -->|2| n10
  n11["<b>放球（DropBall）</b><br/>wait_s = 3"]:::mission
  n7 -->|3| n11
  n12(["<b>放球後停（Hold）</b>"]):::task
  n7 -->|4| n12
  n13["<b>上鎖（Finish）</b>"]:::mission
  n7 -->|5| n13
  classDef seq fill:#dbeafe,stroke:#1d4ed8,color:#0f172a
  classDef sel fill:#fef3c7,stroke:#b45309,color:#0f172a
  classDef par fill:#ede9fe,stroke:#6d28d9,color:#0f172a
  classDef dec fill:#f1f5f9,stroke:#64748b,color:#0f172a
  classDef task fill:#dcfce7,stroke:#15803d,color:#0f172a
  classDef mission fill:#fee2e2,stroke:#b91c1c,color:#0f172a
  classDef cond fill:#fff,stroke:#0f172a,color:#0f172a
```

圖例：→ Sequence（依序，一個失敗就失敗）、? Selector（依序嘗試，一個成功就成功）、
⇉ Parallel、⏱ 逾時、↻ 重試；綠色圓角 = 執行器（tasks/，經 TaskRunner 控制推進器）、
紅色 = 任務層節點（就緒檢查、確認、歸零、解鎖、上鎖）、菱形 = 條件。
邊上的數字是執行順序。樹結束（成功或失敗）時一律 MANUAL、全 0、DISARM。
