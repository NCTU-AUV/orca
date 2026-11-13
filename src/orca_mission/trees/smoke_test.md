# smoke_test

<!-- 由 render_tree 從 smoke_test.yaml 產生，不要手改 -->

下潛 → 右轉 90° → 往前 → 轉回 0° → 浮上 → 上鎖。中間用 Selector 示範「看到門就停、 沒看到就繼續」：模擬的泳池沒有門，所以會走「沒看到」那一支。

| 變數（-p 覆寫） | 預設 |
|---|---|
| `depth` | `0.3` |
| `surge` | `0.2` |

```mermaid
flowchart TD
  n0["→ 動作測試"]:::seq
  n1{{"⏱ 30 s"}}:::dec
  n0 -->|1| n1
  n2["<b>等就緒（WaitReady）</b>"]:::mission
  n1 --> n2
  n3["<b>檢查清單（Confirm）</b><br/>檢查清單 2 項"]:::mission
  n0 -->|2| n3
  n4["<b>歸零（Zero）</b>"]:::mission
  n0 -->|3| n4
  n5["<b>解鎖（Arm）</b><br/>depth = 0.3"]:::mission
  n0 -->|4| n5
  n6{{"⏱ 150 s"}}:::dec
  n0 -->|5| n6
  n7["→ 下水之後"]:::seq
  n6 --> n7
  n8{{"⏱ 40 s"}}:::dec
  n7 -->|1| n8
  n9(["<b>下潛（Dive）</b><br/>depth = 0.3"]):::task
  n8 --> n9
  n10{{"↻ 最多失敗 2 次"}}:::dec
  n7 -->|2| n10
  n11{{"⏱ 20 s"}}:::dec
  n10 --> n11
  n12(["<b>右轉 90 度（TurnTo）</b><br/>yaw = 90"]):::task
  n11 --> n12
  n13["? 有門就停"]:::sel
  n7 -->|3| n13
  n14["→ 看到門"]:::seq
  n13 -->|1| n14
  n15{"<b>有門嗎（Seen）</b><br/>labels = gate/qualification_gate"}:::cond
  n14 -->|1| n15
  n16(["<b>停住看門（Hold）</b><br/>seconds = 3"]):::task
  n14 -->|2| n16
  n17(["<b>往前（Cruise）</b><br/>seconds = 4"]):::task
  n13 -->|2| n17
  n18{{"⏱ 20 s"}}:::dec
  n7 -->|4| n18
  n19(["<b>轉回 0 度（TurnTo）</b><br/>yaw = 0"]):::task
  n18 --> n19
  n20{{"⏱ 40 s"}}:::dec
  n7 -->|5| n20
  n21(["<b>上浮（Dive）</b><br/>depth = 0.1"]):::task
  n20 --> n21
  n22["<b>上鎖（Finish）</b>"]:::mission
  n7 -->|6| n22
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
