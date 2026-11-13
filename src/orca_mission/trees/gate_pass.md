# gate_pass

<!-- 由 render_tree 從 gate_pass.yaml 產生，不要手改 -->

歸零（浮在水面）→ 解鎖 → 下潛 → 原地轉向對準門 → 鎖航向前進、左右只用 sway 修 → 門夠近（或近距離跟丟）→ 盲走 pass_s 秒 → 停 → 上鎖。各步的參數跟 mission/gate_pass.py 相同。

| 變數（-p 覆寫） | 預設 |
|---|---|
| `depth` | `0.3` |
| `surge` | `0.2` |
| `pass_s` | `5` |
| `labels` | `gate/qualification_gate` |
| `min_score` | `0.5` |
| `ready_timeout` | `60` |

```mermaid
flowchart TD
  n0["→ 過門"]:::seq
  n1{{"⏱ 60 s"}}:::dec
  n0 -->|1| n1
  n2["<b>等就緒（WaitReady）</b><br/>cameras = front<br/>detections = True"]:::mission
  n1 --> n2
  n3["<b>檢查清單（Confirm）</b><br/>檢查清單 5 項"]:::mission
  n0 -->|2| n3
  n4["<b>歸零（Zero）</b>"]:::mission
  n0 -->|3| n4
  n5["<b>解鎖（Arm）</b><br/>depth = 0.3"]:::mission
  n0 -->|4| n5
  n6{{"⏱ 180 s"}}:::dec
  n0 -->|5| n6
  n7["→ 下水之後"]:::seq
  n6 --> n7
  n8{{"⏱ 40 s"}}:::dec
  n7 -->|1| n8
  n9(["<b>下潛（Dive）</b><br/>depth = 0.3"]):::task
  n8 --> n9
  n10(["<b>對準門（AlignTo）</b>"]):::task
  n7 -->|2| n10
  n11(["<b>接近門（ApproachGate）</b>"]):::task
  n7 -->|3| n11
  n12(["<b>盲走過門（Cruise）</b><br/>seconds = 5"]):::task
  n7 -->|4| n12
  n13(["<b>停（Hold）</b>"]):::task
  n7 -->|5| n13
  n14["<b>上鎖（Finish）</b>"]:::mission
  n7 -->|6| n14
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
