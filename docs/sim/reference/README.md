# SAUVC 官方參考圖

重建模擬世界時照著這些圖做，不要憑印象。對應需求見 [../world-2027-brief.md](../world-2027-brief.md)。

**來源**：SAUVC 2027 Rulebook，版本 2027.3.2（2026-10-30 更新），<http://sauvc.org/rulebook/>，取自該頁的 `img/`。
**版權**：主辦單位 IEEE OES / The Singapore AUV Challenge 所有，照片上有浮水印。放在這裡只為了隊內建模參考，不要另外散布或改作。

## 尺寸圖（2027 現行規格）

| 檔案 | 內容 |
|---|---|
| `arena-2027.png` | **最重要的一張。** 場地 Top + Side View：三條資格賽道、主場地 28 m、起始區、門的擺放線、橘 flare 的 4–8 m 區域、三支 flare 的帶狀區、8 × 2 m 標靶區與四個 drum、Surfacing Square，以及池底從 1.28 m 斜到 2 m 的側視圖 |
| `gate-2024.jpg` | 導航門正視圖：寬 150 × 高 150 cm，立在約 200 cm 深的水裡，左柱紅黑條紋、右柱綠黑條紋 |
| `qualification-gate.jpg` | 資格賽門側視圖：水面上的白色浮桿，兩根橘色柱（寬 4 cm）往下約 100 cm，柱距 150 cm |
| `flares-2024.jpg` | 紅／黃／藍 flare 側視圖：高約 80 cm、桿徑約 1.6 cm，**插在平底盤上立於池底**，頂端擱一顆高爾夫球 |
| `orange-flare.jpg` | 橘色 flare 側視圖：高約 150 cm、約 15 cm 寬，立在約 200 cm 深的水裡 |
| `surfacing-square-2027.png` | Surfacing Square：200 × 200 cm 浮框，下方約 30 cm 掛藍色標記（2027 新增） |

## 實拍照片（外觀、材質、水況的依據）

| 檔案 | 內容 |
|---|---|
| `orange-flare-2022.jpg` | 橘色 flare 水下實拍。**看得出它是充氣浮管，底部用一塊配重盤壓在池底** — 跟紅黃藍那種硬桿是完全不同的東西 |
| `gate+flare-2022.jpg` | 導航門與橘色 flare 的水下實拍。門是白色 PVC 地面框加兩根立柱與黑色橫桿。**也是池水清澈度的主要依據**：能見度可以看到對面池壁，遠處是被藍化變淡，不是混濁 |
| `gate-2022.jpg` | 導航門水下實拍 |
| `arena-top-2022.jpg` | 2022 年場地俯拍，看得到水道與道具配置 |
| `qualification-2022.jpg` | 2022 年資格賽道俯視，三條道各一個門 |
| `starting-zone-2022.jpg` | 隊員把 AUV 放進水面起始區浮框 |
| `drums-2017.jpg` | 標靶區 drum 的水下實拍 |
| `pinger.jpg` | RJE 45 kHz 水下聲學信標（裝在其中一個紅 drum 裡） |

## 隊上的水下實拍影片（不在 repo 裡）

兩段自己錄的水下影片，檔案大（共約 700 MB），沒有放進版本控制，只記位置：

```
/home/huang/Downloads/GX010255.MP4
/home/huang/Downloads/GX010256.MP4
```

GoPro 錄的泳池水下畫面，拍到 flare、導航門與橘色 flare 的實際樣子與水況，是比上面任何一張官方照片都接近我們實際環境的依據。

這是本機路徑，clone repo 的人看不到；之後搬到隊上共享的地方再回來更新這一段。

## 注意

- 實拍照片是 2017–2024 年的，**尺寸一律以 2027 的尺寸圖與 rulebook 文字為準**，照片只用來看外觀、材質、顏色與水況。
- `sim/models/gate/model.sdf`、`q_gate/model.sdf` 的註解裡提到的 `docs/sim-visual/reference/...` 是更早的路徑，那個目錄不在 repo 裡；要找參考圖就看這裡。
- 隊上自己的實拍（`ml/rfdetr/` 訓練集）是另一個依據，顏色與底盤樣式以實拍為準。
