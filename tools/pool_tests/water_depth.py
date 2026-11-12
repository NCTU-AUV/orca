#!/usr/bin/env python3
"""下水測試：定深，以及中途改目標深度（控制來源：自主程式）。

流程：歸零（浮在水面）→ 切 DEPTH_HOLD、解鎖 → 依序定在各個目標深度，每個 HOLD_S 秒 → 上鎖（浮上來）
航向不給目標，飛控會鎖住解鎖當下的航向。

用法（容器內）：python3 /orca/tools/pool_tests/water_depth.py [目標深度 m ...]
  預設 0.15 0.20 0.10。深度以歸零時（浮著）為 0，上限 0.25（池子最深）。

每一段記錄：到達時間（誤差第一次進入 ±2 cm）、過衝、最後 10 秒的平均誤差和波動。
對照 11-06 傍晚的定深測試：穩定後誤差約 ±0.5 cm、波動約 1 cm，10～15 秒才收斂。
任何時候 Ctrl-C：上鎖，船浮上來。
"""
import sys

from water_common import MAX_DEPTH, format_stats, run, step_stats

HOLD_S = 30.0       # 每個目標維持幾秒
TOL = 0.02          # 「到達」的範圍（m）

targets = [float(x) for x in sys.argv[1:]] or [0.15, 0.20, 0.10]
if not all(0 < d <= MAX_DEPTH for d in targets):
    sys.exit(f'目標深度要在 0 ~ {MAX_DEPTH} m 之間')


def body(t):
    if not t.dive(targets[0]):
        return False
    start = 0.0
    for d in targets:
        t.target_depth = d
        samples = t.hold(HOLD_S, f'定深 {d:.2f} m')
        stats = step_stats([(s, depth) for s, depth, _ in samples], d, start, TOL)
        t.report.append(format_stats(f'{start:.2f} → {d:.2f} m', stats, 'cm', scale=100))
        start = d


run('water_depth', body)
