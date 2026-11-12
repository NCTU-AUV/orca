#!/usr/bin/env python3
"""下水測試：定向，以及中途改目標方向（控制來源：自主程式）。

目標航向只在 DEPTH_HOLD 有效，所以先定深在 DEPTH，再依序改目標航向。
流程：歸零（浮在水面）→ 切 DEPTH_HOLD、解鎖、定深、鎖航向 0 → 等 SETTLE_S 秒
      → 依序改目標航向，每個 HOLD_S 秒 → 上鎖（浮上來）

用法（容器內）：python3 /orca/tools/pool_tests/water_heading.py [目標航向 度 ...]
  預設 90 0 -90 0。航向以歸零時為 0，順時針為正，範圍 -180 ~ 180。

- 相鄰兩個目標正好差 180° 時，往哪邊轉不確定（飛控走最短路徑），程式會拒絕。
- 轉向速度上限 60°/s（ATC_RATE_WPY_MAX），轉 90° 至少 1.5 秒。
- 磁力計還沒重新校正，航向可能慢慢飄。

每一段記錄：到達時間（誤差第一次進入 ±5°）、過衝、最後 10 秒的平均誤差和波動，以及深度最大偏離。
任何時候 Ctrl-C：上鎖，船浮上來。
"""
import sys

from water_common import MAX_DEPTH, format_stats, run, step_stats, wrap

DEPTH = 0.15        # 轉向時定在這個深度（m），上限 MAX_DEPTH
SETTLE_S = 15.0     # 下潛後先等深度穩定
HOLD_S = 20.0       # 每個目標航向維持幾秒
TOL = 5.0           # 「到達」的範圍（度）

assert 0 < DEPTH <= MAX_DEPTH
targets = [float(x) for x in sys.argv[1:]] or [90.0, 0.0, -90.0, 0.0]
if not all(-180 <= y <= 180 for y in targets):
    sys.exit('目標航向要在 -180 ~ 180 之間')
for a, b in zip([0.0] + targets, targets):
    if abs(abs(wrap(b - a)) - 180) < 1:
        sys.exit(f'{a:+.0f} → {b:+.0f} 正好差 180°，往哪邊轉不確定，中間加一個目標')


def body(t):
    t.target_yaw = 0.0
    t.use_yaw = True
    if not t.dive(DEPTH):
        return False
    samples = t.hold(SETTLE_S, f'定深 {DEPTH:.2f} m、鎖航向 0°')
    stats = step_stats([(s, h) for s, _, h in samples], 0.0, 0.0, TOL, angle=True)
    t.report.append(format_stats('下潛時鎖 0°', stats, '°'))
    start = 0.0
    for y in targets:
        t.target_yaw = y
        samples = t.hold(HOLD_S, f'目標航向 {y:+.0f}°')
        stats = step_stats([(s, h) for s, _, h in samples], y, start, TOL, angle=True)
        depths = [d for _, d, _ in samples if d is not None]
        depth_dev = max(abs(d - DEPTH) for d in depths) * 100 if depths else float('nan')
        t.report.append(format_stats(f'{start:+.0f} → {y:+.0f}°', stats, '°')
                        + f'，深度最大偏離 {depth_dev:.1f} cm')
        start = y


run('water_heading', body)
