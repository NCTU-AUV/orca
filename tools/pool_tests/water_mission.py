#!/usr/bin/env python3
"""下水測試：小型任務（控制來源：自主程式）。

流程：歸零（浮在水面）→ 切 DEPTH_HOLD、解鎖、下潛到 DEPTH、鎖航向 0 → 等 SETTLE_S 秒
      → 直走 FORWARD_S 秒 → 停 STOP_S 秒
      → 轉 180°（先轉到 +90 再到 180：直接給 180 時往哪邊轉不確定）
      → 往回直走 FORWARD_S 秒 → 停 STOP_S 秒 → 上鎖（浮上來）

用法（容器內）：python3 /orca/tools/pool_tests/water_mission.py
  SURGE、FORWARD_S 依池子大小調整。

記錄：下潛的到達時間和誤差、兩段直走時航向和深度的最大偏離、轉 180° 花的時間和過衝。
任何時候 Ctrl-C：上鎖，船浮上來。
"""
from water_common import MAX_DEPTH, format_stats, run, step_stats, wrap

DEPTH = 0.30        # 任務深度（m），上限 MAX_DEPTH
SETTLE_S = 15.0     # 下潛後先等深度穩定
SURGE = 0.2         # 直走的推力（-1 ~ 1）
FORWARD_S = 10.0     # 每段直走幾秒
STOP_S = 3.0        # 直走後停幾秒
TURN_TIMEOUT = 15.0 # 每一段轉向最多等幾秒
TURN_SETTLE = 5.0   # 轉到 180 之後再穩定幾秒

assert 0 < DEPTH <= MAX_DEPTH


def straight(t, heading, label):
    """以 SURGE 直走 FORWARD_S 秒再停，回報航向和深度的最大偏離。"""
    t.surge = SURGE
    samples = t.hold(FORWARD_S, label)
    t.surge = 0.0
    samples += [(s + FORWARD_S, d, h) for s, d, h in t.hold(STOP_S, f'{label}後停下')]
    headings = [abs(wrap(h - heading)) for _, _, h in samples if h is not None]
    depths = [abs(d - DEPTH) * 100 for _, d, _ in samples if d is not None]
    t.report.append(f'{label}：航向最大偏離 {max(headings, default=float("nan")):.1f}°，'
                    f'深度最大偏離 {max(depths, default=float("nan")):.1f} cm')


def turn(t, target, start, label, settle=0.0):
    """轉到 target，誤差進入 ±10° 就往下走（最多 TURN_TIMEOUT 秒）。

    settle > 0 時再穩定 settle 秒，回報完整統計；否則只回報花多久進入 ±10°。
    """
    t.target_yaw = target
    near = lambda: t.heading() is not None and abs(wrap(t.heading() - target)) < 10.0
    samples = t.hold(TURN_TIMEOUT, label, until=near)
    elapsed = samples[-1][0] if samples else 0.0
    if not settle:
        t.report.append(f'{label}：' + (f'{elapsed:.1f} 秒進入 ±10°' if near()
                                        else f'{TURN_TIMEOUT:.0f} 秒內沒進入 ±10°'))
        return
    samples += [(s + elapsed, d, h) for s, d, h in t.hold(settle, f'{label}，穩定')]
    stats = step_stats([(s, h) for s, _, h in samples], target, start, 5.0, angle=True,
                       final_s=settle)
    t.report.append(format_stats(label, stats, '°', final_s=settle))


def body(t):
    t.target_yaw = 0.0
    t.use_yaw = True
    if not t.dive(DEPTH):
        return False
    samples = t.hold(SETTLE_S, f'下潛到 {DEPTH:.2f} m、鎖航向 0°')
    stats = step_stats([(s, d) for s, d, _ in samples], DEPTH, 0.0, 0.02)
    t.report.append(format_stats(f'下潛 0 → {DEPTH:.2f} m', stats, 'cm', scale=100))

    # straight(t, 0.0, '去程直走')
    # turn(t, 90.0, 0.0, '轉到 +90°')
    # turn(t, 180.0, 90.0, '轉到 180°', settle=TURN_SETTLE)
    # straight(t, 180.0, '回程直走')


run('water_mission', body)
