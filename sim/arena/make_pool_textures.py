#!/usr/bin/env python3
"""產生 sauvc27_pool 的貼圖：地板三段（淺端平台、斜坡、深區）和池壁。

    python3 sim/arena/make_pool_textures.py [--mm-per-px 15]

需要 numpy、Pillow。pool_tile_base.png 是 4 × 4 m 一塊、可無縫拼接的磁磚（16 × 16 片、
每片 25 cm），取自 SAUVC-Simulation 的 pool_ground（縮小到 512 px）。這裡把它鋪滿整個泳池，
再把奧運池的標線畫在真實的位置上：水道線（沿 x，每 2.5 m 一條，寬 25 cm），
離兩端池壁 2 m 處的 T 字橫線（長 1 m）。池壁頂端是池邊（淺色）加一條深藍色水線帶。

Gazebo 的 plane 把貼圖 UV 0..1 鋪滿整個 plane：u 沿 +x（影像第 0 欄在 -x 端），
v 沿 -y（影像第 0 列在 +y 端）。所以影像就是「從上往下看、+y 朝上」的俯視圖，
每一段地板用自己那一段的影像。尺寸跟 sauvc2027.py 共用同一組常數。
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sauvc2027 as A  # noqa: E402

TILE_SPAN_M = 4.0
LINE_W = 0.25
T_FROM_WALL = 2.0
T_LEN = 1.0
GUTTER_M = 0.10
WATERLINE_M = 0.06

NAVY = np.array([26, 52, 104], np.uint8)
GUTTER = np.array([232, 236, 238], np.uint8)
WATERLINE = np.array([44, 74, 126], np.uint8)


def tiled(base, h_m, w_m, ppm):
    patch_px = max(2, int(round(TILE_SPAN_M * ppm)))
    patch = np.asarray(base.resize((patch_px, patch_px), Image.LANCZOS), np.uint8)
    h, w = int(round(h_m * ppm)), int(round(w_m * ppm))
    return np.tile(patch, (h // patch_px + 1, w // patch_px + 1, 1))[:h, :w].copy()


def floor_top_view(base, ppm):
    """整個池底的俯視圖：第 0 列 = y 最大（遠側池壁），第 0 欄 = x = 0（淺端池壁）。"""
    img = tiled(base, A.POOL_W, A.POOL_L, ppm)
    h, w = img.shape[:2]
    ys = A.POOL_W - (np.arange(h) + 0.5) / ppm
    xs = (np.arange(w) + 0.5) / ppm
    in_x = (xs >= T_FROM_WALL) & (xs <= A.POOL_L - T_FROM_WALL)
    for k in range(int(A.POOL_W // A.LANE_W)):
        yc = (k + 0.5) * A.LANE_W
        rows = np.abs(ys - yc) < LINE_W / 2
        img[np.ix_(rows, in_x)] = NAVY
        t_rows = np.abs(ys - yc) < T_LEN / 2
        for xe in (T_FROM_WALL, A.POOL_L - T_FROM_WALL):
            img[np.ix_(t_rows, np.abs(xs - xe) < LINE_W / 2)] = NAVY
    return img


def wall(base, length_m, ppm):
    """池壁：第 0 列是池壁頂端。"""
    height = A.DECK + A.DEEP_DEPTH + 0.1
    img = tiled(base, height, length_m, ppm)
    from_top = (np.arange(img.shape[0]) + 0.5) / ppm
    img[from_top < GUTTER_M] = GUTTER
    img[(from_top >= GUTTER_M) & (from_top < GUTTER_M + WATERLINE_M)] = WATERLINE
    return img


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mm-per-px', type=float, default=15.0)
    ap.add_argument('--base', default=os.path.join(here, 'pool_tile_base.png'))
    ap.add_argument('--out', default=os.path.join(A.MODELS, 'sauvc27_pool', 'materials', 'textures'))
    a = ap.parse_args()
    ppm = 1000.0 / a.mm_per_px
    base = Image.open(a.base).convert('RGB')
    os.makedirs(a.out, exist_ok=True)

    top = floor_top_view(base, ppm)
    x_slope_end = A.SHALLOW_LEN + A.SLOPE_LEN
    col = lambda x: int(round(x * ppm))  # noqa: E731
    images = {
        'floor_shallow': top[:, :col(A.SHALLOW_LEN)],
        'floor_slope': top[:, col(A.SHALLOW_LEN):col(x_slope_end)],
        'floor_deep': top[:, col(x_slope_end):],
        'wall_long': wall(base, A.POOL_L + 2 * A.WALL_T, ppm),
        'wall_short': wall(base, A.POOL_W, ppm),
    }
    for name, arr in images.items():
        path = os.path.join(a.out, f'{name}.png')
        Image.fromarray(arr).save(path, optimize=True)
        print(f'{os.path.relpath(path, A.SIM)}  {arr.shape[1]}x{arr.shape[0]}  {os.path.getsize(path) / 1e6:.2f} MB')


if __name__ == '__main__':
    main()
