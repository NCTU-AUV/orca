#!/usr/bin/env python3
"""產生 SAUVC 2027 模擬世界的貼圖、mesh，以及重複性高的模型 SDF。

    python3 tools/sim/gen_assets.py          # 在 repo 根目錄執行，主機要有 numpy、opencv-python

輸出都在 sim/models/ 底下，而且都進版本控制：平常不用跑這支，改了下面的尺寸或顏色才重跑。
由這支產生的 SDF 檔頭都有「由 tools/sim/gen_assets.py 產生」，要改請改這裡再重跑，不要手改。

    pool_2027/         50 x 25 m 泳池：池底三段（淺區平台、斜坡、深區）、四面牆、貼圖
    water/             水面（底面的波紋、頂面的半透明水色）與 Gazebo 視窗用的水體霧（particle）
    gate/              導航門（紅黑／綠黑條紋柱）
    flare_red|yellow|blue/   硬桿 flare ＋ 頂端的高爾夫球
    drum_red|blue|red_pinger/   drum（mesh ＋ 一圈 box 的碰撞）

其他道具（q_gate、orange_flare、target_mat、surfacing_square、start_zone、start_marker、
dropper_ball）形狀簡單，SDF 是手寫的。

尺寸依據：SAUVC 2027 Rulebook 2027.3.2 與 docs/sim/reference/ 的官方圖；規則沒寫的（磁磚大小、
水道線、drum 的錐度、flare 底盤）照實拍照片估，各段註解有標。
"""
import math
import os

import cv2
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'sim', 'models')
HEADER = '由 tools/sim/gen_assets.py 產生，不要手改：改那支再重跑'

# ---------------------------------------------------------------- 泳池（規則值）
POOL_X, POOL_Y = 50.0, 25.0          # olympic，x = 長邊（x = 0 是資格賽起點那面牆）
SHELF_X, SHELF_DEPTH = 1.8, 1.28     # 牆邊的淺區平台（arena-2027.png 側視圖的標註）
SLOPE_X = 17.5                       # 斜坡的水平長度
DEEP_DEPTH = 2.0                     # 主場地
WALL_T, WALL_TOP, WALL_BOTTOM = 0.3, 0.3, -2.3      # 牆厚；牆頂高出水面 0.3 m
SLAB_T = 0.5                         # 池底碰撞板的厚度

# 外觀（規則沒寫，照 docs/sim/reference 的實拍）
PX_PER_M = 80                        # 貼圖解析度
TILE_X, TILE_Y = 0.25, 0.125         # 磁磚
LANE_W = 0.25                        # 水道線寬
LANE_PITCH = 2.5                     # 10 條水道
LANE_END = 2.0                       # 水道線離兩端牆 2 m，尾端有 T 字
TILE_RGB = (0.86, 0.93, 0.95)
GROUT_RGB = (0.60, 0.70, 0.74)
LANE_RGB = (0.04, 0.09, 0.28)
DECK_RGB = (0.78, 0.80, 0.80)
TRIM_RGB = (0.06, 0.16, 0.45)

WATER_RGB = (0.03, 0.42, 0.72)       # 遠處水體的顏色（gate+flare-2022.jpg 的遠景）


def bgr8(rgb):
    return tuple(int(round(255 * c)) for c in reversed(rgb))


def save(path, img):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f'  {os.path.relpath(path, ROOT)}  {img.shape[1]}x{img.shape[0]}')


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)
    print(f'  {os.path.relpath(path, ROOT)}')


def model_config(name, desc):
    return f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
'''


def mat(rgb, spec=0.05, extra=''):
    r, g, b = rgb
    return (f'<material><ambient>{r:.3g} {g:.3g} {b:.3g} 1</ambient><diffuse>{r:.3g} {g:.3g} {b:.3g} 1</diffuse>'
            f'<specular>{spec} {spec} {spec} 1</specular>{extra}</material>')


def tex_mat(uri, rough=0.6):
    return (f'<material><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular><pbr><metal>'
            f'<albedo_map>{uri}</albedo_map><metalness>0</metalness><roughness>{rough}</roughness>'
            f'</metal></pbr></material>')


# ---------------------------------------------------------------- 貼圖
def tiles(w_m, h_m, x0_m=0.0, y0_m=0.0):
    """w_m x h_m 的磁磚底圖（BGR）。x0_m、y0_m 是這塊左下角在泳池座標的位置，磁磚縫才接得起來。"""
    w, h = int(round(w_m * PX_PER_M)), int(round(h_m * PX_PER_M))
    img = np.empty((h, w, 3), np.uint8)
    img[:] = bgr8(TILE_RGB)
    tx, ty = int(TILE_X * PX_PER_M), int(TILE_Y * PX_PER_M)
    cols = (np.arange(w) + int(round(x0_m * PX_PER_M))) % tx == 0
    rows = (np.arange(h)[::-1] + int(round(y0_m * PX_PER_M))) % ty == 0     # 影像上方 = +y
    img[rows, :] = bgr8(GROUT_RGB)
    img[:, cols] = bgr8(GROUT_RGB)
    return img


def shade(img, seed, amp=0.035, cell_m=3.0):
    """乘上一層低頻的明暗起伏，磁磚才不會整片一樣平。"""
    rng = np.random.default_rng(seed)
    h, w = img.shape[:2]
    gh, gw = max(2, int(h / PX_PER_M / cell_m) + 2), max(2, int(w / PX_PER_M / cell_m) + 2)
    noise = cv2.resize(rng.normal(0, 1, (gh, gw)).astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
    out = img.astype(np.float32) * (1 + amp * noise)[..., None]
    return np.clip(out, 0, 255).astype(np.uint8)


def pool_floor_textures(out_dir):
    """整個池底畫成一張（x 向右、+y 向上），再切成淺區、斜坡、深區三塊。
    <plane> visual 的 UV：影像左→右 = +x，影像上方 = +y（實測）。"""
    full = tiles(POOL_X, POOL_Y)
    h = full.shape[0]

    def px(m):
        return int(round(m * PX_PER_M))

    for k in range(int(POOL_Y / LANE_PITCH)):
        yc = LANE_PITCH / 2 + k * LANE_PITCH
        r0, r1 = h - px(yc + LANE_W / 2), h - px(yc - LANE_W / 2)
        full[r0:r1, px(LANE_END):px(POOL_X - LANE_END)] = bgr8(LANE_RGB)
        for xe in (LANE_END, POOL_X - LANE_END):            # 尾端的 T 字
            full[h - px(yc + 0.5):h - px(yc - 0.5), px(xe - LANE_W / 2):px(xe + LANE_W / 2)] = bgr8(LANE_RGB)
    full = shade(full, seed=1)
    cuts = [('floor_shelf', 0.0, SHELF_X), ('floor_slope', SHELF_X, SHELF_X + SLOPE_X),
            ('floor_deep', SHELF_X + SLOPE_X, POOL_X)]
    for name, x0, x1 in cuts:
        save(os.path.join(out_dir, name + '.png'), full[:, px(x0):px(x1)])


def pool_wall_textures(out_dir):
    """牆：box 側面的 UV 是「從外面看過去左→右、上方 = +z」（實測）。牆高 WALL_BOTTOM ~ WALL_TOP。"""
    height = WALL_TOP - WALL_BOTTOM

    def wall(length, targets):
        img = tiles(length, height)
        h = img.shape[0]

        def row(z):                                         # z（水面為 0）→ 影像列
            return int(round((WALL_TOP - z) * PX_PER_M))

        img[row(WALL_TOP):row(-0.02)] = bgr8(DECK_RGB)      # 水面以上：池緣
        img[row(-0.02):row(-0.145)] = bgr8(TRIM_RGB)        # 水面下一排深藍磁磚
        if targets:                                         # 兩端牆上每條水道的十字標記
            for k in range(int(POOL_Y / LANE_PITCH)):
                c = int(round((LANE_PITCH / 2 + k * LANE_PITCH) * PX_PER_M))
                hw = int(LANE_W / 2 * PX_PER_M)
                img[row(-0.30):row(-1.20), c - hw:c + hw] = bgr8(LANE_RGB)
                img[row(-0.60):row(-0.85), c - 2 * hw * 2:c + 2 * hw * 2] = bgr8(LANE_RGB)
        return shade(img, seed=2 + targets, cell_m=2.0)

    save(os.path.join(out_dir, 'wall_end.png'), wall(POOL_Y, True))
    save(os.path.join(out_dir, 'wall_side.png'), wall(POOL_X + 2 * WALL_T, False))


def water_surface_texture(path, size=512):
    """水面從水下往上看的樣子：青藍色底加上細碎的亮紋。用週期函數組成，可以無縫平鋪。"""
    rng = np.random.default_rng(7)
    y, x = np.mgrid[0:size, 0:size].astype(np.float32) / size * 2 * np.pi
    f = np.zeros((size, size), np.float32)
    g = np.zeros((size, size), np.float32)
    for _ in range(40):
        kx, ky = int(rng.integers(3, 15)), int(rng.integers(0, 7))      # 波紋沿 y 拉長
        f += np.sin(kx * x + ky * y * rng.choice([-1, 1]) + rng.uniform(0, 2 * np.pi)) / (kx + ky) ** 0.5
    for _ in range(10):
        kx, ky = int(rng.integers(1, 4)), int(rng.integers(0, 3))
        g += np.sin(kx * x + ky * y * rng.choice([-1, 1]) + rng.uniform(0, 2 * np.pi))
    f = (f - f.mean()) / f.std()
    g = (g - g.mean()) / g.std()
    ridge = np.exp(-np.abs(f) * 3.0)                        # 波峰附近的亮線
    base = np.array([0.30, 0.66, 0.84], np.float32)         # RGB
    hi = np.array([0.86, 0.97, 1.00], np.float32)
    rgb = base[None, None] * (1.0 + 0.10 * g[..., None] + 0.06 * f[..., None]) + (hi - base)[None, None] * ridge[..., None] * 0.55
    save(path, (np.clip(rgb[..., ::-1], 0, 1) * 255).astype(np.uint8))


def haze_texture(path, size=64, peak=0.16):
    """水體霧的粒子貼圖：白色、邊緣漸淡的圓，alpha 很低（一顆只擋一點點，疊多了才看得出來）。"""
    y, x = np.mgrid[0:size, 0:size]
    r = np.hypot(x - (size - 1) / 2, y - (size - 1) / 2) / ((size - 1) / 2)
    img = np.zeros((size, size, 4), np.uint8)
    img[..., :3] = 255
    img[..., 3] = (np.clip(1 - r, 0, 1) ** 1.5 * peak * 255).astype(np.uint8)
    save(path, img)


# ---------------------------------------------------------------- mesh
def write_obj(path, name, verts, uvs, normals, faces):
    """faces：[(v, vt, vn) x 3]，索引從 0 起算。附一個空的 .mtl，不然 gz 會警告 Missing material。"""
    mtl = os.path.splitext(os.path.basename(path))[0] + '.mtl'
    lines = [f'# {HEADER}', f'mtllib {mtl}', f'o {name}', 'usemtl surface']
    lines += [f'v {x:.5f} {y:.5f} {z:.5f}' for x, y, z in verts]
    lines += [f'vt {u:.5f} {v:.5f}' for u, v in uvs]
    lines += [f'vn {x:.5f} {y:.5f} {z:.5f}' for x, y, z in normals]
    lines += ['f ' + ' '.join(f'{v + 1}/{t + 1}/{n + 1}' for v, t, n in tri) for tri in faces]
    write(path, '\n'.join(lines) + '\n')
    write(os.path.join(os.path.dirname(path), mtl), f'# {HEADER}\nnewmtl surface\nKd 1 1 1\n')


def quad_obj(path, sx, sy, repeat_u, repeat_v, normal_up):
    """sx x sy 的平面（原點在中心），UV 重複 repeat_u x repeat_v 次。normal_up=False 時面朝下。"""
    hx, hy = sx / 2, sy / 2
    verts = [(-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0)]
    uvs = [(0, 0), (repeat_u, 0), (repeat_u, repeat_v), (0, repeat_v)]
    if normal_up:
        faces = [[(0, 0, 0), (1, 1, 0), (2, 2, 0)], [(0, 0, 0), (2, 2, 0), (3, 3, 0)]]
        normals = [(0, 0, 1)]
    else:
        faces = [[(0, 0, 0), (2, 2, 0), (1, 1, 0)], [(0, 0, 0), (3, 3, 0), (2, 2, 0)]]
        normals = [(0, 0, -1)]
    write_obj(path, 'quad', verts, uvs, normals, faces)


# drum：規則 = 直徑 60 cm、深 30 cm。實拍是上寬下窄的塑膠盆，錐度與壁厚是估的。
DRUM_R_TOP, DRUM_R_BOT, DRUM_H, DRUM_WALL = 0.30, 0.26, 0.30, 0.012


def drum_obj(path, n=48):
    verts, normals, faces = [], [], []
    uvs = [(0, 0)]

    def ring(r, z):
        i0 = len(verts)
        verts.extend((r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for k in range(n))
        return i0

    def ring_normals(sign, slope):
        i0 = len(normals)
        for k in range(n):
            a = 2 * math.pi * k / n
            nx, ny, nz = sign * math.cos(a), sign * math.sin(a), slope
            norm = math.sqrt(nx * nx + ny * ny + nz * nz)
            normals.append((nx / norm, ny / norm, nz / norm))
        return i0

    def band(a, b, na, nb, flip):
        for k in range(n):
            k2 = (k + 1) % n
            q = [(a + k, 0, na + k), (a + k2, 0, na + k2), (b + k2, 0, nb + k2), (b + k, 0, nb + k)]
            if flip:
                q = q[::-1]
            faces.append([q[0], q[1], q[2]])
            faces.append([q[0], q[2], q[3]])

    taper = (DRUM_R_TOP - DRUM_R_BOT) / DRUM_H
    # 外壁
    ob, ot = ring(DRUM_R_BOT, 0.0), ring(DRUM_R_TOP, DRUM_H)
    no = ring_normals(1, -taper)
    band(ob, ot, no, no, False)
    # 內壁
    ib, it = ring(DRUM_R_BOT - DRUM_WALL, DRUM_WALL), ring(DRUM_R_TOP - DRUM_WALL, DRUM_H)
    ni = ring_normals(-1, taper)
    band(ib, it, ni, ni, True)
    # 盆緣（朝上的環）
    up = len(normals)
    normals.append((0, 0, 1))
    r_o, r_i = ring(DRUM_R_TOP, DRUM_H), ring(DRUM_R_TOP - DRUM_WALL, DRUM_H)
    for k in range(n):
        k2 = (k + 1) % n
        faces.append([(r_o + k, 0, up), (r_o + k2, 0, up), (r_i + k2, 0, up)])
        faces.append([(r_o + k, 0, up), (r_i + k2, 0, up), (r_i + k, 0, up)])
    # 盆底（朝上，從上面看得到）與外底（朝下）
    fb = ring(DRUM_R_BOT - DRUM_WALL, DRUM_WALL)
    c_top = len(verts)
    verts.append((0, 0, DRUM_WALL))
    for k in range(n):
        faces.append([(c_top, 0, up), (fb + k, 0, up), (fb + (k + 1) % n, 0, up)])
    down = len(normals)
    normals.append((0, 0, -1))
    ub = ring(DRUM_R_BOT, 0.0)
    c_bot = len(verts)
    verts.append((0, 0, 0))
    for k in range(n):
        faces.append([(c_bot, 0, down), (ub + (k + 1) % n, 0, down), (ub + k, 0, down)])
    write_obj(path, 'tub', verts, uvs, normals, faces)


# ---------------------------------------------------------------- 模型 SDF
def gen_pool():
    d = os.path.join(ROOT, 'pool_2027')
    tex = os.path.join(d, 'materials', 'textures')
    pool_floor_textures(tex)
    pool_wall_textures(tex)

    drop = DEEP_DEPTH - SHELF_DEPTH
    theta = math.atan2(drop, SLOPE_X)
    slope_len = math.hypot(SLOPE_X, drop)
    deep_x = POOL_X - SHELF_X - SLOPE_X
    uri = 'model://pool_2027/materials/textures/'
    wall_h = WALL_TOP - WALL_BOTTOM
    wall_z = (WALL_TOP + WALL_BOTTOM) / 2

    def floor(name, pose, sx, tex_name):
        return f'''
    <link name="{name}">
      <pose>{pose}</pose>
      <collision name="collision">
        <pose>0 0 {-SLAB_T / 2} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {POOL_Y} {SLAB_T}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>{sx:.4f} {POOL_Y}</size></plane></geometry>
        {tex_mat(uri + tex_name + '.png')}
      </visual>
    </link>'''

    def wall(name, x, y, sx, sy, tex_name):
        return f'''
    <link name="{name}">
      <pose>{x} {y} {wall_z} 0 0 0</pose>
      <collision name="collision"><geometry><box><size>{sx} {sy} {wall_h}</size></box></geometry></collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {wall_h}</size></box></geometry>
        {tex_mat(uri + tex_name + '.png')}
      </visual>
    </link>'''

    sdf = f'''<?xml version="1.0"?>
<!--
  {HEADER}

  SAUVC 2027 泳池：{POOL_X:g} x {POOL_Y:g} m olympic。模型原點在泳池的一個角落、水面上：
    x = 0 ~ {POOL_X:g}：長邊。x = 0 是資格賽起點那面牆，主場地在 x 大的那一端
    y = 0 ~ {POOL_Y:g}：短邊（決賽的跑道方向）
    z = 0：水面，往下為負
  池底照 docs/sim/reference/arena-2027.png 側視圖的標註：牆邊 {SHELF_DEPTH} m 深的平台長 {SHELF_X} m，
  接水平長 {SLOPE_X} m 的斜坡，x = {SHELF_X + SLOPE_X:g} m 之後是 {DEEP_DEPTH} m 的平底。
  （rulebook 圖說寫「資格賽門處 1.28 m」，跟圖上的標註對不上；2026-11-17 決定照圖上的標註，
  資格賽門 x = 10 m 處水深約 {SHELF_DEPTH + drop * (10 - SHELF_X) / SLOPE_X:.2f} m。）
  磁磚、水道線、牆上的十字是照實拍照片畫的，規則沒有規定。
  比賽道具不放在這裡，由世界檔另外 <include>。碰撞體所在的 link 名稱以 floor_／wall_ 開頭，
  sim/referee.py 用它分辨碰到的是池底還是池壁。
-->
<sdf version="1.9">
  <model name="pool">
    <static>true</static>
{floor('floor_shelf', f'{SHELF_X / 2} {POOL_Y / 2} {-SHELF_DEPTH} 0 0 0', SHELF_X, 'floor_shelf')}
{floor('floor_slope', f'{SHELF_X + SLOPE_X / 2} {POOL_Y / 2} {-(SHELF_DEPTH + DEEP_DEPTH) / 2} 0 {theta:.6f} 0', slope_len, 'floor_slope')}
{floor('floor_deep', f'{POOL_X - deep_x / 2} {POOL_Y / 2} {-DEEP_DEPTH} 0 0 0', deep_x, 'floor_deep')}
{wall('wall_x0', -WALL_T / 2, POOL_Y / 2, WALL_T, POOL_Y, 'wall_end')}
{wall('wall_x50', POOL_X + WALL_T / 2, POOL_Y / 2, WALL_T, POOL_Y, 'wall_end')}
{wall('wall_y0', POOL_X / 2, -WALL_T / 2, POOL_X + 2 * WALL_T, WALL_T, 'wall_side')}
{wall('wall_y25', POOL_X / 2, POOL_Y + WALL_T / 2, POOL_X + 2 * WALL_T, WALL_T, 'wall_side')}
  </model>
</sdf>
'''
    write(os.path.join(d, 'model.sdf'), sdf)
    write(os.path.join(d, 'model.config'), model_config('pool_2027', 'SAUVC 2027 泳池（50 x 25 m，池底 1.28 m 斜到 2.0 m）'))


def gen_water():
    d = os.path.join(ROOT, 'water')
    water_surface_texture(os.path.join(d, 'materials', 'textures', 'surface_under.png'))
    haze_texture(os.path.join(d, 'materials', 'textures', 'haze.png'))
    quad_obj(os.path.join(d, 'meshes', 'surface_under.obj'), POOL_X, POOL_Y, POOL_X / 1.25, POOL_Y / 1.25, normal_up=False)
    r, g, b = WATER_RGB
    sdf = f'''<?xml version="1.0"?>
<!--
  {HEADER}

  水：讓場景看起來在水下。模型原點同 pool_2027（泳池角落、水面上）。世界檔把它（和 particle 的
  world plugin）夾在 UNDERWATER:BEGIN ／ UNDERWATER:END 之間，UNDERWATER=0 時 sim/start.sh 會把
  那一段拿掉，回到沒有水的乾淨畫面。全部沒有 collision、不投影。

    surface_under  水面的底面（面朝下、不透明、自發光的波紋貼圖）。相機與 Gazebo 視窗都看得到：
                   在水下往前看時，畫面上緣的水面是全反射，看起來就是一片亮青色的波紋。
    surface_top    水面的頂面（面朝上、半透明的水色）。只有從水面上看得到（Gazebo 視窗的預設視角）；
                   水下的相機在它的背面，看不到它。
    haze           水體霧：Harmonic 的 ogre2 沒有 fog，用一堆很淡的大顆 particle 疊出「越遠越藍」，
                   給 Gazebo 視窗看。相機影像的衰減不靠它，是 sim/underwater.py 用深度圖算的：
                   particle 本來就不會進 depth camera；彩色相機用 visibility_mask 0x0FEFFFFF
                   把 particle（flag 0x00100000）排除（見 model://orca_cam）。
                   世界檔要載 gz-sim-particle-emitter-system 才會有粒子。
                   Gazebo 會印三行「ParticleEmitter SetColorRange is currently disabled」，是正常的。
-->
<sdf version="1.9">
  <model name="water">
    <static>true</static>
    <link name="surface">
      <visual name="surface_under">
        <pose>{POOL_X / 2} {POOL_Y / 2} -0.002 0 0 0</pose>
        <geometry><mesh><uri>model://water/meshes/surface_under.obj</uri></mesh></geometry>
        <cast_shadows>false</cast_shadows>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <emissive>0.75 0.75 0.75 1</emissive>
          <pbr><metal>
            <albedo_map>model://water/materials/textures/surface_under.png</albedo_map>
            <emissive_map>model://water/materials/textures/surface_under.png</emissive_map>
            <metalness>0</metalness><roughness>1</roughness>
          </metal></pbr>
        </material>
      </visual>
      <visual name="surface_top">
        <pose>{POOL_X / 2} {POOL_Y / 2} 0 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>{POOL_X} {POOL_Y}</size></plane></geometry>
        <transparency>0.5</transparency>
        <cast_shadows>false</cast_shadows>
        <material><ambient>0.10 0.50 0.78 1</ambient><diffuse>0.10 0.50 0.78 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
    </link>
    <link name="haze">
      <!-- size 的三個數依序對應模型的 y、z、x（實測，跟直覺不同）：這裡是 x 48、y 23、z 0.6 m 的薄層，
           粒子直徑 2 m，所以霧大約佔滿 z = -2.3 ~ -0.3，不會冒出水面或池壁。
           同時存在的粒子數 = rate x lifetime = 720。 -->
      <particle_emitter name="haze" type="box">
        <pose>{POOL_X / 2} {POOL_Y / 2} -1.3 0 0 0</pose>
        <emitting>true</emitting>
        <size>{POOL_Y - 2} 0.6 {POOL_X - 2}</size>
        <particle_size>2 2 2</particle_size>
        <lifetime>30</lifetime>
        <rate>24</rate>
        <min_velocity>0.005</min_velocity>
        <max_velocity>0.02</max_velocity>
        <scale_rate>0</scale_rate>
        <particle_scatter_ratio>0</particle_scatter_ratio>
        <material>
          <diffuse>{r} {g} {b} 1</diffuse>
          <pbr><metal><albedo_map>model://water/materials/textures/haze.png</albedo_map></metal></pbr>
        </material>
      </particle_emitter>
    </link>
  </model>
</sdf>
'''
    write(os.path.join(d, 'model.sdf'), sdf)
    write(os.path.join(d, 'model.config'), model_config('water', '水面與 Gazebo 視窗用的水體霧（搭配 pool_2027）'))


# 導航門：寬 150、高 150 cm，柱徑 4 cm，條紋「色 ~20 cm、黑 ~10 cm」（gate-2024.jpg）。
# 官方正視圖是沿用 100 cm 高的舊圖改標註，150 cm 時條紋有幾段規則沒寫：這裡照 20／10 往上排滿。
GATE_W, GATE_H, GATE_R = 1.50, 1.50, 0.02


def gen_gate():
    black = (0.05, 0.05, 0.05)
    white = (0.94, 0.95, 0.96)
    bar_z = GATE_H - GATE_R                     # 橫桿頂緣在 150 cm

    def post(name, x, colour):
        bands = [('shoe', 0.0, 0.05, white)]
        z, i = 0.05, 0
        while z < bar_z - 1e-6:
            top = min(z + 0.20, bar_z)
            if bar_z - top < 0.10:              # 最上面不夠再放一段黑的，就把彩色那段拉到橫桿
                top = bar_z
            bands.append((f'band_colour_{i}', z, top, colour))
            z = top
            if z < bar_z - 1e-6:
                bands.append((f'band_black_{i}', z, z + 0.10, black))
                z += 0.10
            i += 1
        vis = '\n'.join(
            f'      <visual name="{n}"><pose>0 0 {(a + b) / 2:.4f} 0 0 0</pose>'
            f'<geometry><cylinder><radius>{GATE_R + (0.001 if c is black else 0)}</radius><length>{b - a:.4f}</length></cylinder></geometry>'
            f'{mat(c)}</visual>' for n, a, b, c in bands)
        return f'''
    <link name="{name}">
      <pose>{x} 0 0 0 0 0</pose>
{vis}
      <collision name="collision">
        <pose>0 0 {bar_z / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{GATE_R}</radius><length>{bar_z:.4f}</length></cylinder></geometry>
      </collision>
    </link>'''

    def rail(name, x, y, sx, sy):
        return (f'      <visual name="{name}"><pose>{x} {y} 0 0 0 0</pose><geometry><box><size>{sx} {sy} 0.05</size></box></geometry>'
                f'{mat(white, 0.15)}</visual>')

    sdf = f'''<?xml version="1.0"?>
<!--
  {HEADER}

  導航門，SAUVC 2027 Rulebook 2027.3.2：寬 {GATE_W * 100:g} cm、高 {GATE_H * 100:g} cm，立在池底；
  port 柱（-x）紅黑條紋、starboard 柱（+x）綠黑條紋，柱徑 4 cm。（2026 是高 100 cm。）
  條紋「色 ~20 cm、黑 ~10 cm」取自 docs/sim/reference/gate-2024.jpg。黑色橫桿與白色 PVC 地面框
  來自實拍（gate-2022.jpg、gate+flare-2022.jpg）；地面框沒有 collision（貼著池底，碰到它等於碰池底）。

  模型原點在池底、兩柱中間；兩柱沿模型 x，船沿模型 y 穿過。static：撞到會被擋住。
  規則：碰到門扣 2 分（sim/referee.py）。
-->
<sdf version="1.9">
  <model name="gate">
    <static>true</static>
{post('port_post', -GATE_W / 2, (0.85, 0.08, 0.08))}
{post('starboard_post', GATE_W / 2, (0.10, 0.75, 0.15))}

    <link name="top_bar">
      <pose>0 0 {bar_z:.4f} 0 1.5707963268 0</pose>
      <visual name="visual">
        <geometry><cylinder><radius>{GATE_R}</radius><length>{GATE_W + 2 * GATE_R}</length></cylinder></geometry>
        {mat(black)}
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>{GATE_R}</radius><length>{GATE_W + 2 * GATE_R}</length></cylinder></geometry>
      </collision>
    </link>

    <link name="base_frame">
      <pose>0 0 0.025 0 0 0</pose>
{rail('rail_port', -GATE_W / 2, 0, 0.05, 1.10)}
{rail('rail_starboard', GATE_W / 2, 0, 0.05, 1.10)}
{rail('rail_near', 0, -0.55, GATE_W + 0.05, 0.05)}
{rail('rail_far', 0, 0.55, GATE_W + 0.05, 0.05)}
    </link>
  </model>
</sdf>
'''
    write(os.path.join(ROOT, 'gate', 'model.sdf'), sdf)


# 紅／黃／藍 flare：高 80 cm、桿徑 1.6 cm、頂端一顆高爾夫球（規則）。底盤、質量是估的。
FLARE_H, FLARE_R = 0.80, 0.008
FLARE_BASE_R, FLARE_BASE_H = 0.065, 0.02
BALL_R, BALL_M = 0.02135, 0.0459            # 標準高爾夫球
SEAT_N, SEAT_R, SEAT_A = 6, 0.004, 0.007    # 球座：6 顆小球排成半徑 7 mm 的圈


def gen_flares():
    seat_z = FLARE_H - SEAT_R                               # 小球頂端 = 桿頂 80 cm
    ball_z = seat_z + math.sqrt((BALL_R + SEAT_R) ** 2 - SEAT_A ** 2)
    tip_deg = math.degrees(math.atan2(SEAT_A * math.cos(math.pi / SEAT_N), ball_z - seat_z))
    rod_len = seat_z - FLARE_BASE_H
    rod_z = FLARE_BASE_H + rod_len / 2
    # 水裡的等效重量（浮力沒有另外算在 flare 上，直接用扣掉浮力後的質量）
    m_base, m_rod = 0.7, 0.1
    m = m_base + m_rod
    com = (m_base * FLARE_BASE_H / 2 + m_rod * rod_z) / m
    ixx = (m_base * (3 * FLARE_BASE_R ** 2 + FLARE_BASE_H ** 2) / 12 + m_base * (com - FLARE_BASE_H / 2) ** 2
           + m_rod * rod_len ** 2 / 12 + m_rod * (rod_z - com) ** 2)
    izz = m_base * FLARE_BASE_R ** 2 / 2 + m_rod * FLARE_R ** 2 / 2
    i_ball = 0.4 * BALL_M * BALL_R ** 2
    seats = '\n'.join(
        f'      <collision name="seat_{k}"><pose>{SEAT_A * math.cos(2 * math.pi * k / SEAT_N):.5f} '
        f'{SEAT_A * math.sin(2 * math.pi * k / SEAT_N):.5f} {seat_z:.4f} 0 0 0</pose>'
        f'<geometry><sphere><radius>{SEAT_R}</radius></sphere></geometry></collision>' for k in range(SEAT_N))
    colours = {'red': (0.85, 0.08, 0.08), 'yellow': (0.95, 0.80, 0.10), 'blue': (0.10, 0.30, 0.85)}
    for name, rgb in colours.items():
        sdf = f'''<?xml version="1.0"?>
<!--
  {HEADER}

  {name} flare，SAUVC 2027 Rulebook 2027.3.2：高 {FLARE_H * 100:g} cm、桿徑 {FLARE_R * 200:g} cm 的硬桿，
  頂端擱一顆高爾夫球。外形照 docs/sim/reference/flares-2024.jpg 與隊上的實拍：細桿插在一塊同色的
  圓底盤上，直接立在池底。沒有浮力、沒有繫繩、沒有固定：撞下去整支會晃、會倒、會被推走，
  不會自己站回直立（跟橘色 flare 完全不同，見 orange_flare）。

  兩個 link：
    pole  底盤 + 桿 + 球座。質量 {m} kg 是扣掉浮力後的等效值（世界檔的 Buoyancy 沒有把它列進
          <enable>），重心很低；在離地 50 cm 處大約 {m * 9.8 * FLARE_BASE_R / 0.5:.1f} N 就推得倒。
    ball  高爾夫球（直徑 {BALL_R * 200:.2f} cm、{BALL_M * 1000:.1f} g），沒有 joint，只是擱在球座上。
          球座是 {SEAT_N} 顆小球排成的圈，桿子傾斜超過約 {tip_deg:.0f}° 或被撞一下球就掉。
          世界檔的 Buoyancy 要 <enable>模型名::ball</enable>，球才會慢慢沉而不是直接摔下去。
  self_collide：同一個模型的 link 預設互不碰撞，要開才擱得住球。
  模型原點在池底（底盤底面中心）。規則：球掉下來 20 分，三支照指定順序再加 60 分（sim/referee.py）。
-->
<sdf version="1.9">
  <model name="flare_{name}">
    <self_collide>true</self_collide>

    <link name="pole">
      <inertial>
        <pose>0 0 {com:.4f} 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.5f}</ixx><iyy>{ixx:.5f}</iyy><izz>{izz:.6f}</izz></inertia>
      </inertial>
      <collision name="base">
        <pose>0 0 {FLARE_BASE_H / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{FLARE_BASE_R}</radius><length>{FLARE_BASE_H}</length></cylinder></geometry>
      </collision>
      <collision name="rod">
        <pose>0 0 {rod_z:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{FLARE_R}</radius><length>{rod_len:.4f}</length></cylinder></geometry>
      </collision>
{seats}
      <visual name="base">
        <pose>0 0 {FLARE_BASE_H / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{FLARE_BASE_R}</radius><length>{FLARE_BASE_H}</length></cylinder></geometry>
        {mat(rgb)}
      </visual>
      <visual name="base_hub">
        <pose>0 0 {FLARE_BASE_H + 0.015} 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.03</length></cylinder></geometry>
        {mat(rgb)}
      </visual>
      <visual name="rod">
        <pose>0 0 {rod_z:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{FLARE_R}</radius><length>{rod_len:.4f}</length></cylinder></geometry>
        {mat(rgb)}
      </visual>
      <visual name="seat">
        <pose>0 0 {seat_z - 0.003:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{SEAT_A + SEAT_R}</radius><length>0.012</length></cylinder></geometry>
        {mat(rgb)}
      </visual>
    </link>

    <link name="ball">
      <pose>0 0 {ball_z:.5f} 0 0 0</pose>
      <inertial>
        <mass>{BALL_M}</mass>
        <inertia><ixx>{i_ball:.3e}</ixx><iyy>{i_ball:.3e}</iyy><izz>{i_ball:.3e}</izz></inertia>
      </inertial>
      <collision name="collision"><geometry><sphere><radius>{BALL_R}</radius></sphere></geometry></collision>
      <visual name="visual">
        <geometry><sphere><radius>{BALL_R}</radius></sphere></geometry>
        {mat((0.95, 0.95, 0.93), 0.3)}
      </visual>
    </link>
  </model>
</sdf>
'''
        d = os.path.join(ROOT, f'flare_{name}')
        write(os.path.join(d, 'model.sdf'), sdf)
        write(os.path.join(d, 'model.config'), model_config(f'flare_{name}', f'{name} flare（硬桿 + 高爾夫球），SAUVC 2027'))


def gen_drums():
    drum_obj(os.path.join(ROOT, 'drum_red', 'meshes', 'tub.obj'))
    n = 16
    r_mid = (DRUM_R_TOP + DRUM_R_BOT) / 2 - DRUM_WALL / 2
    seg = 2 * (r_mid + DRUM_WALL) * math.tan(math.pi / n) + 0.004
    walls = '\n'.join(
        f'      <collision name="wall_{k}"><pose>{r_mid * math.cos(2 * math.pi * k / n):.4f} {r_mid * math.sin(2 * math.pi * k / n):.4f} '
        f'{DRUM_H / 2} 0 0 {2 * math.pi * k / n:.5f}</pose><geometry><box><size>{DRUM_WALL} {seg:.4f} {DRUM_H}</size></box></geometry></collision>'
        for k in range(n))
    variants = {
        'drum_red': ((0.85, 0.10, 0.08), False, '紅 drum（沒有 pinger）：球進去 10 分'),
        'drum_blue': ((0.10, 0.25, 0.75), False, '藍 drum：球進去 30 分'),
        'drum_red_pinger': ((0.85, 0.10, 0.08), True, '裝 45 kHz pinger 的紅 drum：球進去 50 分'),
    }
    for name, (rgb, pinger, desc) in variants.items():
        pinger_vis = '''
      <!-- RJE ULB-362B/45 kHz pinger（docs/sim/reference/pinger.jpg）：只有外觀，沒有模擬聲學 -->
      <visual name="pinger">
        <pose>0.12 0 0.029 0 1.5707963268 0</pose>
        <geometry><cylinder><radius>0.017</radius><length>0.10</length></cylinder></geometry>
        ''' + mat((0.75, 0.75, 0.72), 0.4) + '''
      </visual>''' if pinger else ''
        sdf = f'''<?xml version="1.0"?>
<!--
  {HEADER}

  {desc}。SAUVC 2027 Rulebook 2027.3.2：直徑 {DRUM_R_TOP * 200:g} cm、深 {DRUM_H * 100:g} cm，放在綠墊上。
  外形照 docs/sim/reference/drums-2017.jpg 與隊上的實拍：上寬下窄的塑膠盆（錐度、壁厚是估的）。
  外觀是 mesh（drum_red/meshes/tub.obj，三種 drum 共用）；碰撞是盆底一片圓板加一圈 {n} 塊 box，
  中間是空的，球掉得進去、留得住。static。模型原點在盆底中心。
  哪個模型放在綠墊的哪個位置由世界檔決定（比賽會隨機）；sim/referee.py 用模型的 uri 名稱
  （drum_blue／drum_red_pinger／drum_red）決定球進去的分數。
-->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="tub">
      <visual name="tub">
        <geometry><mesh><uri>model://drum_red/meshes/tub.obj</uri></mesh></geometry>
        {mat(rgb, 0.2)}
      </visual>{pinger_vis}
      <collision name="bottom">
        <pose>0 0 {DRUM_WALL / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{DRUM_R_BOT}</radius><length>{DRUM_WALL}</length></cylinder></geometry>
      </collision>
{walls}
    </link>
  </model>
</sdf>
'''
        d = os.path.join(ROOT, name)
        write(os.path.join(d, 'model.sdf'), sdf)
        write(os.path.join(d, 'model.config'), model_config(name, desc + '，SAUVC 2027'))


def main():
    print('pool_2027'); gen_pool()
    print('water'); gen_water()
    print('gate'); gen_gate()
    print('flares'); gen_flares()
    print('drums'); gen_drums()


if __name__ == '__main__':
    main()
