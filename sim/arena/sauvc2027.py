#!/usr/bin/env python3
"""SAUVC 2027 模擬場地：泳池、道具模型、資格賽與決賽的世界檔。

尺寸一律取自 2027 規則書（https://sauvc.org/rulebook/），規則書沒寫的地方在註解裡標「假設」。
這支程式是場地尺寸的唯一來源：改尺寸改這裡，再重新產生。只用標準函式庫。

    python3 sim/arena/sauvc2027.py                      # 重新產生模型和兩個預設世界（照官方場地圖擺）
    python3 sim/arena/sauvc2027.py finals --seed 7      # 決賽隨機佈局 → sim/worlds/generated/
    python3 sim/arena/sauvc2027.py qualification --lane 3
    python3 sim/arena/sauvc2027.py models               # 只重新產生 sim/models/sauvc27_*

泳池貼圖另外用 make_pool_textures.py 產生（需要 numpy、Pillow）。

世界座標（兩個世界相同）：原點在泳池角落的水面，
  x：沿 50 m 長邊，0 = 淺端池壁（資格賽起點的牆）
  y：沿 25 m 短邊，0 = 決賽出發區那一側的池壁
  z：向上，水面 = 0
每個世界旁邊會寫一份 <世界>.layout.json：各道具的真值位置，之後裁判、評分程式可以讀。
"""
import argparse
import json
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
MODELS = os.path.join(SIM, 'models')
WORLDS = os.path.join(SIM, 'worlds')

# ---------------------------------------------------------------- 泳池（規則書 §8、Figure 5）
POOL_L = 50.0           # 奧運池 50 m × 25 m
POOL_W = 25.0
SHALLOW_DEPTH = 1.28    # 側視圖：淺端水深
SHALLOW_LEN = 1.8       # 側視圖：淺端平台長度
SLOPE_LEN = 17.5        # 側視圖：斜坡的水平長度
DEEP_DEPTH = 2.0        # 側視圖：深區水深
DECK = 0.4              # 假設：池壁高出水面 0.4 m（沿用 2026 模擬）
WALL_T = 0.3

LANE_W = 2.5            # 奧運池 10 水道 × 2.5 m，水道線沿 x


def depth_at(x):
    """x 處的水深 [m]。"""
    if x <= SHALLOW_LEN:
        return SHALLOW_DEPTH
    if x >= SHALLOW_LEN + SLOPE_LEN:
        return DEEP_DEPTH
    return SHALLOW_DEPTH + (x - SHALLOW_LEN) / SLOPE_LEN * (DEEP_DEPTH - SHALLOW_DEPTH)


def floor_z(x):
    return -depth_at(x)


# ---------------------------------------------------------------- 資格賽區（§7、§8.1、Figure 4/5/8）
QUAL_STRIP = 10.0                   # 資格賽區在淺端，寬 10 m
QUAL_LANES_Y = (2.5, 12.5, 22.5)    # 場地圖：Qualification Arena 1、2、3（由出發區那一側算起）
QUAL_GATE_X = 10.0                  # 「離起點約 10 m」
QUAL_GATE_W = 1.5                   # 150 cm 寬（兩柱中心距）
QUAL_GATE_DEPTH = 1.0               # 約 100 cm 深，從水面垂下
QUAL_POST = 0.04                    # 柱寬 4 cm（Figure 8）

# ---------------------------------------------------------------- 決賽主場地（§8、§9、Figure 5）
MAIN_X = (22.0, 50.0)               # 主場地 28 m，在深端（假設：從池壁量 28 m）
START_ZONE = (36.0, 2.0)            # 出發區 140 × 140 cm；場地圖位置（不按比例，取近似）
START_ZONE_SIZE = 1.4
ORANGE_ZONE_Y = (4.0, 8.0)          # 橘色 flare：離池邊 4–8 m 的帶狀區，x 在主場地內任意
GATE_LINE_Y = 12.0                  # 導航門：離池邊約 12 m 的線上任意 x
GATE_W = 1.5                        # 150 cm 寬、150 cm 高
GATE_H = 1.5
TARGET_ZONE = (31.0, 23.0)          # 綠色墊 8 m × 2 m 的中心；場地圖位置（近似）
TARGET_ZONE_SIZE = (8.0, 2.0)
DRUM_SPACING = 2.0                  # 假設：4 個桶沿墊子長邊等距排列
DRUM_D = 0.60                       # 直徑 60 cm、深 30 cm
DRUM_H = 0.30
FLARE_BAND_Y = (13.5, 20.5)         # 場地圖上紅黃藍 flare 的帶狀區（規則書文字：主場地內任意）
SURFACING_SQUARE = (11.5, 7.0)      # 「浮在資格賽門旁邊」；場地圖在 Arena 1、2 之間，緊鄰 10 m 線
SURFACING_SQUARE_SIZE = 2.0

# 預設佈局：照官方場地圖（Figure 5）上的位置
CANONICAL_FINALS = {
    'gate_x': 31.5,
    'orange_flare': (32.6, 5.4),
    'drum_order': ['blue', 'red', 'red', 'red'],    # 由 x 小到大
    'pinger': 2,                                    # 哪一個桶（0 起算）有 pinger，必須是紅桶
    'flares': {'yellow': (28.4, 17.9), 'red': (39.5, 19.4), 'blue': (45.4, 16.2)},
    'flare_order': 'R-B-Y',
}

# ---------------------------------------------------------------- 顏色
ORANGE = (1.00, 0.45, 0.05)
WHITE = (0.94, 0.95, 0.96)
BLACK = (0.05, 0.05, 0.05)
RED = (0.85, 0.08, 0.08)
GREEN = (0.10, 0.75, 0.15)
FLARE_RGB = {'red': (0.85, 0.05, 0.05), 'yellow': (0.95, 0.80, 0.10), 'blue': (0.15, 0.35, 0.85)}
DRUM_RGB = {'red': (0.78, 0.13, 0.11), 'blue': (0.10, 0.34, 0.76)}
MAT_GREEN = (0.15, 0.50, 0.20)
MARKER_BLUE = (0.05, 0.20, 0.90)
START_LINE = (0.85, 0.60, 0.10)


# ================================================================ SDF 小工具
def f(*v):
    return ' '.join(f'{x:.4f}'.rstrip('0').rstrip('.') if isinstance(x, float) else str(x) for x in v)


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f'<pose>{f(float(x), float(y), float(z), float(roll), float(pitch), float(yaw))}</pose>'


def box(sx, sy, sz):
    return f'<box><size>{f(float(sx), float(sy), float(sz))}</size></box>'


def cyl(r, length):
    return f'<cylinder><radius>{f(float(r))}</radius><length>{f(float(length))}</length></cylinder>'


def sphere(r):
    return f'<sphere><radius>{f(float(r))}</radius></sphere>'


def material(rgb, spec=0.05):
    r, g, b = rgb
    amb = f(r * 0.4, g * 0.4, b * 0.4)
    return (f'<material><ambient>{amb} 1</ambient><diffuse>{f(r, g, b)} 1</diffuse>'
            f'<specular>{f(spec, spec, spec)} 1</specular></material>')


def part(name, geom, p, rgb, collide=True):
    """一個形狀：visual，加上同形狀的 collision。"""
    out = f'<visual name="{name}">{p}<geometry>{geom}</geometry>{material(rgb)}</visual>'
    if collide:
        out += f'<collision name="{name}_col">{p}<geometry>{geom}</geometry></collision>'
    return out


def inertial(mass, ixx, iyy, izz, z=0.0):
    return (f'<inertial>{pose(z=z)}<mass>{f(float(mass))}</mass>'
            f'<inertia><ixx>{ixx:.3g}</ixx><ixy>0</ixy><ixz>0</ixz>'
            f'<iyy>{iyy:.3g}</iyy><iyz>0</iyz><izz>{izz:.3g}</izz></inertia></inertial>')


def model_sdf(name, doc, body, static=True, self_collide=False):
    extra = '    <self_collide>true</self_collide>\n' if self_collide else ''
    return (f'<?xml version="1.0"?>\n<!--\n{doc.strip()}\n\n'
            f'  由 sim/arena/sauvc2027.py 產生，不要手改。\n-->\n'
            f'<sdf version="1.9">\n  <model name="{name}">\n'
            f'    <static>{"true" if static else "false"}</static>\n{extra}'
            f'    {body}\n  </model>\n</sdf>\n')


def model_config(name, desc):
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <description>{desc}</description>\n</model>\n')


# ================================================================ 模型
def m_pool():
    tex = 'model://sauvc27_pool/materials/textures'

    def floor_part(name, x0, x1):
        z0, z1 = floor_z(x0), floor_z(x1)
        cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
        run = x1 - x0
        length = math.hypot(run, z1 - z0)
        pitch = math.atan2(z0 - z1, run)       # 正的 pitch：往 +x 向下
        mat = (f'<material><diffuse>1 1 1 1</diffuse><specular>0.08 0.08 0.08 1</specular>'
               f'<pbr><metal><albedo_map>{tex}/{name}.png</albedo_map>'
               f'<metalness>0</metalness><roughness>0.55</roughness></metal></pbr></material>')
        # 視覺用 plane（UV 0..1 鋪滿，貼圖是這一段的單張烘焙）；碰撞用薄 box（plane 的碰撞是無限大）
        return (f'<link name="{name}">{pose(cx, POOL_W / 2, cz, 0, pitch, 0)}'
                f'<visual name="visual"><geometry><plane><normal>0 0 1</normal>'
                f'<size>{f(length, POOL_W)}</size></plane></geometry>{mat}</visual>'
                f'<collision name="collision">{pose(z=-0.1)}<geometry>{box(length, POOL_W, 0.2)}</geometry>'
                f'<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>'
                f'</collision></link>')

    x_slope_end = SHALLOW_LEN + SLOPE_LEN
    floors = (floor_part('floor_shallow', 0.0, SHALLOW_LEN)
              + floor_part('floor_slope', SHALLOW_LEN, x_slope_end)
              + floor_part('floor_deep', x_slope_end, POOL_L))

    bottom = -DEEP_DEPTH - 0.1
    h = DECK - bottom
    cz = (DECK + bottom) / 2

    def wall(name, x, y, sx, sy, texture):
        g = box(sx, sy, h)
        mat = (f'<material><diffuse>1 1 1 1</diffuse><specular>0.08 0.08 0.08 1</specular>'
               f'<pbr><metal><albedo_map>{tex}/{texture}.png</albedo_map>'
               f'<metalness>0</metalness><roughness>0.55</roughness></metal></pbr></material>')
        return (f'<link name="{name}">{pose(x, y, cz)}'
                f'<visual name="visual"><geometry>{g}</geometry>{mat}</visual>'
                f'<collision name="collision"><geometry>{g}</geometry></collision></link>')

    t = WALL_T
    walls = (wall('wall_shallow', -t / 2, POOL_W / 2, t, POOL_W, 'wall_short')
             + wall('wall_deep', POOL_L + t / 2, POOL_W / 2, t, POOL_W, 'wall_short')
             + wall('wall_start_side', POOL_L / 2, -t / 2, POOL_L + 2 * t, t, 'wall_long')
             + wall('wall_far_side', POOL_L / 2, POOL_W + t / 2, POOL_L + 2 * t, t, 'wall_long'))
    doc = f'''
  SAUVC 2027 泳池：{POOL_L:g} m × {POOL_W:g} m 奧運池（規則書 §8 Figure 5 側視圖）。
  淺端水深 {SHALLOW_DEPTH} m、平台長 {SHALLOW_LEN} m，斜坡水平長 {SLOPE_LEN} m，之後深區水深 {DEEP_DEPTH} m。
  原點在淺端、出發區那一側的池角水面；x 沿長邊，y 沿短邊。池壁高出水面 {DECK} m。
  地板分三段，各段的貼圖由 sim/arena/make_pool_textures.py 產生（水道線在真實位置）。'''
    return model_sdf('sauvc27_pool', doc, floors + walls), '50 x 25 m SAUVC 2027 pool with sloped floor'


def m_qual_gate():
    w, d, p = QUAL_GATE_W, QUAL_GATE_DEPTH, QUAL_POST
    body = '<link name="gate">'
    body += part('float_bar', box(w + p, 0.10, 0.06), pose(), WHITE)
    for side, x in (('port', -w / 2), ('starboard', w / 2)):
        body += part(f'{side}_post', box(p, p, d), pose(x, 0, -d / 2), ORANGE)
    body += '</link>'
    doc = f'''
  資格賽門（規則書 §8.1、Figure 8）：{w * 100:.0f} cm 寬、約 {d * 100:.0f} cm 深，從水面垂下，
  左右兩柱 {p * 100:.0f} cm 寬、橘色，頂上是浮在水面的白色浮桿。碰到任何一邊就算失敗。
  原點在水面、兩柱中間；柱子在模型 x 方向，從模型 ±y 方向穿過。'''
    return model_sdf('sauvc27_qual_gate', doc, body), 'SAUVC 2027 qualification gate (hangs 1 m from the surface)'


def m_gate():
    w, h, r = GATE_W, GATE_H, 0.02
    shoe = 0.06
    top = h - r
    # Figure 11：每柱 4 段顏色、3 段黑（約 20 cm／10 cm），最上面一段是顏色，底下有白色腳
    unit = (top - shoe) / 11.0
    segs, z = [], shoe
    for i in range(7):
        seg = unit * (2 if i % 2 == 0 else 1)
        segs.append(('colour' if i % 2 == 0 else 'black', z, seg))
        z += seg
    body = ''
    for side, x, rgb in (('port', -w / 2, RED), ('starboard', w / 2, GREEN)):
        body += f'<link name="{side}_post">{pose(x, 0, 0)}'
        body += part('shoe', cyl(r, shoe), pose(z=shoe / 2), WHITE, collide=False)
        for i, (kind, z0, seg) in enumerate(segs):
            body += part(f'{kind}_{i}', cyl(r + (0.0005 if kind == 'black' else 0), seg),
                         pose(z=z0 + seg / 2), rgb if kind == 'colour' else BLACK, collide=False)
        body += f'<collision name="collision">{pose(z=h / 2)}<geometry>{cyl(r, h)}</geometry></collision>'
        body += '</link>'
    body += f'<link name="top_bar">{pose(0, 0, h, 0, math.pi / 2, 0)}'
    body += part('bar', cyl(r, w + 2 * r), pose(), BLACK)
    body += '</link>'
    # 白色 PVC 底座：規則書圖沒畫，比賽照片（2022）上有；偵測模型是用照片訓練的
    body += f'<link name="base_frame">{pose(z=0.025)}'
    for name, x, y, sx, sy in (('rail_port', -w / 2, 0, 0.05, 1.10), ('rail_starboard', w / 2, 0, 0.05, 1.10),
                               ('rail_near', 0, -0.55, w + 0.05, 0.05), ('rail_far', 0, 0.55, w + 0.05, 0.05)):
        body += part(name, box(sx, sy, 0.05), pose(x, y, 0), WHITE)
    body += '</link>'
    doc = f'''
  導航門（規則書 §9.1、Figure 11）：{w * 100:.0f} cm 寬、{h * 100:.0f} cm 高，立在池底。
  左舷（port）柱紅黑條紋、右舷（starboard）柱綠黑條紋，柱徑 4 cm，頂上黑色橫桿。
  左右舷以「從模型 -y 往 +y 穿過」為準：紅柱在 -x、綠柱在 +x。
  白色 PVC 底座取自 2022 比賽照片（規則書圖沒有）。原點在池底、兩柱中間。'''
    return model_sdf('sauvc27_gate', doc, body), 'SAUVC 2027 navigation gate, 1.5 x 1.5 m, red port / green starboard'


def m_orange_flare():
    body = '<link name="flare">'
    body += part('weight', cyl(0.11, 0.04), pose(z=0.02), BLACK)
    body += part('blade', box(0.15, 0.07, 1.50), pose(z=0.79), (1.00, 0.26, 0.05))
    body += part('cuff', box(0.15, 0.075, 0.10), pose(z=0.09), (0.10, 0.10, 0.10), collide=False)
    body += '</link>'
    doc = '''
  橘色 flare（規則書 §9.1、Figure 12）：約 150 cm 高、約 15 cm 寬，立在池底。
  碰到就立刻中止這次 attempt。比賽照片上是綁在配重上的充氣標示浮標（扁的），
  外形沿用 2026 模擬（SAUVC-Simulation 的 orange_flare）。static：不會被撞歪。原點在池底。'''
    return model_sdf('sauvc27_orange_flare', doc, body), 'SAUVC 2027 orange flare (obstacle), 1.5 m tall'


def m_flare(colour):
    rgb = FLARE_RGB[colour]
    pole_h, pole_r = 0.80, 0.008
    ball_r, ring_r, ring_top = 0.02135, 0.0145, pole_h + 0.014
    body = f'<link name="flare">{inertial(1.5, 0.02, 0.02, 0.004, z=0.04)}'
    body += part('foot', cyl(0.07, 0.03), pose(z=0.015), rgb)
    body += part('pole', cyl(pole_r, pole_h), pose(z=pole_h / 2), rgb)
    body += part('cup', cyl(0.012, 0.006), pose(z=pole_h + 0.003), rgb)
    for i in range(8):     # 托住球的一圈小凸起，球才放得穩，碰一下就掉
        a = 2 * math.pi * i / 8
        body += part(f'cup_lip_{i}', box(0.006, 0.006, 0.008),
                     pose(ring_r * math.cos(a), ring_r * math.sin(a), pole_h + 0.010, 0, 0, a), rgb)
    body += '</link>'
    inner = ring_r - 0.003
    ball_z = ring_top + math.sqrt(ball_r ** 2 - inner ** 2) + 0.001
    i_ball = 0.4 * 0.0459 * ball_r ** 2
    body += (f'<link name="golf_ball">{pose(z=ball_z)}{inertial(0.0459, i_ball, i_ball, i_ball)}'
             + part('ball', sphere(ball_r), pose(), WHITE) + '</link>')
    doc = f'''
  {colour} flare（規則書 §9.4、Figure 17）：80 cm 高、直徑約 1.6 cm，頂上放一顆高爾夫球。
  撞到（或靠近）讓球掉下來就得分。flare 本體與球都是動態的：
    - 本體 1.5 kg、重心壓低在底座，被撞會晃、會倒；
    - 球是獨立的 link（沒有 joint），放在杆頂的小托座上，晃動夠大就會滾落。
  桿子用規則書的 1.6 cm（2026 模擬為了偵測把視覺放大 5 倍，這裡沒有）。原點在池底。'''
    # self_collide：同一個模型裡的 link 預設彼此不碰撞，不開的話球會直接穿過托座掉到池底。
    # gz-sim 看的是模型層級的 <self_collide>，寫在 link 上沒有用。
    return model_sdf(f'sauvc27_flare_{colour}', doc, body, static=False, self_collide=True), \
        f'SAUVC 2027 {colour} flare with a golf ball on top'


def m_drum(colour):
    rgb = DRUM_RGB[colour]
    r, h, t, n = DRUM_D / 2, DRUM_H, 0.012, 24
    chord = 2 * r * math.sin(math.pi / n) + 0.004
    body = '<link name="drum">'
    body += part('bottom', cyl(r, 0.01), pose(z=0.005), rgb)
    for i in range(n):
        a = 2 * math.pi * i / n
        rr = r - t / 2
        body += part(f'wall_{i}', box(t, chord, h), pose(rr * math.cos(a), rr * math.sin(a), h / 2, 0, 0, a), rgb)
    body += '</link>'
    doc = f'''
  {colour} 桶（規則書 §9.2）：直徑 {DRUM_D * 100:.0f} cm、深 {DRUM_H * 100:.0f} cm，開口朝上，放在綠色墊子上。
  桶壁用 {n} 片拼成，裡面是空的（之後放球、撿球用得到）。原點在桶底中心。'''
    return model_sdf(f'sauvc27_drum_{colour}', doc, body), f'SAUVC 2027 {colour} drum, 60 cm x 30 cm, open top'


def m_pinger():
    body = '<link name="pinger">'
    body += part('body', cyl(0.025, 0.10), pose(z=0.06), (0.15, 0.15, 0.15), collide=False)
    body += '</link>'
    doc = '''
  桶裡的 pinger（規則書 §9.2：RJE ULB-362B/45 kHz）。只有外觀，沒有模擬聲學訊號。
  放在其中一個紅桶的桶底，原點在桶底。'''
    return model_sdf('sauvc27_pinger', doc, body), 'Acoustic pinger placeholder (visual only)'


def m_target_zone():
    sx, sy = TARGET_ZONE_SIZE
    body = '<link name="mat">' + part('mat', box(sx, sy, 0.01), pose(z=0.005), MAT_GREEN) + '</link>'
    doc = f'''
  Target zone（規則書 §9.2）：鋪在池底的綠色墊子，{sx:g} m × {sy:g} m（長邊沿模型 x）。原點在池底、墊子中心。'''
    return model_sdf('sauvc27_target_zone', doc, body), 'SAUVC 2027 target zone mat, 8 x 2 m'


def frame(size, pipe, rgb, collide):
    s = size / 2 - pipe / 2
    out = ''
    for name, x, y, sx, sy in (('north', 0, s, size, pipe), ('south', 0, -s, size, pipe),
                               ('east', s, 0, pipe, size - 2 * pipe), ('west', -s, 0, pipe, size - 2 * pipe)):
        out += part(f'pipe_{name}', box(sx, sy, pipe), pose(x, y, 0), rgb, collide)
    return out


def m_starting_zone():
    body = '<link name="frame">' + frame(START_ZONE_SIZE, 0.04, WHITE, collide=False) + '</link>'
    doc = f'''
  出發區（規則書 §8.2）：水面上 {START_ZONE_SIZE * 100:.0f} × {START_ZONE_SIZE * 100:.0f} cm 的浮框。
  船只能放在這裡的水面，離開出發區前要自己下潛。只有外觀（沒有碰撞），不影響船出發。原點在水面、框中心。'''
    return model_sdf('sauvc27_starting_zone', doc, body), 'SAUVC 2027 starting zone, 1.4 x 1.4 m floating frame'


def m_surfacing_square():
    size, pipe = SURFACING_SQUARE_SIZE, 0.05
    body = '<link name="square">' + frame(size, pipe, WHITE, collide=True)
    c = size / 2 - pipe / 2
    for i, (x, y) in enumerate(((c, c), (-c, c), (-c, -c), (c, -c))):
        body += part(f'marker_{i}', box(0.05, 0.05, 0.30), pose(x, y, -pipe / 2 - 0.15), MARKER_BLUE)
    body += '</link>'
    doc = f'''
  Surfacing Square（規則書 §9.5、Figure 18）：浮在水面的 {size * 100:.0f} × {size * 100:.0f} cm PVC 框，
  框下掛藍色標記約 30 cm 深（假設：四個角各一根）。完成導航後在框內浮出、沒碰到框，+20 分。
  框和標記都有碰撞。原點在水面、框中心。'''
    return model_sdf('sauvc27_surfacing_square', doc, body), 'SAUVC 2027 surfacing square, 2 x 2 m'


def m_qual_start_line():
    body = '<link name="mark">' + part('mark', box(0.01, 1.0, 0.40), pose(0.005, 0, -0.1), START_LINE,
                                       collide=False) + '</link>'
    doc = '''
  資格賽起點（規則書 §8.1）：池壁上標出的 1 m 寬一段，船要從這裡出發、開始時碰著牆。
  只有外觀。原點在池壁表面的水面處，+x 朝池內。'''
    return model_sdf('sauvc27_qual_start_line', doc, body), 'SAUVC 2027 qualification starting line marker'


MODEL_BUILDERS = [m_pool, m_qual_gate, m_gate, m_orange_flare, m_target_zone, m_pinger,
                  m_starting_zone, m_surfacing_square, m_qual_start_line,
                  *[(lambda c=c: m_flare(c)) for c in FLARE_RGB],
                  *[(lambda c=c: m_drum(c)) for c in DRUM_RGB]]


def write_models():
    for build in MODEL_BUILDERS:
        sdf, desc = build()
        name = sdf.split('<model name="', 1)[1].split('"', 1)[0]
        d = os.path.join(MODELS, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8') as fh:
            fh.write(sdf)
        with open(os.path.join(d, 'model.config'), 'w', encoding='utf-8') as fh:
            fh.write(model_config(name, desc))
        print(f'  {os.path.relpath(d, SIM)}/')


# ================================================================ 世界
WORLD_HEAD = '''<?xml version="1.0"?>
<!--
{doc}

  由 sim/arena/sauvc2027.py 產生，不要手改；道具位置的真值在旁邊的 {name}.layout.json。
  座標：原點在淺端、出發區那一側的池角水面；x 沿 50 m 長邊，y 沿 25 m 短邊，z 向上。
-->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- 相機的算圖。ogre2；沒開 GUI 時 start.sh 會加 headless rendering -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- 更新模型裡的 IMU 感測器（ArduPilotPlugin 讀它） -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!-- 水的浮力：z < 0 為 1000 kg/m^3，z > 0 為空氣 -->
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1000</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1</density>
        </density_change>
      </graded_buoyancy>
    </plugin>

    <scene>
      <ambient>0.7 0.72 0.74 1</ambient>
      <background>0.8 0.88 0.92 1</background>
    </scene>

    <!-- 室內泳池：接近正上方的柔和主光，加兩道低角度補光讓四面池壁都看得到 -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.75 0.76 0.78 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.15 0.08 -0.98</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <light type="directional" name="wall_fill_a">
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.3 0.31 0.32 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.82 0.41 -0.41</direction>
      <cast_shadows>false</cast_shadows>
    </light>
    <light type="directional" name="wall_fill_b">
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.3 0.31 0.32 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>-0.82 -0.41 -0.41</direction>
      <cast_shadows>false</cast_shadows>
    </light>

    <include>
      <uri>model://sauvc27_pool</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
'''


class World:
    def __init__(self, name, doc):
        self.name, self.doc = name, doc
        self.items = []         # (uri, name, (x, y, z, yaw), comment)
        self.layout = {'world': name, 'frame': 'x 沿 50 m 長邊（0 = 淺端池壁），y 沿 25 m 短邊，z 向上（0 = 水面）',
                       'props': {}}

    def add(self, uri, name, x, y, z, yaw=0.0, comment=None, kind=None):
        self.items.append((uri, name, (x, y, z, yaw), comment))
        self.layout['props'][name] = {'model': uri, 'kind': kind or uri,
                                      'pose': [round(x, 4), round(y, 4), round(z, 4), round(yaw, 4)]}

    def add_vehicle(self, x, y, yaw):
        # 船的模型名字一定要是 orca（ArduPilotPlugin、推進器 topic 用 /model/orca/...），所以不給 <name>
        self.items.append(('orca_cam', None, (x, y, -0.2, yaw),
                           'Orca + 前視、底部相機（model://orca_cam）；浮在水面，任務開始時歸零'))
        self.layout['vehicle_start'] = [round(x, 4), round(y, 4), -0.2, round(yaw, 4)]

    def sdf(self):
        out = WORLD_HEAD.format(doc=self.doc.strip('\n'), name=self.name)
        for uri, name, (x, y, z, yaw), comment in self.items:
            out += '\n'
            if comment:
                out += f'    <!-- {comment} -->\n'
            out += f'    <include>\n      <uri>model://{uri}</uri>\n'
            if name:
                out += f'      <name>{name}</name>\n'
            out += f'      {pose(x, y, z, 0, 0, yaw)}\n    </include>\n'
        out += '\n  </world>\n</sdf>\n'
        return out

    def write(self, directory, filename=None):
        os.makedirs(directory, exist_ok=True)
        filename = filename or self.name
        path = os.path.join(directory, f'{filename}.sdf')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(self.sdf())
        with open(os.path.join(directory, f'{filename}.layout.json'), 'w', encoding='utf-8') as fh:
            json.dump(self.layout, fh, ensure_ascii=False, indent=2)
            fh.write('\n')
        return path


def add_qualification_props(w):
    for i, y in enumerate(QUAL_LANES_Y, start=1):
        w.add('sauvc27_qual_start_line', f'qual_start_{i}', 0.0, y, 0.0,
              comment=f'資格賽 Arena {i}：池壁上的起點標記、10 m 外的資格賽門（門面朝 +x）',
              kind='qualification_start')
        w.add('sauvc27_qual_gate', f'qual_gate_{i}', QUAL_GATE_X, y, 0.0, math.pi / 2, kind='qualification_gate')


def qualification_world(lane=2, name='sauvc27_qualification'):
    y = QUAL_LANES_Y[lane - 1]
    w = World(name, f'''  SAUVC 2027 資格賽場地（規則書 §7、§8.1）。
  三條資格賽道在淺端，各自有池壁上 1 m 寬的起點、10 m 外從水面垂下 1 m 的資格賽門。
  船在 Arena {lane}（y = {y:g} m）的起點，船尾貼著牆、船頭朝 +x 對著門。
  資格賽：穿過門 → U-turn → 再反向穿過一次。全程不能浮出水面、不能碰池底、池壁、門。
  門所在處水深約 {depth_at(QUAL_GATE_X):.2f} m（池底由淺端 1.28 m 往深區斜下去），門下緣離池底約 {depth_at(QUAL_GATE_X) - QUAL_GATE_DEPTH:.2f} m。
  只放資格賽的道具；決賽主場地的道具在 sauvc27_finals.sdf。''')
    add_qualification_props(w)
    w.add_vehicle(0.35, y, 0.0)
    w.layout['qualification_lane'] = lane
    return w


def sample_finals(rng):
    """依規則書可變動的部分隨機抽一組決賽佈局。"""
    x0, x1 = MAIN_X
    gate_x = rng.uniform(x0 + 1.5, x1 - 1.5)
    orange = (rng.uniform(x0 + 0.5, x1 - 0.5), rng.uniform(*ORANGE_ZONE_Y))
    order = ['blue', 'red', 'red', 'red']
    rng.shuffle(order)
    pinger = rng.choice([i for i, c in enumerate(order) if c == 'red'])
    taken = [(gate_x, GATE_LINE_Y), orange, TARGET_ZONE]
    flares = {}
    for colour in ('red', 'yellow', 'blue'):
        for _ in range(500):
            p = (rng.uniform(x0 + 1.0, x1 - 1.0), rng.uniform(*FLARE_BAND_Y))
            if all(math.dist(p, q) >= 2.0 for q in taken):
                break
        else:
            raise RuntimeError('找不到放 flare 的位置')
        flares[colour] = p
        taken.append(p)
    letters = ['R', 'Y', 'B']
    rng.shuffle(letters)
    return {'gate_x': gate_x, 'orange_flare': orange, 'drum_order': order, 'pinger': pinger,
            'flares': flares, 'flare_order': '-'.join(letters)}


def finals_world(layout, name='sauvc27_finals', seed=None):
    src = f'隨機佈局 seed={seed}' if seed is not None else '預設佈局：照官方場地圖（Figure 5）的位置'
    w = World(name, f'''  SAUVC 2027 決賽場地（規則書 §8、§9）。{src}。
  船浮在出發區（{START_ZONE_SIZE * 100:.0f} × {START_ZONE_SIZE * 100:.0f} cm），船頭朝 +y。主場地在深區（水深 {DEEP_DEPTH:g} m）：
    - 導航門：y = {GATE_LINE_Y:g} m 的線上（x = {layout['gate_x']:.2f}），150 × 150 cm，立在池底，紅柱在左舷
    - 橘色 flare：y {ORANGE_ZONE_Y[0]:g}–{ORANGE_ZONE_Y[1]:g} m 的帶狀區，碰到就中止
    - target zone：綠色墊 8 × 2 m，四個桶（一藍三紅），其中一個紅桶有 pinger
    - 紅、黃、藍 flare，頂上有高爾夫球；這次的順序 {layout['flare_order']}（完成導航後才告知）
    - Surfacing Square：資格賽門旁的水面
  資格賽的門和起點也在池裡（同一個泳池），當作環境的一部分。''')
    add_qualification_props(w)
    w.add('sauvc27_surfacing_square', 'surfacing_square', *SURFACING_SQUARE, 0.0,
          comment='Surfacing Square：浮在資格賽門旁的水面', kind='surfacing_square')
    w.add('sauvc27_starting_zone', 'starting_zone', *START_ZONE, 0.0,
          comment='出發區（水面浮框）', kind='starting_zone')
    w.add_vehicle(START_ZONE[0], START_ZONE[1], math.pi / 2)

    floor = -DEEP_DEPTH
    w.add('sauvc27_gate', 'gate', layout['gate_x'], GATE_LINE_Y, floor, 0.0,
          comment='導航門：從 -y 往 +y 穿過，紅柱（port）在 -x', kind='gate')
    w.add('sauvc27_orange_flare', 'orange_flare', *layout['orange_flare'], floor,
          comment='橘色 flare（障礙物）', kind='orange_flare')
    w.add('sauvc27_target_zone', 'target_zone', *TARGET_ZONE, floor, comment='Target zone 綠色墊', kind='target_zone')
    mat_top = floor + 0.01
    for i, colour in enumerate(layout['drum_order']):
        x = TARGET_ZONE[0] + (i - 1.5) * DRUM_SPACING
        pinger = i == layout['pinger']
        w.add(f'sauvc27_drum_{colour}', f'drum_{i}', x, TARGET_ZONE[1], mat_top,
              comment=f'{colour} 桶' + ('（有 pinger）' if pinger else ''), kind=f'{colour}_drum')
        w.layout['props'][f'drum_{i}']['pinger'] = pinger
        if pinger:
            w.add('sauvc27_pinger', 'pinger', x, TARGET_ZONE[1], mat_top + 0.01, kind='pinger')
    for colour, (x, y) in layout['flares'].items():
        w.add(f'sauvc27_flare_{colour}', f'{colour}_flare', x, y, floor, comment=f'{colour} flare',
              kind=f'{colour}_flare')
    w.layout['flare_order'] = layout['flare_order']
    if seed is not None:
        w.layout['seed'] = seed
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', nargs='?', default='all', choices=['all', 'models', 'qualification', 'finals'])
    ap.add_argument('--seed', type=int, help='決賽：隨機佈局的 seed（不給 = 照官方場地圖）')
    ap.add_argument('--lane', type=int, default=2, choices=[1, 2, 3], help='資格賽：船在哪一條賽道')
    ap.add_argument('--out', help='世界檔輸出目錄（預設：預設佈局寫 sim/worlds/，其他寫 sim/worlds/generated/）')
    a = ap.parse_args()

    if a.what in ('all', 'models'):
        print('模型：')
        write_models()
    if a.what in ('all', 'qualification'):
        default = a.lane == 2
        w = qualification_world(a.lane)
        out = a.out or (WORLDS if default else os.path.join(WORLDS, 'generated'))
        path = w.write(out, None if default else f'sauvc27_qualification_lane{a.lane}')
        print(f'資格賽：{os.path.relpath(path, SIM)}')
    if a.what in ('all', 'finals'):
        if a.seed is None:
            w = finals_world(CANONICAL_FINALS)
            path = w.write(a.out or WORLDS)
        else:
            w = finals_world(sample_finals(random.Random(a.seed)), seed=a.seed)
            path = w.write(a.out or os.path.join(WORLDS, 'generated'), f'sauvc27_finals_seed{a.seed}')
        print(f'決賽：{os.path.relpath(path, SIM)}（flare 順序 {w.layout["flare_order"]}）')


if __name__ == '__main__':
    main()
