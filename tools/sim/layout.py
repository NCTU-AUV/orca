#!/usr/bin/env python3
"""SAUVC 2027 決賽主場地的隨機佈局：給一個 seed，照規則抽出道具的位置、drum 的顏色、flare 的順序，
產生一個世界檔。只用 Python 標準函式庫，主機上直接跑。

    python3 tools/sim/layout.py sauvc_final.sdf --seed 7        # → sim/worlds/generated/sauvc_final_seed7.sdf
    python3 tools/sim/layout.py sauvc_final.sdf --seed random   # 隨機挑一個 seed
    python3 tools/sim/layout.py sauvc_final.sdf --print         # 只印範本現在的（固定）佈局

平常不用直接跑：deploy/sim 的腳本看到環境變數 SEED 就會呼叫它（deploy/sim/seed.sh），例如
    SEED=7 WORLD=sauvc_final.sdf ./mission.sh gate_pass_yaw -p depth=1.0

輸出（都在 sim/worlds/generated/，不進 git）：
    <範本>_seed<N>.sdf          世界檔。以 sim/worlds/<範本> 為底，只改下面「隨機」的那幾項，其他（註解、格式）原封不動
    <範本>_seed<N>.layout.json  這次的佈局（真值）：arena 座標的位置、drum 的顏色、flare 的順序。sim/referee.py 會把它
                                寫進紀錄（logs/referee/）
標準輸出只印世界檔相對 sim/worlds 的路徑（給腳本接），其他說明印在標準錯誤。
同一個 seed 每次產生的檔案一模一樣（Python 3 的 random.Random 對同一個整數 seed 的結果固定）。

隨機的項目，依 SAUVC 2027 Rulebook 2027.3.2（2026-10-30），座標是 arena（見 docs/sim/world-2027.md）：
    導航門        沿 y = 12 m 的線任意位置（§9.1「anywhere on a horizontal line … approximately 12m away」）
    橘色 flare    門之前、離場地側邊 4–8 m 的矩形區內任意位置（§9.1）
    紅黃藍 flare  照場地圖（Figure 5）門與標靶區之間的帶狀區（y 14.4–19.3 m）任意位置，彼此至少隔 FLARE_GAP。
                  §9.4 的文字是「主場地內任意位置」，2026-11-19 決定先照圖上的帶狀區
    drum          四個位置（綠墊上）固定，哪個是藍的、pinger 在哪個紅的隨機（§9.2）
    flare 順序    紅黃藍的排列（§9.4）。由 sim/referee.py 在過門之後發布（/comms/flare_order）
不隨機：起始區、綠墊、Surfacing Square、資格賽道、船的出發位置。主場地是 x = 22～50 m。
"""
import argparse
import json
import os
import random
import re
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
WORLDS = os.path.join(REPO, 'sim', 'worlds')
GENERATED = 'generated'

# ---- 規則（arena 座標，m）
MAIN_X = (22.0, 50.0)           # 主場地
EDGE = 0.5                      # 道具中心離主場地邊界至少這麼遠
GATE_Y = 12.0
GATE_EDGE = 1.5                 # 門中心離主場地邊界（門寬 1.5 m，柱子外緣 0.77 m）
ORANGE_Y = (4.0, 8.0)
FLARE_Y = (14.4, 19.3)
FLARE_GAP = 2.0                 # 紅黃藍 flare 彼此的最小距離（撞一支不會碰到另一支，船長約 0.75 m）
COLOURS = ('red', 'yellow', 'blue')
CODE = {'red': 'R', 'yellow': 'Y', 'blue': 'B'}
DRUMS = ('drum_1', 'drum_2', 'drum_3', 'drum_4')

TEMPLATES = ('sauvc_final.sdf',)    # 有隨機佈局的世界


def includes(text):
    """世界檔裡的 <include>（以 <uri> 開頭的才算：註解裡的「<include>」字樣不算）。"""
    return list(re.finditer(r'<include>\s*<uri>.*?</include>', text, flags=re.S))


def find(text, uri, name=None):
    hits = [m for m in includes(text)
            if f'<uri>model://{uri}</uri>' in m.group(0) and (name is None or f'<name>{name}</name>' in m.group(0))]
    if len(hits) != 1:
        raise SystemExit(f'範本裡 model://{uri}' + (f'（{name}）' if name else '') + f' 有 {len(hits)} 個，應該剛好 1 個')
    return hits[0]


def pose_xy(block):
    m = re.search(r'<pose relative_to="arena">([^<]+)</pose>', block)
    v = [float(x) for x in m.group(1).split()]
    return v[0], v[1]


def extract(text):
    """範本現在的佈局（固定佈局）。"""
    gx, gy = pose_xy(find(text, 'gate').group(0))
    ox, oy = pose_xy(find(text, 'orange_flare').group(0))
    flares = {c: dict(zip('xy', pose_xy(find(text, f'flare_{c}').group(0)))) for c in COLOURS}
    drums = []
    for name in DRUMS:
        m = next(m for m in includes(text) if f'<name>{name}</name>' in m.group(0))
        kind = re.search(r'<uri>model://([^<]+)</uri>', m.group(0)).group(1)
        x, y = pose_xy(m.group(0))
        drums.append(dict(name=name, kind=kind, x=x, y=y))
    order = re.search(r'<orca:referee[^>]*flare_order="([^"]*)"', text).group(1).split()
    return dict(gate=dict(x=gx, y=gy), orange_flare=dict(x=ox, y=oy), flares=flares, drums=drums,
                flare_order=order, flare_order_code='-'.join(CODE[c] for c in order))


def sample(seed, fixed):
    """照規則抽一個佈局。fixed：範本的佈局（drum 的四個位置取自它）。"""
    rng = random.Random(seed)
    r2 = lambda v: round(v, 2)          # noqa: E731  位置取到公分
    gate = dict(x=r2(rng.uniform(MAIN_X[0] + GATE_EDGE, MAIN_X[1] - GATE_EDGE)), y=GATE_Y)
    orange = dict(x=r2(rng.uniform(MAIN_X[0] + EDGE, MAIN_X[1] - EDGE)), y=r2(rng.uniform(*ORANGE_Y)))
    flares = {}
    for c in COLOURS:
        for _ in range(1000):
            p = dict(x=r2(rng.uniform(MAIN_X[0] + EDGE, MAIN_X[1] - EDGE)), y=r2(rng.uniform(*FLARE_Y)))
            if all((p['x'] - q['x']) ** 2 + (p['y'] - q['y']) ** 2 >= FLARE_GAP ** 2 for q in flares.values()):
                flares[c] = p
                break
        else:
            raise SystemExit('抽不到彼此夠遠的 flare 位置')
    blue = rng.randrange(4)
    pinger = rng.choice([i for i in range(4) if i != blue])
    drums = [dict(d, kind='drum_blue' if i == blue else 'drum_red_pinger' if i == pinger else 'drum_red')
             for i, d in enumerate(fixed['drums'])]
    order = list(COLOURS)
    rng.shuffle(order)
    return dict(gate=gate, orange_flare=orange, flares=flares, drums=drums,
                flare_order=order, flare_order_code='-'.join(CODE[c] for c in order))


def set_xy(text, match, x, y):
    block = match.group(0)
    new = re.sub(r'(<pose relative_to="arena">)\s*\S+\s+\S+',
                 lambda m: f'{m.group(1)}{x:g} {y:g}', block, count=1)
    return text[:match.start()] + new + text[match.end():]


def apply(text, layout, seed, template):
    text = set_xy(text, find(text, 'gate'), layout['gate']['x'], layout['gate']['y'])
    text = set_xy(text, find(text, 'orange_flare'), layout['orange_flare']['x'], layout['orange_flare']['y'])
    for c in COLOURS:
        text = set_xy(text, find(text, f'flare_{c}'), layout['flares'][c]['x'], layout['flares'][c]['y'])
    for d in layout['drums']:
        m = next(m for m in includes(text) if f'<name>{d["name"]}</name>' in m.group(0))
        block = re.sub(r'<uri>model://[^<]+</uri>', f'<uri>model://{d["kind"]}</uri>', m.group(0), count=1)
        text = text[:m.start()] + block + text[m.end():]
    text, n = re.subn(r'(<orca:referee[^>]*flare_order=")[^"]*(")',
                      lambda m: m.group(1) + ' '.join(layout['flare_order']) + m.group(2), text, count=1)
    assert n == 1
    head = (f'<!--\n  由 tools/sim/layout.py 從 sim/worlds/{template} 產生（seed={seed}），不要手改；要改規則改 layout.py。\n'
            f'  {summary(layout)}\n  佈局的真值在旁邊的 .layout.json。下面是範本原本的說明。\n-->\n')
    decl = re.match(r'<\?xml[^>]*\?>\s*', text)
    return text[:decl.end()] + head + text[decl.end():] if decl else head + text


def summary(layout):
    f = layout['flares']
    blue = next(d['name'] for d in layout['drums'] if d['kind'] == 'drum_blue')
    pinger = next(d['name'] for d in layout['drums'] if d['kind'] == 'drum_red_pinger')
    return (f'門 x={layout["gate"]["x"]:g}，橘色 flare ({layout["orange_flare"]["x"]:g}, {layout["orange_flare"]["y"]:g})，'
            f'紅 ({f["red"]["x"]:g}, {f["red"]["y"]:g})、黃 ({f["yellow"]["x"]:g}, {f["yellow"]["y"]:g})、'
            f'藍 ({f["blue"]["x"]:g}, {f["blue"]["y"]:g})，藍 drum {blue}、pinger {pinger}，flare 順序 {layout["flare_order_code"]}')


def generate(template, seed):
    """產生世界檔與 layout.json，回傳世界檔相對 sim/worlds 的路徑。"""
    with open(os.path.join(WORLDS, template)) as f:
        text = f.read()
    layout = sample(seed, extract(text))
    stem = os.path.splitext(template)[0]
    rel = f'{GENERATED}/{stem}_seed{seed}.sdf'
    os.makedirs(os.path.join(WORLDS, GENERATED), exist_ok=True)
    with open(os.path.join(WORLDS, rel), 'w') as f:
        f.write(apply(text, layout, seed, template))
    with open(os.path.join(WORLDS, rel[:-4] + '.layout.json'), 'w') as f:
        json.dump(dict(template=template, seed=seed, world=rel, frame='arena', **layout), f, ensure_ascii=False, indent=1)
    return rel, layout


def main():
    ap = argparse.ArgumentParser(description='SAUVC 2027 決賽主場地的隨機佈局')
    ap.add_argument('world', help='範本世界檔（sim/worlds 底下的檔名），目前只有 sauvc_final.sdf')
    ap.add_argument('--seed', help='整數（≥ 0），或 random')
    ap.add_argument('--print', action='store_true', help='只印範本的固定佈局，不產生檔案')
    a = ap.parse_args()
    template = os.path.basename(a.world)
    if template not in TEMPLATES or os.path.dirname(a.world) not in ('', '.'):
        print(f'{a.world} 沒有隨機佈局。目前只有決賽主場地：WORLD=sauvc_final.sdf SEED=<數字> ./mission.sh …',
              file=sys.stderr)
        return 2
    if a.print:
        with open(os.path.join(WORLDS, template)) as f:
            print(json.dumps(extract(f.read()), ensure_ascii=False, indent=1))
        return 0
    if a.seed is None:
        ap.error('要給 --seed（數字或 random）或 --print')
    if a.seed == 'random':
        seed = random.SystemRandom().randrange(1, 100000)
    elif a.seed.isdigit():
        seed = int(a.seed)
    else:
        ap.error(f'--seed 要是 ≥ 0 的整數或 random，收到 {a.seed!r}')
    rel, layout = generate(template, seed)
    print(f'隨機佈局 seed={seed}：{summary(layout)}', file=sys.stderr)
    print(f'  世界檔 sim/worlds/{rel}（重現：SEED={seed}）', file=sys.stderr)
    print(rel)
    return 0


if __name__ == '__main__':
    sys.exit(main())
