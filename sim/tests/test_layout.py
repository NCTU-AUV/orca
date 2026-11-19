#!/usr/bin/env python3
"""tools/sim/layout.py（隨機佈局）的單元測試。只用 Python 標準函式庫，不需要 Gazebo、ROS：

    python3 sim/tests/test_layout.py        # CI 也跑這個
    python3 -m pytest -q sim/tests/test_layout.py

檢查：很多個 seed 抽出來的佈局都在規則的範圍內、道具不重疊；同一個 seed 每次產生的世界檔一模一樣；
世界檔只改了該改的地方，而且是合法的 XML；範本本身（固定佈局）不受影響。
"""
import json
import math
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'tools', 'sim'))
import layout as L  # noqa: E402

TEMPLATE = 'sauvc_final.sdf'
SEEDS = range(500)


def template_text():
    with open(os.path.join(L.WORLDS, TEMPLATE)) as f:
        return f.read()


def test_rules_hold_for_many_seeds():
    fixed = L.extract(template_text())
    for seed in SEEDS:
        lay = L.sample(seed, fixed)
        g, o = lay['gate'], lay['orange_flare']
        assert g['y'] == L.GATE_Y and L.MAIN_X[0] + L.GATE_EDGE <= g['x'] <= L.MAIN_X[1] - L.GATE_EDGE, (seed, g)
        assert L.MAIN_X[0] + L.EDGE <= o['x'] <= L.MAIN_X[1] - L.EDGE, (seed, o)
        assert L.ORANGE_Y[0] <= o['y'] <= L.ORANGE_Y[1] < g['y'], (seed, o)       # 在門之前
        fl = lay['flares']
        assert set(fl) == {'red', 'yellow', 'blue'}
        for c, p in fl.items():
            assert L.MAIN_X[0] + L.EDGE <= p['x'] <= L.MAIN_X[1] - L.EDGE, (seed, c, p)
            assert L.FLARE_Y[0] <= p['y'] <= L.FLARE_Y[1], (seed, c, p)
        pts = list(fl.values())
        for i in range(3):
            for j in range(i + 1, 3):
                assert math.dist((pts[i]['x'], pts[i]['y']), (pts[j]['x'], pts[j]['y'])) >= L.FLARE_GAP - 1e-9, seed
        kinds = [d['kind'] for d in lay['drums']]
        assert kinds.count('drum_blue') == 1 and kinds.count('drum_red_pinger') == 1 and kinds.count('drum_red') == 2, kinds
        assert [(d['name'], d['x'], d['y']) for d in lay['drums']] == [(d['name'], d['x'], d['y']) for d in fixed['drums']]
        assert sorted(lay['flare_order']) == ['blue', 'red', 'yellow'], lay['flare_order']
        assert lay['flare_order_code'] == '-'.join(L.CODE[c] for c in lay['flare_order'])


def test_every_choice_shows_up():
    """抽很多次，每一種 drum 顏色的位置、每一種順序都會出現（不是偏到某幾種）。"""
    fixed = L.extract(template_text())
    blue, pinger, orders = set(), set(), set()
    for seed in SEEDS:
        lay = L.sample(seed, fixed)
        blue.add(next(d['name'] for d in lay['drums'] if d['kind'] == 'drum_blue'))
        pinger.add(next(d['name'] for d in lay['drums'] if d['kind'] == 'drum_red_pinger'))
        orders.add(lay['flare_order_code'])
    assert blue == set(L.DRUMS) and pinger == set(L.DRUMS), (blue, pinger)
    assert len(orders) == 6, orders


def test_same_seed_same_world_and_only_layout_changes():
    text = template_text()
    fixed = L.extract(text)
    a = L.apply(text, L.sample(7, fixed), 7, TEMPLATE)
    b = L.apply(text, L.sample(7, fixed), 7, TEMPLATE)
    c = L.apply(text, L.sample(8, fixed), 8, TEMPLATE)
    assert a == b and a != c
    ET.fromstring(a.encode())                                       # 合法的 XML
    lay = L.sample(7, fixed)
    got = L.extract(a)                                              # 從產生的世界檔讀回來，要跟抽出來的一樣
    for key in ('gate', 'orange_flare', 'flares', 'drums', 'flare_order'):
        assert got[key] == lay[key], (key, got[key], lay[key])
    # 除了開頭的說明與那幾行，其他一字不差
    changed = [l for l in a.splitlines() if l not in set(text.splitlines())]
    for line in changed:
        assert ('relative_to="arena"' in line or '<orca:referee' in line or 'layout.py' in line
                or line.strip() in ('<!--', '-->') or '佈局' in line or line.startswith('  門 x=')), line


def test_generate_writes_world_and_layout():
    with tempfile.TemporaryDirectory() as tmp:
        worlds = L.WORLDS
        os.makedirs(os.path.join(tmp, 'w'))
        with open(os.path.join(tmp, 'w', TEMPLATE), 'w') as f:
            f.write(template_text())
        L.WORLDS = os.path.join(tmp, 'w')
        try:
            rel, lay = L.generate(TEMPLATE, 42)
            assert rel == 'generated/sauvc_final_seed42.sdf'
            with open(os.path.join(L.WORLDS, rel[:-4] + '.layout.json')) as f:
                truth = json.load(f)
            assert truth['seed'] == 42 and truth['world'] == rel and truth['flare_order'] == lay['flare_order']
            with open(os.path.join(L.WORLDS, rel)) as f:
                assert re.search(r'flare_order="' + ' '.join(lay['flare_order']) + '"', f.read())
        finally:
            L.WORLDS = worlds


def test_template_layout_is_the_documented_fixed_layout():
    fixed = L.extract(template_text())
    assert fixed['gate'] == {'x': 31.4, 'y': 12.0}
    assert fixed['flare_order'] == ['red', 'blue', 'yellow']
    assert [d['kind'] for d in fixed['drums']] == ['drum_blue', 'drum_red', 'drum_red_pinger', 'drum_red']


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print(f'ok  {t.__name__}')
