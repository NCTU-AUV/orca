#!/usr/bin/env python3
"""比較兩次偵測驗證的結果（evaluate.py 產生的 summary.json）。主機上直接跑，只需要 Python 3：

    python3 tools/sim_detect/compare.py deploy/sim/logs/detect/<舊> deploy/sim/logs/detect/<新>

例如換了權重、改了水下效果、改了道具模型之後，看每個道具的偵測率、最遠偵測距離變多少。
兩次的位姿清單不一樣時，數字還是印得出來，但不要直接比。
"""
import json
import os
import sys

NAMES_ZH = {
    'gate': '導航門', 'q_gate': '資格賽門', 'orange_flare': '橘色 flare', 'flare_red': '紅 flare',
    'flare_yellow': '黃 flare', 'flare_blue': '藍 flare', 'drum_red': '紅 drum', 'drum_blue': '藍 drum',
    'surfacing_square': 'Surfacing Square',
}


def load(path):
    with open(os.path.join(path, 'summary.json')) as f:
        s = json.load(f)
    with open(os.path.join(path, 'meta.json')) as f:
        m = json.load(f)
    return s, m


def fmt(x, pct=False):
    if x is None:
        return '—'
    return f'{100 * x:.0f}%' if pct else f'{x:g}'


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    (a, ma), (b, mb) = load(sys.argv[1]), load(sys.argv[2])
    print(f'A = {a["run"]}（{os.path.basename(a["weights"])}）\nB = {b["run"]}（{os.path.basename(b["weights"])}）')
    for key in ('distances', 'bearings', 'depths', 'world'):
        if ma.get(key) != mb.get(key):
            print(f'注意：兩次的 {key} 不一樣（{ma.get(key)} vs {mb.get(key)}）')
    print()
    print(f'{"道具":<18}{"水下":<4}{"偵測率 A→B":<16}{"平均信心 A→B":<16}{"最遠 m A→B":<14}{"連續 m A→B":<14}')
    for p in [p for p in a['props'] if p in b['props']]:
        for c in ('off', 'on'):
            x, y = a['props'][p][c], b['props'][p][c]
            print(f'{NAMES_ZH.get(p, p):<18}{"開" if c == "on" else "關":<4}'
                  f'{fmt(x["rate"], True) + " → " + fmt(y["rate"], True):<16}'
                  f'{fmt(x["mean_conf"]) + " → " + fmt(y["mean_conf"]):<16}'
                  f'{fmt(x["max_dist"]) + " → " + fmt(y["max_dist"]):<14}'
                  f'{fmt(x["continuous_dist"]) + " → " + fmt(y["continuous_dist"]):<14}')


if __name__ == '__main__':
    main()
