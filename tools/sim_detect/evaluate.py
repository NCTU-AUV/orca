#!/usr/bin/env python3
"""偵測驗證，第二步：用 RF-DETR 跑 capture.py 拍的影像，算每個道具「在多遠、什麼角度、什麼深度偵測得到」。

用 deploy/sim/detect_sweep.sh 跑（detect_eval 服務，orca-perception image，跟實機偵測節點同一套
rfdetr 與權重）。也可以單獨對已經拍好的資料重算（例如換了權重）：

    docker compose --profile sweep run --rm detect_eval python3 -u /orca/tools/sim_detect/evaluate.py /out/<RUN>

輸入：<RUN>/frames.csv 與 frames/（capture.py 產生）
輸出（寫在同一個資料夾）：
    detections.csv   每張影像的所有偵測（分數 ≥ 0.05）
    results.csv      每張影像的判定：有沒有偵測到、最高信心、IoU、被認成什麼
    summary.json     統計（compare.py 拿它比較兩次的結果）
    summary.md       給人看的報告
    samples/<道具>.jpg   幾張代表性的影像（上：水下效果關，下：開），畫了真值框（黃）與偵測框

判定規則：
    看得到     真值框（segmentation camera，已扣掉被遮住的部分）至少 MIN_VISIBLE_PX 個像素。
               看不到的影像（出了畫面、被擋住、太遠只剩幾個像素）不列入偵測率，另外計數。
    偵測到     有一個「期望類別」的框，分數 ≥ THRESHOLD（跟實機偵測節點的預設 0.4 一樣），
               而且跟真值框 IoU ≥ 0.3，或框的中心落在真值框（四周放寬 25%、至少 8 px）裡。
               細的東西（flare 的桿子、門柱）在遠處只有幾個像素寬，IoU 很容易偏低，所以用中心點補。
    平均信心   偵測到的那些影像裡，配對到的框的分數平均。
    最遠偵測距離   有任何一張偵測到的最大距離。
    連續偵測距離   從第一個偵測率 ≥ 50% 的距離往外，每個距離都 ≥ 50% 能到多遠（太近時道具超出畫面，偵測率本來就低，
               所以開頭不到 50% 的距離跳過；報告寫成「起點～終點」）。
    資格賽門（q_gate）另外算「認成 gate 也算」的寬鬆版本：過門的行為樹兩類都接受。
    被認成別的類別   分數 ≥ 0.4、跟真值框 IoU ≥ 0.3、但類別不對的框（只用 IoU：門、浮框的真值框很大，
               後面的 drum、flare 的框中心也會落在裡面，那不算認錯）。
    Surfacing Square 模型沒有這一類，偵測率一定是 0；報告列出它被認成什麼。
"""
import csv
import json
import os
import sys
import time
from collections import Counter, defaultdict

import cv2
import numpy as np
from PIL import Image

THRESHOLD = 0.4
LOW = 0.05
MIN_VISIBLE_PX = 20
IOU_MIN = 0.3
NAMES_ZH = {
    'gate': '導航門', 'q_gate': '資格賽門', 'orange_flare': '橘色 flare', 'flare_red': '紅 flare',
    'flare_yellow': '黃 flare', 'flare_blue': '藍 flare', 'drum_red': '紅 drum', 'drum_blue': '藍 drum',
    'surfacing_square': 'Surfacing Square',
}
CONDS = ('off', 'on')
COND_ZH = {'off': '關', 'on': '開'}


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def centre_inside(det, truth):
    w, h = truth[2] - truth[0], truth[3] - truth[1]
    mx, my = max(8.0, 0.25 * w), max(8.0, 0.25 * h)
    cx, cy = (det[0] + det[2]) / 2, (det[1] + det[3]) / 2
    return truth[0] - mx <= cx <= truth[2] + mx and truth[1] - my <= cy <= truth[3] + my


def overlaps(det, truth):
    return iou(det, truth) >= IOU_MIN or centre_inside(det, truth)


def run_model(run_dir, frames):
    from rfdetr import RFDETRLarge
    weights = os.environ.get('WEIGHTS', '/model/checkpoint_best_total.pth')
    print(f'載入 {weights} …', flush=True)
    model = RFDETRLarge(pretrain_weights=weights, num_classes=8)
    names = list(model.class_names)
    out = []
    t0 = time.time()
    for i, f in enumerate(frames):
        img = Image.open(os.path.join(run_dir, f['file'])).convert('RGB')
        det = model.predict(img, threshold=LOW)
        for (x1, y1, x2, y2), cid, conf in zip(det.xyxy, det.class_id, det.confidence):
            out.append(dict(id=f['id'], cond=f['cond'], cls=names[int(cid)], score=round(float(conf), 4),
                            x1=round(float(x1), 1), y1=round(float(y1), 1), x2=round(float(x2), 1), y2=round(float(y2), 1)))
        if (i + 1) % 200 == 0:
            print(f'  {i + 1}/{len(frames)}（{(i + 1) / (time.time() - t0):.1f} 張/秒）', flush=True)
    return out, names, weights


def judge(frames, dets):
    by_frame = defaultdict(list)
    for d in dets:
        by_frame[(d['id'], d['cond'])].append(d)
    results = []
    for f in frames:
        labels = [x for x in f['labels'].split('|') if x]
        alt = [x for x in f['alt_labels'].split('|') if x]
        visible = int(f['visible_px']) >= MIN_VISIBLE_PX
        truth = tuple(float(f[k]) for k in ('x1', 'y1', 'x2', 'y2')) if f['x1'] != '' else None
        best, best_alt, best_iou, confused = 0.0, 0.0, 0.0, Counter()
        for d in by_frame[(f['id'], f['cond'])]:
            box = (d['x1'], d['y1'], d['x2'], d['y2'])
            if truth is None or not overlaps(box, truth):
                continue
            if d['cls'] in labels and d['score'] > best:
                best, best_iou = d['score'], iou(box, truth)
            if d['cls'] in labels + alt:
                best_alt = max(best_alt, d['score'])
            # 認錯：只算跟真值框真的重疊（IoU）的框。門、浮框的真值框很大，後面的 drum、flare 中心也會落在裡面
            if d['cls'] not in labels and d['score'] >= THRESHOLD and iou(box, truth) >= IOU_MIN:
                confused[d['cls']] += 1
        results.append(dict(id=f['id'], cond=f['cond'], prop=f['prop'], distance=float(f['distance']),
                            bearing=float(f['bearing']), depth=float(f['depth']), visible_px=int(f['visible_px']),
                            visible=visible, hit=visible and best >= THRESHOLD, best_conf=round(best, 4),
                            iou=round(best_iou, 3), hit_alt=visible and best_alt >= THRESHOLD,
                            confused='|'.join(f'{k}:{v}' for k, v in sorted(confused.items()))))
    return results


def stats(rows):
    vis = [r for r in rows if r['visible']]
    hits = [r for r in vis if r['hit']]
    s = dict(frames=len(rows), visible=len(vis), hits=len(hits),
             rate=round(len(hits) / len(vis), 3) if vis else None,
             mean_conf=round(float(np.mean([r['best_conf'] for r in hits])), 3) if hits else None,
             max_dist=max((r['distance'] for r in hits), default=None),
             rate_alt=round(sum(r['hit_alt'] for r in vis) / len(vis), 3) if vis else None)
    by_d = defaultdict(list)
    for r in vis:
        by_d[r['distance']].append(r['hit'])
    # 連續偵測距離：太近時道具超出畫面、偵測率本來就低，所以從第一個 ≥ 50% 的距離開始，往外到第一次掉到 50% 以下為止
    start = cont = None
    for d in sorted(by_d):
        ok = np.mean(by_d[d]) >= 0.5
        if start is None:
            if ok:
                start = cont = d
        elif ok:
            cont = d
        else:
            break
    s['continuous_from'], s['continuous_dist'] = start, cont
    s['by_distance'] = {str(d): dict(visible=len(v), rate=round(float(np.mean(v)), 3)) for d, v in sorted(by_d.items())}
    for key in ('depth', 'bearing'):
        g = defaultdict(list)
        for r in vis:
            g[r[key]].append(r['hit'])
        s[f'by_{key}'] = {str(k): dict(visible=len(v), rate=round(float(np.mean(v)), 3)) for k, v in sorted(g.items())}
    conf = Counter()
    for r in rows:
        for item in filter(None, r['confused'].split('|')):
            k, v = item.split(':')
            conf[k] += int(v)
    s['confused_as'] = dict(conf.most_common())
    return s


def pct(x):
    return '—' if x is None else f'{100 * x:.0f}%'


def num(x, fmt='{:g}'):
    return '—' if x is None else fmt.format(x)


def rng(st):
    if st.get('continuous_dist') is None:
        return '—'
    return f'{st["continuous_from"]:g}～{st["continuous_dist"]:g}'


def write_report(run_dir, meta, summary, props):
    L = []
    L.append(f'# 偵測驗證：{meta["run"]}\n')
    L.append(f'- 世界：`{meta["world"]}`，拍攝時間 {meta["created"]}' + (f'，程式版本 `{meta["git"]}`' if meta.get('git') else ''))
    L.append(f'- 權重：`{summary["weights"]}`（{len(summary["class_names"])} 類），偵測門檻 {THRESHOLD}')
    L.append(f'- 位姿：距離 {meta["distances"]} m × 角度 {meta["bearings"]}° × 深度 {meta["depths"]} m，'
             f'共 {meta["poses"]} 個（出了泳池、低於池底、太靠近別的道具的跳過）。每個位姿拍「水下效果關」與「開」各一張')
    uw = meta['underwater']
    r3 = lambda v: [round(x, 3) for x in v]     # noqa: E731
    L.append(f'- 水下效果（開）：強度 {uw["strength"]}，衰減 {r3(uw["atten"])}、散射 {r3(uw["scatter"])}、水色 {r3(uw["veil"])}、'
             f'模糊 {uw["blur"]} px、噪點 {uw["noise"]}（sim/underwater.py）')
    L.append(f'- 「看得到」= 真值框至少 {MIN_VISIBLE_PX} 個像素；偵測率只算看得到的影像。判定規則見 tools/sim_detect/evaluate.py 檔頭\n')

    L.append('## 總覽\n')
    L.append('| 道具 | 期望類別 | 看得到的張數（關／開） | 偵測率（關 → 開） | 平均信心（關 → 開） | 最遠偵測距離 m（關 → 開） | 連續偵測距離 m（關 → 開） |')
    L.append('|---|---|---|---|---|---|---|')
    for p in props:
        a, b = summary['props'][p]['off'], summary['props'][p]['on']
        lab = ', '.join(meta['props'][p]['labels']) or '（沒有這一類）'
        L.append(f'| {NAMES_ZH.get(p, p)} | {lab} | {a["visible"]}／{b["visible"]} | {pct(a["rate"])} → {pct(b["rate"])} | '
                 f'{num(a["mean_conf"], "{:.2f}")} → {num(b["mean_conf"], "{:.2f}")} | {num(a["max_dist"])} → {num(b["max_dist"])} | '
                 f'{rng(a)} → {rng(b)} |')
    if 'q_gate' in props:
        a, b = summary['props']['q_gate']['off'], summary['props']['q_gate']['on']
        L.append(f'\n資格賽門認成 `gate` 也算（過門的行為樹兩類都接受）：偵測率 {pct(a["rate_alt"])} → {pct(b["rate_alt"])}。')
    L.append('\n「連續偵測距離」：從第一個偵測率 ≥ 50% 的距離往外，每個距離都 ≥ 50% 的範圍（太近時道具超出畫面，開頭不到 50% 的跳過）。\n')

    L.append('## 依距離\n')
    dists = [str(float(d)) for d in meta['distances']]
    L.append('偵測率，格式「關 → 開」；空白 = 那個距離沒有看得到的影像。\n')
    L.append('| 道具 | ' + ' | '.join(f'{float(d):g} m' for d in dists) + ' |')
    L.append('|---|' + '---|' * len(dists))
    for p in props:
        cells = []
        for d in dists:
            a = summary['props'][p]['off']['by_distance'].get(d)
            b = summary['props'][p]['on']['by_distance'].get(d)
            cells.append('' if not a and not b else f'{pct(a["rate"]) if a else "—"} → {pct(b["rate"]) if b else "—"}')
        L.append(f'| {NAMES_ZH.get(p, p)} | ' + ' | '.join(cells) + ' |')

    for key, title, unit in (('depth', '依相機深度', ' m'), ('bearing', '依觀看角度', '°')):
        vals = [str(float(v)) for v in meta[key + 's']]
        L.append(f'\n## {title}\n')
        L.append('| 道具 | ' + ' | '.join(f'{float(v):g}{unit}' for v in vals) + ' |')
        L.append('|---|' + '---|' * len(vals))
        for p in props:
            cells = []
            for v in vals:
                a = summary['props'][p]['off'][f'by_{key}'].get(v)
                b = summary['props'][p]['on'][f'by_{key}'].get(v)
                cells.append('' if not a and not b else f'{pct(a["rate"]) if a else "—"} → {pct(b["rate"]) if b else "—"}')
            L.append(f'| {NAMES_ZH.get(p, p)} | ' + ' | '.join(cells) + ' |')

    L.append('\n## 被認成別的類別\n')
    L.append('分數 ≥ 0.4、跟道具的真值框 IoU ≥ 0.3、但類別不是期望的那一類（同一張影像可能不只一個）。\n')
    L.append('| 道具 | 水下效果關 | 水下效果開 |')
    L.append('|---|---|---|')
    for p in props:
        cells = []
        for c in CONDS:
            cf = summary['props'][p][c]['confused_as']
            cells.append(', '.join(f'{k} × {v}' for k, v in cf.items()) or '—')
        L.append(f'| {NAMES_ZH.get(p, p)} | {cells[0]} | {cells[1]} |')
    L.append('\n代表性影像：`samples/<道具>.jpg`（上排水下效果關、下排開；黃框 = 真值，其他顏色 = 偵測）。')
    with open(os.path.join(run_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


def samples(run_dir, frames, dets, props, meta):
    """每個道具一張：同一個角度、深度，幾個距離，上排關、下排開。"""
    os.makedirs(os.path.join(run_dir, 'samples'), exist_ok=True)
    by_frame = defaultdict(list)
    for d in dets:
        if d['score'] >= THRESHOLD:
            by_frame[(d['id'], d['cond'])].append(d)
    want_d = [d for d in (2, 5, 10, 15) if d in meta['distances']] or meta['distances'][:4]
    for p in props:
        rows = [f for f in frames if f['prop'] == p]
        if not rows:
            continue
        depth = min({float(f['depth']) for f in rows}, key=lambda v: abs(v - 1.0))
        grid = []
        for c in CONDS:
            tiles = []
            for d in want_d:
                cand = [f for f in rows if f['cond'] == c and float(f['distance']) == d and float(f['depth']) == depth
                        and float(f['bearing']) == 0]
                if not cand:
                    tiles.append(np.zeros((240, 320, 3), np.uint8))
                    continue
                f = cand[0]
                im = cv2.imread(os.path.join(run_dir, f['file']))
                if f['x1'] != '':
                    cv2.rectangle(im, (int(f['x1']), int(f['y1'])), (int(f['x2']), int(f['y2'])), (0, 255, 255), 1)
                for d2 in by_frame[(f['id'], c)]:
                    cv2.rectangle(im, (int(d2['x1']), int(d2['y1'])), (int(d2['x2']), int(d2['y2'])), (255, 80, 255), 2)
                    cv2.putText(im, f'{d2["cls"]} {d2["score"]:.2f}', (int(d2['x1']), max(12, int(d2['y1']) - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 80, 255), 1, cv2.LINE_AA)
                cv2.putText(im, f'{d:g} m  underwater {c}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(im, f'{d:g} m  underwater {c}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                tiles.append(cv2.resize(im, (320, 240), interpolation=cv2.INTER_AREA))
            grid.append(np.hstack(tiles))
        cv2.imwrite(os.path.join(run_dir, 'samples', f'{p}.jpg'), np.vstack(grid), [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join('/out', os.environ.get('RUN', ''))
    with open(os.path.join(run_dir, 'meta.json')) as f:
        meta = json.load(f)
    with open(os.path.join(run_dir, 'frames.csv')) as f:
        frames = list(csv.DictReader(f))
    for f in frames:
        f['id'] = int(f['id'])
    dets, names, weights = run_model(run_dir, frames)
    with open(os.path.join(run_dir, 'detections.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['id', 'cond', 'cls', 'score', 'x1', 'y1', 'x2', 'y2'])
        w.writeheader()
        w.writerows(dets)
    results = judge(frames, dets)
    with open(os.path.join(run_dir, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(results[0]))
        w.writeheader()
        w.writerows(results)
    props = [p for p in meta['props'] if any(r['prop'] == p for r in results)]
    summary = dict(run=meta['run'], weights=weights, class_names=names, threshold=THRESHOLD, min_visible_px=MIN_VISIBLE_PX,
                   evaluated=time.strftime('%Y-%m-%d %H:%M:%S'),
                   props={p: {c: stats([r for r in results if r['prop'] == p and r['cond'] == c]) for c in CONDS} for p in props})
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    write_report(run_dir, meta, summary, props)
    samples(run_dir, frames, dets, props, meta)
    print(f'完成：{run_dir}/summary.md', flush=True)


if __name__ == '__main__':
    main()
