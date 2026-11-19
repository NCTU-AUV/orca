"""讀飛控 DataFlash（.BIN）成 numpy 陣列。

load(path) -> dict[訊息類型] = dict[欄位] = np.ndarray，每個類型都多一欄 t（飛控開機後秒數）。
- IMU、BARO 只留第一顆（I == 0）。
- ArduSub 4.7.2 的 RATE.TimeUS 一律是 0，改用前一筆 ATT 的時間。
- EV（事件）另外放在 'EV' 底下：163 浮出水面、164 離開水面、165 碰底、166 離開池底。
"""
import collections

import numpy as np
from pymavlink import mavutil

TYPES = ['RCOU', 'IMU', 'ATT', 'RATE', 'CTUN', 'BARO', 'MODE', 'BAT', 'MOTB', 'EV', 'ARM']

EV_SURFACED, EV_NOT_SURFACED, EV_BOTTOMED, EV_NOT_BOTTOMED = 163, 164, 165, 166


def load(path):
    m = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    rows = collections.defaultdict(list)
    last_att = 0
    while True:
        r = m.recv_match(type=TYPES)
        if r is None:
            break
        d = r.to_dict()
        t = r.get_type()
        if t == 'ATT':
            last_att = d['TimeUS']
        if t == 'RATE' and d['TimeUS'] == 0:
            d['TimeUS'] = last_att
        if t in ('IMU', 'BARO') and d.get('I', 0) != 0:
            continue
        rows[t].append(d)
    out = {}
    for t, lst in rows.items():
        keys = [k for k in lst[0] if k != 'mavpackettype' and isinstance(lst[0][k], (int, float))]
        out[t] = {k: np.array([x[k] for x in lst], float) for k in keys}
        out[t]['t'] = out[t]['TimeUS'] / 1e6
    return out
