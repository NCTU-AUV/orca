#!/usr/bin/env python3
"""SolidWorks 組合件（.SLDASM）→ 模擬用的 STL 與各零件姿態。不需要 SolidWorks。

    python3 tools/cad/sldasm_mesh.py <組合件.SLDASM> <輸出資料夾> [--name 零件名=檔名 ...] [--max-tris 12000]

SolidWorks 2015 之後的檔案是一串用 raw deflate 壓縮的區段，從裡面拿兩樣東西：
- swXmlContents/COMPINSTANCETREE：每個零件實例的 4x4 變換（列向量、平移在最後一列，單位 m）。
- FaceTessellations/：SolidWorks 存檔時順便存的顯示網格。每個面是
  [4,8,2,K] + K 條 triangle strip 的長度、[12,100,2,n] + n 個頂點、[12,100,2,n] + n 個法向量（float32，零件座標）。
  同一個零件的多個實例共用一份網格。
零件網格用 vertex clustering 減面到 --max-tris 以下，以零件自己的座標輸出成 binary STL（一個零件一個檔）。

座標換算：組合件 → 模擬機體 FLU（x 前、y 左、z 上）。預設照 ORCA AUV 5.0 的組合件：
船頭 = 組合件 +Z、上 = +Y（SolidWorks 預設的上視方向），左 = +X；原點 = 整台網格外框的中心。
船頭朝哪一端從 CAD 看不出來（前後對稱），換方向用 --fwd、--up。

印出每個實例在 FLU 的 pose（x y z roll pitch yaw，SDF 的格式），以及 T200 推進器的中心（外框中心）與軸向，
貼進 sim/models/orca/model.sdf。外框中心只是幾何中心，不是質心。
"""
import argparse
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
import zlib

import numpy as np

SW = '{http://www.solidworks.com/sw2003/schema}'
AXES = {'+X': (0, 1), '-X': (0, -1), '+Y': (1, 1), '-Y': (1, -1), '+Z': (2, 1), '-Z': (2, -1)}
DEFAULT_NAMES = ['外殼側板(重作版)=side_panel', 'T200-THRUSTER-R1 - 複製=t200', '艙體壓克力圓柱=tube',
                 '艙體固定環(176mm)=ring']


def sections(b):
    """{區段名稱: 解壓後的內容}。名稱用 nibble 對調混淆；同名的取最大的那份。"""
    out = {}
    for m in re.finditer(re.escape(b'\x14\x00\x06\x00\x08\x00'), b):
        o = m.start()
        _, _, comp, _, nl = struct.unpack_from('<IIIII', b, o + 6)
        if not 0 < nl < 400:
            continue
        name = bytes(((c << 4) | (c >> 4)) & 0xff for c in b[o + 26:o + 26 + nl]).decode('utf-8', 'replace')
        try:
            data = zlib.decompressobj(-15).decompress(b[o + 26 + nl:o + 26 + nl + comp])
        except zlib.error:
            continue
        if len(data) > len(out.get(name, b'')):
            out[name] = data
    return out


def utf16_strings(b):
    """Directory 裡的字串：ff fe ff + 長度（字元數）+ UTF-16LE。回傳 [(起點, 結尾, 字串)]。"""
    res = []
    for m in re.finditer(rb'\xff\xfe\xff(.)', b, re.S):
        n = m.group(1)[0]
        res.append((m.start(), m.end() + 2 * n, b[m.end():m.end() + 2 * n].decode('utf-16le', 'replace')))
    return res


def tessellation(b, start, end):
    """stream 的 [start, end) 解成三角形 (N, 3, 3)，朝向照頂點法向量（朝外）。"""
    tris = []
    for m in re.finditer(re.escape(struct.pack('<iii', 4, 8, 2)), b[:end]):
        o = m.start()
        if o < start:
            continue
        k = struct.unpack_from('<i', b, o + 12)[0]
        if not 0 < k < 100000 or o + 16 + 4 * k + 16 > end:
            continue
        lens = struct.unpack_from('<%di' % k, b, o + 16)
        vo = o + 16 + 4 * k
        if struct.unpack_from('<iii', b, vo) != (12, 100, 2):
            continue
        n = struct.unpack_from('<i', b, vo + 12)[0]
        no = vo + 16 + 12 * n
        if n != sum(lens) or min(lens) < 3 or no + 16 + 12 * n > end \
                or struct.unpack_from('<iiii', b, no) != (12, 100, 2, n):
            continue
        v = np.frombuffer(b, '<f4', 3 * n, vo + 16).reshape(-1, 3).astype(float)
        nv = np.frombuffer(b, '<f4', 3 * n, no + 16).reshape(-1, 3).astype(float)
        idx = np.concatenate([np.arange(s, s + ln - 2) for s, ln in zip(np.cumsum((0,) + lens[:-1]), lens)])
        t = np.stack([v[idx], v[idx + 1], v[idx + 2]], axis=1)
        fn = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        keep = np.linalg.norm(fn, axis=1) > 1e-14
        flip = np.einsum('ij,ij->i', fn, nv[idx] + nv[idx + 1] + nv[idx + 2]) < 0
        t[flip] = t[flip][:, [0, 2, 1]]
        tris.append(t[keep])
    return np.concatenate(tris) if tris else np.zeros((0, 3, 3))


def decimate(t, max_tris):
    """vertex clustering：格子從 1 mm 開始放大，直到三角形數 <= max_tris。"""
    h = 0.001
    while len(t) > max_tris:
        v = t.reshape(-1, 3)
        _, inv = np.unique(np.floor(v / h).astype(np.int64), axis=0, return_inverse=True)
        inv = inv.ravel()
        cnt = np.bincount(inv)
        rep = np.stack([np.bincount(inv, v[:, k]) / cnt for k in range(3)], axis=1)
        f = inv.reshape(-1, 3)
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        _, u = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        out = rep[f[np.sort(u)]]
        if len(out) <= max_tris:
            return out
        h *= 1.25
    return t


def write_stl(path, t):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-30
    rec = np.zeros(len(t), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, t
    with open(path, 'wb') as f:
        f.write(b'orca sldasm_mesh.py'.ljust(80, b' ') + struct.pack('<I', len(t)) + rec.tobytes())


def rpy(r):
    p = np.arcsin(-np.clip(r[2, 0], -1, 1))
    if abs(np.cos(p)) < 1e-9:
        return np.array([0.0, p, np.arctan2(-r[0, 1], r[1, 1])])
    return np.array([np.arctan2(r[2, 1], r[2, 2]), p, np.arctan2(r[1, 0], r[0, 0])])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sldasm')
    ap.add_argument('out')
    ap.add_argument('--name', action='append', default=None, help='零件名=STL 檔名（不含 .stl），可重複')
    ap.add_argument('--max-tris', type=int, default=12000, help='每個零件的三角形上限')
    ap.add_argument('--fwd', default='+Z', choices=AXES, help='船頭方向（組合件座標）')
    ap.add_argument('--up', default='+Y', choices=AXES, help='上方（組合件座標）')
    a = ap.parse_args()
    names = dict(s.split('=', 1) for s in (a.name or DEFAULT_NAMES))

    sec = sections(open(a.sldasm, 'rb').read())
    tree = ET.fromstring(sec['swXmlContents/COMPINSTANCETREE'])
    models = {m.get('id'): m.get('swName') for m in tree.iter(SW + 'swModel')}
    comps = []
    for ref in tree.iter(SW + 'swReference'):
        if ref.get('swSuppressed') == 'YES':
            continue
        m = np.array([float(v) for v in ref.get('swTransform').split()]).reshape(4, 4)
        comps.append(dict(part=models[ref.get('swModelRef')],
                          inst=f"{ref.get('swComponentName')}-{ref.get('swReferenceNumber')}",
                          R=m[:3, :3].T, t=m[3, :3]))

    # 零件網格在 stream 裡的起點：Directory 中「實例名@組合件」之後的 stream 名稱前 8 bytes 是 id，stream 裡同一個 id 開頭
    d = sec['FaceTessellations/Directory']
    strs = utf16_strings(d)
    inst_part = {c['inst']: c['part'] for c in comps}
    starts = {}
    for (s0, e0, nm), (s1, _, stream) in zip(strs, strs[1:]):
        inst = nm.split('@')[0]
        if '@' in nm and stream and inst in inst_part and inst_part[inst] not in starts:
            data = sec.get('FaceTessellations/' + stream)
            if data is None:
                continue
            pos = data.find(d[s1 - 8:s1])
            if pos >= 0:
                starts[inst_part[inst]] = (stream, pos)
    meshes = {}
    for part, (stream, pos) in starts.items():
        data = sec['FaceTessellations/' + stream]
        end = min([p for s, p in starts.values() if s == stream and p > pos], default=len(data))
        meshes[part] = tessellation(data, pos, end)
    missing = {c['part'] for c in comps} - set(meshes)
    if missing:
        sys.exit(f'找不到這些零件的網格：{missing}')

    # 組合件 → FLU
    fi, fs = AXES[a.fwd]
    ui, us = AXES[a.up]
    ex, ez = np.eye(3)[fi] * fs, np.eye(3)[ui] * us
    A = np.stack([ex, np.cross(ez, ex), ez])
    world = np.concatenate([np.einsum('ij,nkj->nki', A @ c['R'], meshes[c['part']]) + A @ c['t'] for c in comps])
    v = world.reshape(-1, 3)
    origin = (v.min(0) + v.max(0)) / 2
    print(f'整台（FLU，原點 = 外框中心）：長 {np.ptp(v[:, 0]):.4f}、寬 {np.ptp(v[:, 1]):.4f}、高 {np.ptp(v[:, 2]):.4f} m')
    print(f'原點在組合件座標 {np.round(A.T @ origin, 5)}')

    os.makedirs(a.out, exist_ok=True)
    print('\n# STL（零件座標，m）')
    for part, t in meshes.items():
        fn = names.get(part, part)
        dt = decimate(t, a.max_tris)
        write_stl(os.path.join(a.out, fn + '.stl'), dt)
        print(f'{fn}.stl  ← {part}：{len(t)} → {len(dt)} 個三角形')

    print('\n# 實例在 FLU 的 pose：x y z roll pitch yaw')
    for c in comps:
        r, p = A @ c['R'], A @ c['t'] - origin
        print(f"{names.get(c['part'], c['part']):12s} {c['inst']:32s} " + ' '.join(f'{x:.4f}' for x in np.r_[p, rpy(r)]))

    print('\n# 推進器（零件名含 T200）：中心 = 零件外框中心，軸 = 零件 z 軸')
    for c in comps:
        if 'T200' in c['part']:
            t = meshes[c['part']].reshape(-1, 3)
            ctr = A @ (c['R'] @ ((t.min(0) + t.max(0)) / 2) + c['t']) - origin
            ax = A @ c['R'][:, 2]
            print(f"{c['inst']:32s} 中心 " + ' '.join(f'{x:.4f}' for x in ctr) + '  軸 ' + ' '.join(f'{x:.4f}' for x in ax))


if __name__ == '__main__':
    main()
