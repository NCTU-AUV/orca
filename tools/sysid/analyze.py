#!/usr/bin/env python3
"""從實機 .BIN 粗估模擬的物理參數：先剔除異常，再逐軸擬合。

用法（主機，venv 要有 numpy scipy matplotlib pymavlink）：
    analyze.py <fc_logs 資料夾> [--bins 19 20 21] [--fc-time <fc_time.csv>] [--out <輸出資料夾>]
               [--sdf <model.sdf>] [--thrust sdf|linear] [--volt 16] [--asym]

機體一律從 model.sdf 讀（預設 sim/models/orca/model.sdf）：質量、質心、慣量、推進器的位置與推力方向、
ArduSub 輸出通道（RCOU C1..C8）對應哪顆推進器、阻力與附加質量、浮力體積、推力曲線。換機體只要改 model.sdf。
限制：base_link 在模型原點、沒有旋轉，其他 link 的 pose 相對於模型或 base_link；浮力體積 = base_link 所有
collision 的體積（box、cylinder、sphere）；慣量只取對角項，推進器 link 自己的慣量不轉座標（量很小）。

PWM→推力（--thrust）：
- sdf（預設）：跟模擬一樣。model.sdf 有 orca::ThrustCurve 就查它的推力表（--volt 可以換電壓，--asym 不取平均），
  沒有就用 ArduPilotPlugin 的直線換算。
- linear：舊模擬的直線公式（±45 N），只用來跟換推力曲線之前的結果比。
T200 推力表（sim/models/orca/t200_bollard.csv，Blue Robotics 公開的靜態推力，10～20 V）有死區（約 1472～1528 µs
為 0），小推力遠比直線小。預設把正反轉取平均（跟模擬一樣）；--asym 照表直接用（PWM > 1500 = 高效率方向）。
實測 --asym 跟預設結果一樣：MOT_n_DIRECTION 正負各半，同一個動作裡 PWM > 1500 和 < 1500 的推進器成對出現。
12～16 V 的曲線形狀幾乎一樣，電壓只讓推力整體乘一個倍數，擬合出的質量、阻力、浮力也跟著乘同一個倍數。
剔除和段落挑選一律用舊的直線公式算門檻，不同推力模型挑到的資料完全相同，結果可以直接比。
「模擬現況」用 model.sdf 現在的參數和推力曲線算。

剔除：未解鎖與解鎖前後、水面附近（< 0.2 m）、碰池底（EV 165～166 與 > 0.38 m）、推力很大
船卻幾乎不動（被抓住或頂牆）。迴歸再用 Huber 損失降低殘差大的點的權重。

擬合：
- 淨浮力：水下、定深穩定時，垂直推力的平均（穩態時推力剛好抵銷淨浮力，跟控制器無關）。
- yaw：輸出誤差（從推力開迴路模擬 yaw rate，對實測擬合），只用沒有平移推力的時段（平移時
  的水動力耦合會讓定向控制器出力，一自由度模型解釋不了）。第一個 .BIN 擬合、其他的驗證。
- surge、sway：3 秒窗的輸出誤差，預測加速度對實測加速度（不積分實測值，所以不會漂移），
  每窗的起始速度當未知數。
- roll、pitch、heave 不擬合：定深模式下這幾軸是控制器在出力，迴歸會擬合到控制律。要 MANUAL
  的資料。

座標全部換成模擬的機體 FLU（x 前、y 左、z 上）；.BIN 是 FRD。
"""
import argparse
import csv
import datetime
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, filtfilt
from scipy.sparse import lil_matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from binlog import (load, EV_BOTTOMED, EV_NOT_BOTTOMED)  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

G = 9.80665
RHO = 1000.0
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DEFAULT_SDF = os.path.join(REPO, 'sim', 'models', 'orca', 'model.sdf')

SURFACE_DEPTH = 0.2     # 比這淺算水面附近 [m]（深度以浮著歸零時為 0）
BOTTOM_DEPTH = 0.38     # 比這深算可能碰底 [m]
YAW_CHUNK = 8.0         # yaw 開迴路模擬每段最長幾秒，每段從實測值重新開始
TRANS_WIN = 3.0         # 平移的窗長 [s]
PRE = 1.0               # 推進器延遲的暖機：窗前多取幾秒的推力 [s]


# ================================================================ 推力曲線
def old_linear_curve(pwm):
    """舊模擬的 PWM→推力 [N]（±45 N 直線）。剔除、挑段的門檻一律用它，跟推力模型無關。"""
    return 90 * ((pwm - 1100) / 800 - 0.5)


def t200_curve(path, volt, asym=False):
    """推力表（第一列 pwm,10V,12V,…；# 開頭是註解）依電壓線性內插，回傳 PWM→推力 [N] 的函式。
    跟 sim/plugins/thrust_curve 的 orca::ThrustCurve 同一套算法。"""
    with open(path) as f:
        rows = list(csv.reader(line for line in f if not line.startswith('#')))
    vs = np.array([float(h.rstrip('V')) for h in rows[0][1:]])
    tab = np.array(rows[1:], float)
    if not vs[0] <= volt <= vs[-1]:
        raise SystemExit(f'電壓要在 {vs[0]:g}～{vs[-1]:g} V')
    k = min(np.searchsorted(vs, volt, 'right'), len(vs) - 1)
    a = (volt - vs[k - 1]) / (vs[k] - vs[k - 1])
    pw, f = tab[:, 0], (1 - a) * tab[:, k] + a * tab[:, k + 1]
    if not asym:   # 正反轉取平均：f(1500 + d) = -f(1500 - d)
        f = (f - np.interp(3000 - pw, pw, f)) / 2
    return lambda pwm: np.interp(pwm, pw, f)


# ================================================================ 從 model.sdf 讀機體
def _floats(el, default):
    return np.array([float(v) for v in el.text.split()]) if el is not None and el.text else np.array(default, float)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _pose(el):
    """el 底下的 <pose> → (位置, 旋轉矩陣, relative_to)。"""
    pe = el.find('pose')
    v = _floats(pe, [0] * 6)
    if pe is not None and pe.get('degrees') == 'true':
        v[3:] = np.radians(v[3:])
    return v[:3], _rpy(*v[3:]), (pe.get('relative_to') if pe is not None else None)


class Vehicle:
    """model.sdf 裡跟擬合有關的東西。位置都相對於整船的質心（FLU）。"""

    def __init__(self, path):
        self.path = path
        model = ET.parse(path).getroot().find('model')
        links = {}
        for ln in model.findall('link'):
            p, R, rel = _pose(ln)
            if rel not in (None, '__model__', 'base_link'):
                raise SystemExit(f'{path}：link {ln.get("name")} 的 pose 相對於 {rel}，不支援')
            links[ln.get('name')] = (p, R, ln)
        bp, bR, base = links['base_link']
        if np.any(bp != 0) or not np.allclose(bR, np.eye(3)):
            raise SystemExit(f'{path}：base_link 要在模型原點、沒有旋轉')

        # 質量、質心、慣量（平行軸定理，只取對角項）
        ms, cs, Is = [], [], []
        for p, R, ln in links.values():
            inr = ln.find('inertial')
            if inr is None:
                continue
            ip, _, _ = _pose(inr)
            it = inr.find('inertia')
            ms.append(float(inr.findtext('mass', '0')))
            cs.append(p + R @ ip)
            Is.append([float(it.findtext(k, '0')) for k in ('ixx', 'iyy', 'izz')] if it is not None else [0, 0, 0])
        ms, cs = np.array(ms), np.array(cs)
        self.mass = ms.sum()
        self.com = ms @ cs / self.mass
        r = cs - self.com
        self.inertia = np.sum(Is, axis=0) + ms @ np.column_stack(
            [r[:, 1] ** 2 + r[:, 2] ** 2, r[:, 0] ** 2 + r[:, 2] ** 2, r[:, 0] ** 2 + r[:, 1] ** 2])

        # 推進器：joint 的子 link 位置、joint 軸（子 link 座標）轉到機體
        joints = {}
        for j in model.findall('joint'):
            p, R, _ = links[j.findtext('child')]
            ax = _floats(j.find('axis/xyz'), [1, 0, 0])
            joints[j.get('name')] = (p - self.com, R @ ax / np.linalg.norm(ax))
        plugins = {pl.get('filename'): pl for pl in model.findall('plugin')}
        ap = plugins.get('ArduPilotPlugin')
        if ap is None:
            raise SystemExit(f'{path} 沒有 ArduPilotPlugin')
        # RCOU C{ch+1} → 推進器：位置、推力方向、multiplier 的正負號、直線換算 (servo_min, servo_max, offset, |multiplier|)
        self.channels = []
        for c in sorted(ap.findall('control'), key=lambda c: int(c.get('channel'))):
            mult = float(c.findtext('multiplier'))
            p, d = joints[c.findtext('jointName')]
            self.channels.append(dict(ch=int(c.get('channel')), joint=c.findtext('jointName'), pos=p, dir=d,
                                      sign=np.sign(mult),
                                      lin=(float(c.findtext('servo_min', '1000')), float(c.findtext('servo_max', '2000')),
                                           float(c.findtext('offset', '0')), abs(mult))))
        tc = plugins.get('OrcaThrustCurve')
        self.table = self.volt = None
        if tc is not None:
            uri = tc.findtext('table')
            if uri.startswith('model://'):   # sim/models/<模型>/<檔案>
                self.table = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(path))), uri[len('model://'):])
            else:
                self.table = os.path.join(os.path.dirname(os.path.abspath(path)), uri)
            self.volt = float(tc.findtext('voltage'))

        hy = plugins.get('gz-sim-hydrodynamics-system')

        def coef(tags):
            return [-float(hy.findtext(t, '0')) if hy is not None else 0.0 for t in tags]
        self.added = coef(['xDotU', 'yDotV', 'zDotW', 'kDotP', 'mDotQ', 'nDotR'])
        self.lin = coef(['xU', 'yV', 'zW', 'kP', 'mQ', 'nR'])
        self.quad = coef(['xUabsU', 'yVabsV', 'zWabsW', 'kPabsP', 'mQabsQ', 'nRabsR'])

        vol = 0.0
        for col in base.findall('collision'):
            geo = col.find('geometry')
            if geo.find('box') is not None:
                vol += np.prod(_floats(geo.find('box/size'), [0, 0, 0]))
            elif geo.find('cylinder') is not None:
                vol += np.pi * float(geo.findtext('cylinder/radius')) ** 2 * float(geo.findtext('cylinder/length'))
            elif geo.find('sphere') is not None:
                vol += 4 / 3 * np.pi * float(geo.findtext('sphere/radius')) ** 3
            else:
                raise SystemExit(f'{path}：base_link 的 collision 只支援 box、cylinder、sphere')
        self.volume = vol
        self.net_buoyancy = RHO * vol * G - self.mass * G

    def curve(self, volt=None, asym=False):
        """模擬的 PWM→推力；volt、asym 只在有推力表時有用。回傳 (函式, 名稱)。"""
        if self.table:
            v = volt or self.volt
            return t200_curve(self.table, v, asym), f't200_{v:g}V' + ('_asym' if asym else '')
        lins = {c['lin'] for c in self.channels}
        if len(lins) != 1:
            raise SystemExit(f'{self.path}：各通道的直線換算不一樣，不支援')
        lo, hi, off, m = lins.pop()
        return (lambda pwm: m * (np.clip((pwm - lo) / (hi - lo), 0, 1) + off)), 'sdf_linear'

    def yaw_torque_max(self, curve):
        """所有推進器同向出滿力時的最大 yaw 力矩 [N m]。"""
        fmax = (abs(float(curve(np.array(1100.)))) + abs(float(curve(np.array(1900.))))) / 2
        return fmax * sum(abs(np.cross(c['pos'], c['dir'])[2]) for c in self.channels)


# ================================================================ 小工具
def lp(x, fs, fc=2.0):
    """零相位低通（離線）。"""
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def zoh(ts, xs, t):
    i = np.clip(np.searchsorted(ts, t, 'right') - 1, 0, len(ts) - 1)
    return xs[i]


def runs(mask):
    """mask 裡連續 True 的區段 [(i0, i1)]，i1 不含。"""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    e = np.flatnonzero(np.diff(m))
    return list(zip(e[::2], e[1::2]))


def dilate(mask, n):
    out = mask.copy()
    for k in range(1, n + 1):
        out[k:] |= mask[:-k]
        out[:-k] |= mask[k:]
    return out


def box(x, n):
    return np.convolve(x, np.ones(n) / n, 'same')


def lag(x, dt, tau):
    """推進器的一階延遲（後向尤拉）。"""
    if tau <= 1e-3:
        return x.copy()
    out = np.empty_like(x)
    out[0] = x[0]
    al = dt / (tau + dt)
    for i in range(1, len(x)):
        out[i] = out[i - 1] + al * (x[i] - out[i - 1])
    return out


def wrench(rc, veh, curve):
    """RCOU → 機體 FLU 的合力、合力矩（對整船質心）。"""
    F = np.zeros((len(rc['t']), 3))
    M = np.zeros_like(F)
    for c in veh.channels:
        f = c['sign'] * curve(rc[f'C{c["ch"] + 1}'])
        F += np.outer(f, c['dir'])
        M += np.outer(f, np.cross(c['pos'], c['dir']))
    return F, M


class Clock:
    """飛控開機秒數 → 時鐘時間（有 fc_time.csv 才換算）。"""

    def __init__(self, path):
        self.off = None
        if path:
            with open(path) as f:
                row = next(csv.DictReader(f))
            self.off, self.drift = float(row['offset_s']), float(row['drift_ppm'])

    def __call__(self, tb):
        if self.off is None:
            return f'{tb:8.1f}s'
        return datetime.datetime.fromtimestamp(self.off + (1 + self.drift * 1e-6) * tb).strftime('%H:%M:%S')


# ================================================================ 讀資料、剔除
def build(path, veh, curve, sim_curve):
    d = load(path)
    im, rc, att, c = d['IMU'], d['RCOU'], d['ATT'], d['CTUN']
    t = im['t']
    fs = 1 / np.median(np.diff(t))
    s = dict(name=os.path.basename(path), t=t, fs=fs)
    # F、M：用來擬合的推力模型；F_sim、M_sim：模擬現在的推力（算「模擬現況」）；
    # F_lin、M_lin：舊的直線公式，只用來剔除、挑段落（門檻跟推力模型無關）
    for key, cv in (('', curve), ('_sim', sim_curve), ('_lin', old_linear_curve)):
        F, M = wrench(rc, veh, cv)
        s['F' + key], s['M' + key] = zoh(rc['t'], F, t), zoh(rc['t'], M, t)
    roll = np.radians(np.interp(t, att['t'], att['Roll']))
    pitch = np.radians(np.interp(t, att['t'], att['Pitch']))
    s['w'] = np.column_stack([im['GyrX'], -im['GyrY'], -im['GyrZ']])
    a_frd = np.column_stack([im['AccX'] - G * np.sin(pitch),
                             im['AccY'] + G * np.sin(roll) * np.cos(pitch),
                             im['AccZ'] + G * np.cos(roll) * np.cos(pitch)])
    s['a_raw'] = a_frd * [1, -1, -1]                  # 扣掉重力的加速度（FLU），還含偏差
    s['depth'] = np.interp(t, c['t'], -c['Alt'])
    s['depth_err'] = np.interp(t, c['t'], c['DAlt'] - c['Alt'])
    s['vz'] = np.interp(t, c['t'], c['CRt'] / 100)    # 向上為正 [m/s]
    arm = d.get('ARM')
    if arm is not None and (arm['ArmState'] == 1).any():
        t_on = arm['t'][arm['ArmState'] == 1][0]
        off = arm['t'][(arm['ArmState'] == 0) & (arm['t'] > t_on)]
        s['arm'] = (t_on, off[0] if len(off) else t[-1])
    else:   # 第一次解鎖的 ARM 有時沒寫進去：用 RCOU 離開中位的範圍
        act = np.flatnonzero(np.any(np.abs(np.column_stack([rc[f'C{i}'] for i in range(1, 9)]) - 1500) > 5, axis=1))
        s['arm'] = (rc['t'][act[0]], rc['t'][act[-1]]) if len(act) else (t[0], t[0])
    ev = d.get('EV', {'t': np.array([]), 'Id': np.array([])})
    s['events'] = list(zip(ev['t'], ev['Id'].astype(int)))
    s['why'] = reject(s)
    armed = ~s['why']['未解鎖／解鎖前後']
    # 加速度計偏差：安靜時段（推力小、角速度小）扣完重力後的中位數
    quiet = (np.abs(s['F_lin'][:, 0]) < 3) & (np.abs(s['F_lin'][:, 1]) < 3) & (np.abs(s['w']).max(1) < 0.1) & armed
    s['bias'] = np.median(lp(s['a_raw'], fs, 1)[quiet if quiet.sum() > 50 else armed], axis=0)
    s['ok_base'] = armed & ~s['why']['碰池底'] & ~s['why']['推很大卻不動']
    s['ok_sub'] = s['ok_base'] & ~s['why']['水面附近']
    return s


def reject(s):
    t, fs = s['t'], s['fs']
    k = int(round(fs))
    why = {'未解鎖／解鎖前後': ~((t > s['arm'][0] + 2) & (t < s['arm'][1] - 0.5)),
           '水面附近': s['depth'] < SURFACE_DEPTH}
    bottom = s['depth'] > BOTTOM_DEPTH
    t_on = None
    for te, eid in s['events']:
        if eid == EV_BOTTOMED:
            t_on = te
        elif eid == EV_NOT_BOTTOMED and t_on is not None:
            bottom |= (t > t_on - 1) & (t < te + 1)
            t_on = None
    if t_on is not None:
        bottom |= t > t_on - 1
    why['碰池底'] = bottom
    Fh = box(np.hypot(s['F_lin'][:, 0], s['F_lin'][:, 1]), k)
    ah = box(np.hypot(lp(s['a_raw'][:, 0], fs, 1) - np.median(s['a_raw'][:, 0]),
                      lp(s['a_raw'][:, 1], fs, 1) - np.median(s['a_raw'][:, 1])), k)
    wz = box(np.abs(lp(s['w'][:, 2], fs)), k)
    Nz = box(np.abs(s['M_lin'][:, 2]), k)
    held = ((Fh > 15) & (ah < 0.1) & (wz < 0.1)) | ((Nz > 10) & (wz < 0.05))
    why['推很大卻不動'] = dilate(held, k)
    return why


# ================================================================ 淨浮力
def fit_buoyancy(S, clk):
    rows = []
    for s in S:
        n = int(5 * s['fs'])
        ok = s['ok_sub'] & (np.abs(s['vz']) < 0.03) & (np.abs(s['depth_err']) < 0.03)
        for i0, i1 in runs(ok):
            for j in range(i0, i1 - n + 1, n):
                sl = slice(j, j + n)
                rows.append((s['name'], clk(s['t'][j]), -s['F'][sl, 2].mean(), s['depth'][sl].mean()))
    return rows


# ================================================================ yaw（輸出誤差）
def yaw_chunks(S, names, sub=False):
    out = []
    for s in S:
        if s['name'] not in names:
            continue
        k = int(round(s['fs']))
        pure = box(np.hypot(s['F_lin'][:, 0], s['F_lin'][:, 1]), 2 * k) < 10
        ok = (s['ok_sub'] if sub else s['ok_base']) & pure
        rf = lp(s['w'][:, 2], s['fs'])
        n, pre = int(YAW_CHUNK * s['fs']), int(PRE * s['fs'])
        for i0, i1 in runs(ok):
            for j in range(max(i0, pre), i1, n):
                e = min(j + n, i1)
                if e - j >= 2 * s['fs']:
                    out.append(dict(name=s['name'], t=s['t'][j:e], N_full=s['M'][j - pre:e, 2],
                                    N_sim_full=s['M_sim'][j - pre:e, 2], N_lin_full=s['M_lin'][j - pre:e, 2],
                                    pre=pre, fs=s['fs'],
                                    r0=rf[j], r=lp(s['w'][j:e, 2], s['fs']), t0=s['arm'][0]))
    return out


def yaw_sim(p, c, key='N_full'):
    """p = [I, d1, d2, 偏置, 推進器延遲]；回傳跟實測做同樣低通後的 yaw rate。
    key='N_sim_full' 用模擬現在的推力（算 model.sdf 現況的 R^2）。"""
    I, d1, d2, c0, tau = p
    t = c['t']
    N = lag(c[key], 1 / c['fs'], tau)[c['pre']:]
    r = np.empty(len(t))
    r[0] = c['r0']
    for i in range(1, len(t)):
        dt, rr = t[i] - t[i - 1], r[i - 1]
        r[i] = (I * rr + dt * (N[i - 1] - c0 - d2 * rr * abs(rr))) / (I + dt * d1)   # 線性阻尼半隱式
    return lp(r, c['fs'])


def yaw_fit(chunks, tau=None):
    """tau=None 時一起擬合推進器延遲，否則固定。"""
    def full(q):
        return np.append(q, tau) if tau is not None else q
    lo, hi = [0.3, 0, 0, -5, 0], [40, 200, 100, 5, 1.0]
    k = 4 if tau is not None else 5
    res = least_squares(lambda q: np.concatenate([yaw_sim(full(q), c) - c['r'] for c in chunks]),
                        [2.0, 5.0, 1.0, 0.0, 0.2][:k], bounds=(lo[:k], hi[:k]), loss='soft_l1', f_scale=0.3)
    return full(res.x)


def r2(pred, meas):
    return 1 - np.sum((pred - meas) ** 2) / np.sum((meas - meas.mean()) ** 2)


def yaw_r2(p, chunks, key='N_full'):
    return r2(np.concatenate([yaw_sim(p, c, key) for c in chunks]), np.concatenate([c['r'] for c in chunks]))


def yaw_groups(chunks, tau, n=3):
    """依每段的 yaw 力矩大小（直線推力的 |N| 中位數，跟推力模型無關）分 n 組、各自擬合（推進器延遲固定）。
    推力曲線對的話，各組的 I_eff、阻尼應該差不多。"""
    key = np.array([np.median(np.abs(c['N_lin_full'][c['pre']:])) for c in chunks])
    order = np.argsort(key)
    out = []
    for g in np.array_split(order, n):
        ch = [chunks[i] for i in g]
        p = yaw_fit(ch, tau=tau)
        nmed = np.median(np.concatenate([np.abs(c['N_full'][c['pre']:]) for c in ch]))
        out.append((key[g].min(), key[g].max(), nmed, p, yaw_r2(p, ch), secs(ch)))
    return out


def secs(chunks):
    return sum(len(c['t']) / c['fs'] for c in chunks)


# ================================================================ surge、sway（輸出誤差）
def trans_windows(S, ax, sub):
    out = []
    for s in S:
        fs = s['fs']
        n, pre = int(TRANS_WIN * fs), int(PRE * fs)
        ok = s['ok_sub'] if sub else s['ok_base']
        wz = np.abs(lp(s['w'][:, 2], fs))
        F, FL = s['F'][:, ax], s['F_lin'][:, ax]
        for i0 in range(pre, len(s['t']) - n, n // 2):
            sl = slice(i0, i0 + n)
            if ok[sl].all() and wz[sl].mean() < 0.3 and np.ptp(FL[sl]) >= 8:
                out.append(dict(name=s['name'], t=s['t'][sl], F_full=F[i0 - pre:i0 + n], pre=pre, fs=fs,
                                a=lp(s['a_raw'][sl, ax] - s['bias'][ax], fs), t0=s['arm'][0]))
    return out


def trans_sim(m, d1, d2, tau, v0, w):
    """回傳跟實測做同樣低通後的加速度。"""
    t = w['t']
    F = lag(w['F_full'], 1 / w['fs'], tau)[w['pre']:]
    a = np.empty(len(t))
    v = v0
    for i in range(len(t)):
        a[i] = (F[i] - d1 * v - d2 * v * abs(v)) / m
        if i + 1 < len(t):
            dt = t[i + 1] - t[i]
            v = (m * v + dt * (F[i] - d2 * v * abs(v))) / (m + dt * d1)
    return lp(a, w['fs'])


def trans_fit(wins, mass, m_fixed=None, tau_fixed=None):
    """參數：m、d1、d2、tau（推進器延遲）、每窗起始速度 v0。m、tau 可以固定。mass：剛體質量（m 的下限）。"""
    n = len(wins)
    free = [m_fixed is None, True, True, tau_fixed is None]
    k = sum(free)

    def unpack(p):
        vals, j = [], 0
        for f, fx in zip(free, (m_fixed, None, None, tau_fixed)):
            if f:
                vals.append(p[j]); j += 1
            else:
                vals.append(fx)
        return (*vals, p[k:])

    def res(p):
        m, d1, d2, tau, v0 = unpack(p)
        return np.concatenate([trans_sim(m, d1, d2, tau, v0[j], w) - w['a'] for j, w in enumerate(wins)])

    rows = sum(len(w['t']) for w in wins)
    sp = lil_matrix((rows, k + n), dtype=int)
    sp[:, :k] = 1
    r0 = 0
    for j, w in enumerate(wins):
        sp[r0:r0 + len(w['t']), k + j] = 1
        r0 += len(w['t'])
    p0 = [v for v, f in zip([mass + 10, 30, 10, 0.2], free) if f]
    lo = [v for v, f in zip([mass, 0, 0, 0], free) if f]
    hi = [v for v, f in zip([300, 500, 500, 1.0], free) if f]
    r = least_squares(res, np.concatenate([p0, np.zeros(n)]),
                      bounds=(np.concatenate([lo, -2 * np.ones(n)]), np.concatenate([hi, 2 * np.ones(n)])),
                      jac_sparsity=sp, loss='soft_l1', f_scale=0.3)
    a = np.concatenate([w['a'] for w in wins])
    return unpack(r.x), r2(res(r.x) + a, a)


# ================================================================ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('fc_logs')
    ap.add_argument('--bins', nargs='+', help='編號，第一個用來擬合 yaw、其他驗證（預設全部）')
    ap.add_argument('--fc-time', help='fc_time.csv：把飛控時間換成時鐘時間')
    ap.add_argument('--out', help='圖的輸出資料夾（預設 <fc_logs>/../sysid/<推力模型>）')
    ap.add_argument('--sdf', default=DEFAULT_SDF, help='機體的 model.sdf（預設 sim/models/orca/model.sdf）')
    ap.add_argument('--thrust', choices=['sdf', 'linear'], default='sdf',
                    help='PWM→推力：sdf 跟模擬一樣（預設）、linear 舊模擬的 ±45 N 直線')
    ap.add_argument('--volt', type=float, help='推力表的電壓 [V]（預設照 model.sdf 的 orca::ThrustCurve）')
    ap.add_argument('--asym', action='store_true', help='推力表不把正反轉取平均')
    args = ap.parse_args()
    if args.bins:
        names = [f'{int(b):08d}.BIN' for b in args.bins]
    else:
        names = sorted(f for f in os.listdir(args.fc_logs) if f.endswith('.BIN'))
    veh = Vehicle(args.sdf)
    sim_curve, sim_tag = veh.curve()
    if args.thrust == 'linear':
        curve, tag = old_linear_curve, 'linear'
    else:
        curve, tag = veh.curve(args.volt, args.asym)
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.fc_logs)), 'sysid', tag)
    os.makedirs(out, exist_ok=True)
    clk = Clock(args.fc_time)
    S = [build(os.path.join(args.fc_logs, n), veh, curve, sim_curve) for n in names]
    S = [s for s in S if s['arm'][1] - s['arm'][0] > 5]

    def pts(cv):
        return '／'.join(f'{float(cv(np.array(p))):.1f}' for p in (1550, 1600, 1700, 1900))
    print(f'機體：{args.sdf}')
    print(f'  質量 {veh.mass:.2f} kg，質心 {np.round(veh.com, 3)} m，剛體轉動慣量（含推進器）{np.round(veh.inertia, 3)} kg m^2，'
          f'浮力體積 {veh.volume:.4f} m^3，淨浮力 {veh.net_buoyancy:+.1f} N')
    print(f'  模擬的推力：{sim_tag}（單顆 PWM 1550／1600／1700／1900 → {pts(sim_curve)} N）')
    print(f'擬合用的推力：{tag}（→ {pts(curve)} N）')

    print('\n== 剔除（秒，可重疊）')
    for s in S:
        dt = 1 / s['fs']
        armed = ~s['why']['未解鎖／解鎖前後']
        parts = '、'.join(f'{k} {np.sum(v & armed) * dt:.0f}' for k, v in s['why'].items() if k != '未解鎖／解鎖前後')
        print(f'  {s["name"]}（{clk(s["arm"][0])}～{clk(s["arm"][1])}，解鎖 {armed.sum() * dt:.0f} 秒）：{parts}；'
              f'可用 {s["ok_base"].sum() * dt:.0f} 秒，其中水下 {s["ok_sub"].sum() * dt:.0f} 秒')
        for key in ('碰池底', '推很大卻不動'):
            spans = [f'{clk(s["t"][i0])}～{clk(s["t"][i1 - 1])}' for i0, i1 in runs(s['why'][key] & armed)]
            if spans:
                print(f'    {key}：' + '、'.join(spans))
    print('  加速度計偏差（FLU，扣重力後）：' + '；'.join(f'{s["name"][6:8]} {np.round(s["bias"], 2)}' for s in S))

    # ---- 淨浮力
    rows = fit_buoyancy(S, clk)
    print('\n== 淨浮力（水下、定深穩定，5 秒一段）')
    if rows:
        Bs = np.array([r[2] for r in rows])
        B = np.median(Bs)
        print(f'  {B:+.1f} N（{len(Bs)} 段，四分位 {np.percentile(Bs, 25):+.1f}～{np.percentile(Bs, 75):+.1f}，深度 '
              f'{min(r[3] for r in rows):.2f}～{max(r[3] for r in rows):.2f} m）；模擬 {veh.net_buoyancy:+.1f} N')
        print(f'  → 浮力體體積 {veh.volume:.4f} → {(B + veh.mass * G) / (RHO * G):.4f} m^3（質量不變的話）')
    else:
        B = np.nan
        print('  沒有可用的段')

    # ---- yaw
    print('\n== yaw（輸出誤差，只用沒有平移推力的時段，每段 ≤ 8 秒從實測值開始）')
    sdf_yaw = [veh.inertia[2] + veh.added[5], veh.lin[5], veh.quad[5], 0.0, 0.0]

    def show(tag, p, ch):
        print(f'  {tag}（{secs(ch):.0f} 秒）：I_eff {p[0]:.2f}  線性 {p[1]:.2f}  二次 {p[2]:.2f}  偏置 {p[3]:+.2f} N m'
              f'  推進器延遲 {p[4]:.2f} s  R^2 {yaw_r2(p, ch):.2f}')

    tr, te = yaw_chunks(S, [S[0]['name']]), yaw_chunks(S, [s['name'] for s in S[1:]])
    p1 = yaw_fit(tr)
    show(f'用 {S[0]["name"]} 擬合', p1, tr)
    if te:
        print(f'    沒參與擬合的 {secs(te):.0f} 秒：R^2 {yaw_r2(p1, te):.2f}'
              f'（模擬現況 {yaw_r2(sdf_yaw, te, "N_sim_full"):.2f}）')
    allc = yaw_chunks(S, [s['name'] for s in S])
    pa = yaw_fit(allc)
    show('全部擬合', pa, allc)
    print(f'    模擬現況 R^2 {yaw_r2(sdf_yaw, allc, "N_sim_full"):.2f}')
    p0 = yaw_fit(allc, tau=0.0)
    show('全部擬合、不考慮推進器延遲', p0, allc)
    sub = yaw_chunks(S, [s['name'] for s in S], sub=True)
    if secs(sub) > 10:
        show('只用水下', yaw_fit(sub), sub)
    print(f'  依力矩大小分 3 組（推進器延遲固定 {pa[4]:.2f} s；推力曲線對的話各組應該差不多）：')
    for lo_, hi_, nmed, p, rr, sec in yaw_groups(allc, pa[4]):
        print(f'    直線推力 |N| {lo_:4.1f}～{hi_:4.1f}（本模型中位數 {nmed:4.1f} N m，{sec:.0f} 秒）：'
              f'I_eff {p[0]:5.2f}  線性 {p[1]:6.2f}  二次 {p[2]:6.2f}  R^2 {rr:.2f}')
    for nm, p, cv in (('擬合', pa, curve), ('模擬現況', sdf_yaw, sim_curve)):
        n_max = veh.yaw_torque_max(cv)
        rmax = n_max / p[1] if p[2] < 1e-6 else (-p[1] + np.sqrt(p[1] ** 2 + 4 * p[2] * n_max)) / (2 * p[2])
        print(f'  滿推力 {n_max:.0f} N m 的穩態轉速：{nm} {rmax:.1f} rad/s（{np.degrees(rmax):.0f}°/s），'
              f'時間常數約 {p[0] / (p[1] + 2 * p[2] * rmax):.2f} s')
    fig, axs = plt.subplots(len(S), 1, figsize=(22, 4 * len(S)), squeeze=False)
    for a_, s in zip(axs[:, 0], S):
        a_.plot(s['t'] - s['arm'][0], lp(s['w'][:, 2], s['fs']), color='0.6', lw=.8, label='measured')
        for c in allc:
            if c['name'] == s['name']:
                a_.plot(c['t'] - c['t0'], yaw_sim(p1, c), 'C0', lw=1)
                a_.plot(c['t'] - c['t0'], yaw_sim(sdf_yaw, c, 'N_sim_full'), 'C3', lw=.7, alpha=.6)
        a_.plot([], [], 'C0', label=f'fit on {S[0]["name"]}')
        a_.plot([], [], 'C3', label='model.sdf now')
        a_.set_ylim(-4, 4); a_.grid(alpha=.3); a_.legend(loc='upper right'); a_.set_ylabel('yaw rate rad/s')
        a_.set_title(s['name']); a_.set_xlabel('s from arm')
    fig.tight_layout(); fig.savefig(os.path.join(out, 'yaw.png'), dpi=60); plt.close(fig)

    # ---- surge、sway
    for ax, nm in ((0, 'surge'), (1, 'sway')):
        for sub in (True, False):
            wins = trans_windows(S, ax, sub)
            label = '只用水下' if sub else '全部（含水面）'
            print(f'\n== {nm}（{label}）：{len(wins)} 個 {TRANS_WIN:.0f} 秒窗')
            if len(wins) < 5:
                continue
            (m, d1, d2, tau, v0), rr = trans_fit(wins, veh.mass)
            print(f'  m_eff {m:.1f} kg（附加 {m - veh.mass:+.1f}；模擬 {veh.added[ax]:g}）  線性 {d1:.1f}（模擬 {veh.lin[ax]:g}）'
                  f'  二次 {d2:.1f}（模擬 {veh.quad[ax]:g}）  推進器延遲 {tau:.2f} s  加速度 R^2 {rr:.2f}')
            print(f'  0.3 m/s 時阻力 {d1 * .3 + d2 * .09:.1f} N（模擬 {veh.lin[ax] * .3 + veh.quad[ax] * .09:.1f}）；'
                  f'窗起點速度 {v0.min():+.2f}～{v0.max():+.2f} m/s')
            for mf in (veh.mass + 1, veh.mass + 15, veh.mass + 40):
                (_, e1, e2, et, _), rf = trans_fit(wins, veh.mass, mf)
                print(f'    固定 m_eff {mf:5.1f}：線性 {e1:5.1f}  二次 {e2:5.1f}  延遲 {et:.2f}  R^2 {rf:.3f}')
            (m0, f1, f2, _, _), rf = trans_fit(wins, veh.mass, tau_fixed=0.0)
            print(f'    不考慮推進器延遲：m_eff {m0:.1f}  線性 {f1:.1f}  二次 {f2:.1f}  R^2 {rf:.3f}')
            if sub:
                k = min(len(wins), 12)
                fig, axs = plt.subplots(k, 1, figsize=(10, 2.2 * k), squeeze=False)
                for a_, j in zip(axs[:, 0], range(k)):
                    w = wins[j]
                    a_.plot(w['t'] - w['t0'], w['a'], color='0.5', label='measured')
                    a_.plot(w['t'] - w['t0'], trans_sim(m, d1, d2, tau, v0[j], w), 'C0', label='fit')
                    a_.plot(w['t'] - w['t0'], w['F_full'][w['pre']:] / m, 'C2', lw=.6, label='F / m_eff')
                    a_.set_title(w['name'], fontsize=8); a_.grid(alpha=.3)
                axs[0, 0].legend(fontsize=7)
                fig.tight_layout(); fig.savefig(os.path.join(out, f'{nm}.png'), dpi=60); plt.close(fig)
    print(f'\n圖：{out}/')


if __name__ == '__main__':
    main()
