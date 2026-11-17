#!/usr/bin/env python3
"""水下成像後處理：用深度圖把 Gazebo 乾淨明亮的算圖變成像實機在水下拍到的影像。

Harmonic 的 ogre2 沒有 fog，所以不在 Gazebo 裡做，而是在 sim/gz_camera_bridge.py 把影像轉成 ROS
之前處理。sim/models/orca_cam 的每顆彩色相機旁邊有一顆同位置、同視角的 depth camera，這裡用它的
深度算每個像素到相機的距離 d，套簡化的水下成像模型（Jaffe–McGlamery／Akkaynak 的那一類）：

    I = J * tint * exp(-a * d)  +  V * (1 - exp(-b * d))
        └── 直接光：越遠越暗、紅色先不見 ──┘   └── 後向散射：越遠越接近水體的藍 ──┘

再加上隨距離變強的模糊（前向散射）與噪點。只改像素，影像的大小、編碼、camera_info 都不動。

目標是 SAUVC 比賽池的樣子（docs/sim/reference/gate+flare-2022.jpg）：水是乾淨明亮的藍，
能見度可以看到對面池壁，遠處的東西是被藍化、變淡，不是被泥沙吃掉。

預設值的來源（2026-11-17 量的）：隊上的水下實拍（ml/rfdetr 驗證集隨機 200 張）各取影像下緣、中段、上緣
的中位數顏色，跟官方照片 gate+flare-2022.jpg 的近景地板、對面池壁比對：
    隊上實拍   下緣 (0.49, 0.73, 0.80)   中段 (0.37, 0.69, 0.77)   上緣 (0.25, 0.62, 0.72)
    官方照片   近景地板 (0.08, 0.72, 0.84)   對面池壁 (0.01, 0.40, 0.67)   遠處水色 (0.01, 0.56, 0.80)
    模型（白磁磚）  2 m (0.47, 0.75, 0.84)   15 m (0.31, 0.66, 0.79)   無限遠 (0.12, 0.55, 0.75)
近處的顏色對隊上實拍（模型是用它訓練的），遠處的水色往官方照片的藍靠，不要灰濛濛的。
衰減係數約 0.05～0.07 /m：10 m 外剩約 6 成對比，25 m 外的對面池壁還看得到（官方照片看得到）。
距離是從畫面位置估的，所以只是「大致對得上」；要更準得拿隊上 GoPro 影片（docs/sim/reference/README.md）量。

開關與強度（環境變數，deploy/sim/compose.yaml 傳進來；ROS 參數可以在執行中改，見 gz_camera_bridge.py）：
    UNDERWATER=1|0             0 = 完全關掉，影像原封不動（偵測驗證的 A/B 比較用）
    UNDERWATER_STRENGTH=1.0    整體強度：衰減、散射、模糊、噪點一起乘。0.5 = 更清澈，2 = 更混
細項（要重調外觀時用，一般不用動）：
    UNDERWATER_ATTEN=0.07,0.055,0.05     直接光的衰減係數 a（R,G,B）[1/m]
    UNDERWATER_SCATTER=0.07,0.055,0.05   後向散射的係數 b（R,G,B）[1/m]
    UNDERWATER_VEIL=0.12,0.55,0.75       遠處水體的顏色 V（R,G,B，0~1）
    UNDERWATER_TINT=0.60,0.84,0.92       陽光從水面照到物體那一段造成的整體偏青（紅光先被吃掉）
    UNDERWATER_BLUR=2.0                  最遠處的高斯模糊 sigma [px]（640 寬的影像）
    UNDERWATER_NOISE=2.5                 噪點的標準差（8-bit 的灰階值）

直接執行可以對一張存好的影像試參數：
    python3 underwater.py rgb.npy depth.npy out.png --hfov 0.9725
"""
import os

import cv2
import numpy as np

MAX_RANGE = 60.0        # depth camera 沒打到東西（inf）時當成這麼遠 [m]，同 orca_cam 前視相機的 far clip
LUT_STEP = 0.01         # 距離查表的解析度 [m]


def _vec(name, default):
    raw = os.environ.get(name)
    if not raw:
        return np.array(default, np.float32)
    vals = [float(v) for v in raw.split(',')]
    if len(vals) != 3:
        raise ValueError(f'{name} 要三個數（R,G,B），收到 {raw!r}')
    return np.array(vals, np.float32)


class Underwater:
    """一顆相機一個實例（查表、噪點、距離修正都跟影像大小、視角有關）。"""

    def __init__(self, hfov, enabled=None, strength=None, seed=None):
        self.hfov = hfov
        self.enabled = (os.environ.get('UNDERWATER', '1') != '0') if enabled is None else bool(enabled)
        self.strength = float(os.environ.get('UNDERWATER_STRENGTH', '1.0')) if strength is None else float(strength)
        self.atten = _vec('UNDERWATER_ATTEN', (0.07, 0.055, 0.05))
        self.scatter = _vec('UNDERWATER_SCATTER', (0.07, 0.055, 0.05))
        self.veil = _vec('UNDERWATER_VEIL', (0.12, 0.55, 0.75))
        self.tint = _vec('UNDERWATER_TINT', (0.60, 0.84, 0.92))
        self.blur = float(os.environ.get('UNDERWATER_BLUR', '2.0'))
        self.noise = float(os.environ.get('UNDERWATER_NOISE', '2.5'))
        self.rng = np.random.default_rng(seed)     # 噪點；給 seed 就每次都一樣（偵測驗證用）
        self._lut_key = None
        self._ray = {}          # (深度圖 w, h) → 每個像素「沿光軸的深度 → 到相機的距離」的倍率
        self._noise = {}        # (w, h) → 預先算好的幾張噪點

    # ---- 查表：距離 → 透射率、散射量、模糊權重
    def _luts(self):
        key = (self.strength, tuple(self.atten), tuple(self.scatter))
        if key != self._lut_key:
            d = np.arange(0, MAX_RANGE + LUT_STEP, LUT_STEP, dtype=np.float32)[:, None]
            s = max(self.strength, 0.0)
            self._t = np.exp(-d * self.atten[None] * s) * self.tint[None]
            self._b = (1 - np.exp(-d * self.scatter[None] * s)) * self.veil[None]
            self._w = (1 - np.exp(-d[:, 0] * 0.08 * s)).astype(np.float32)   # 模糊：10 m 處約 55%
            self._lut_key = key
        return self._t, self._b, self._w

    def _ray_scale(self, w, h):
        if (w, h) not in self._ray:
            f = (w / 2) / np.tan(self.hfov / 2)
            x = (np.arange(w, dtype=np.float32) + 0.5 - w / 2) / f
            y = (np.arange(h, dtype=np.float32) + 0.5 - h / 2) / f
            self._ray[(w, h)] = np.sqrt(1 + x[None] ** 2 + y[:, None] ** 2)
        return self._ray[(w, h)]

    def _noise_frame(self, w, h):
        if (w, h) not in self._noise:
            self._noise[(w, h)] = self.rng.normal(0, 1, (4, h, w, 3)).astype(np.float32)
        pool = self._noise[(w, h)]
        frame = pool[self.rng.integers(len(pool))]
        # 每一幀隨機平移、翻面，四張就不會看出重複
        frame = np.roll(frame, (int(self.rng.integers(h)), int(self.rng.integers(w))), axis=(0, 1))
        return frame if self.rng.integers(2) else -frame

    def apply(self, rgb, depth):
        """rgb：HxWx3 uint8。depth：depth camera 的影像（float32，公尺，沿光軸；可以比 rgb 小）。
        回傳 HxWx3 uint8。enabled = False 或 depth 是 None 時原封不動回傳 rgb。"""
        if not self.enabled or depth is None:
            return rgb
        h, w = rgb.shape[:2]
        dh, dw = depth.shape
        rng_m = np.nan_to_num(depth, nan=MAX_RANGE, posinf=MAX_RANGE, neginf=0.0) * self._ray_scale(dw, dh)
        if (dw, dh) != (w, h):
            rng_m = cv2.resize(rng_m, (w, h), interpolation=cv2.INTER_LINEAR)
        idx = np.clip(rng_m * (1 / LUT_STEP), 0, MAX_RANGE / LUT_STEP).astype(np.int32)
        t, b, wgt = self._luts()

        img = rgb.astype(np.float32) * (1 / 255.0)
        out = img * t[idx] + b[idx]

        s = max(self.strength, 0.0)
        if self.blur * s > 0.05:
            far = cv2.GaussianBlur(out, (0, 0), self.blur * s)
            k = wgt[idx][..., None]
            out = out + (far - out) * k
            out = cv2.GaussianBlur(out, (0, 0), 0.6 * min(s, 1.5))     # 鏡頭、防水殼本身的一點點糊
        if self.noise * s > 0:
            out = out + self._noise_frame(w, h) * (self.noise * s / 255.0)
        return np.clip(out * 255.0 + 0.5, 0, 255).astype(np.uint8)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='對存好的一張影像套水下效果（.npy：rgb HxWx3 uint8、depth float32）')
    ap.add_argument('rgb')
    ap.add_argument('depth')
    ap.add_argument('out')
    ap.add_argument('--hfov', type=float, default=0.9725)
    ap.add_argument('--strength', type=float, default=None)
    a = ap.parse_args()
    uw = Underwater(a.hfov, enabled=True, strength=a.strength)
    res = uw.apply(np.load(a.rgb), np.load(a.depth))
    cv2.imwrite(a.out, cv2.cvtColor(res, cv2.COLOR_RGB2BGR))
