"""Measure the wing drawing and compute aero / mechanism demands.

Workflow
1. flood-fill the background (sheet frame blocks it, so fill from inside too)
2. the four enclosed white regions are the wing panels
3. calibrate: forewing tip to forewing tip = 490 mm (as dimensioned)
4. per panel: area, span, chord, integral r^1 / r^2 / r^3 dA about the wing root
5. combine into half-wing numbers, compute lift / hinge moment / inertia / servo demand
6. annotate the drawing, write JSON + markdown report

Run:  python measure_wing.py
"""
import json
import os
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))          # tools/calc
ROOT = os.path.dirname(os.path.dirname(HERE))              # project root
SRC = os.path.join(ROOT, 'docs', 'img', 'wing_A2_1to1.jpg')
ANN = os.path.join(ROOT, 'docs', 'img', 'wing_measured.png')
OUT_JSON = os.path.join(HERE, 'wing_measure.json')
OUT_MD = os.path.join(HERE, 'wing_report.md')

RHO, G = 1.225, 9.81
SPAN_MM = 490.0


def components(interior, H, W, min_px=15000):
    """4-connected component labelling with a BFS; returns arrays of (y, x)."""
    ys, xs = interior.nonzero()
    lab = np.zeros((H, W), np.int32)
    comps, cur = [], 0
    for k in np.argsort(ys * W + xs):
        y0, x0 = ys[k], xs[k]
        if lab[y0, x0]:
            continue
        cur += 1
        q = deque([(y0, x0)])
        lab[y0, x0] = cur
        pix = []
        while q:
            y, x = q.popleft()
            pix.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and interior[ny, nx] and lab[ny, nx] == 0:
                    lab[ny, nx] = cur
                    q.append((ny, nx))
        if len(pix) >= min_px:
            comps.append(np.array(pix))
    return comps


img = Image.open(SRC).convert('L')
W, H = img.size
a = np.array(img)
walls = a < 170

b = Image.fromarray((walls * 255).astype(np.uint8)).convert('L')
for seed in [(2, 2), (W - 3, 2), (2, H - 3), (W - 3, H - 3), (W // 2, int(0.12 * H))]:
    ImageDraw.floodfill(b, seed, 128)
filled = np.array(b) == 128
interior = (~filled) & (~walls)

comps = components(interior, H, W)
print('panels found:', [c.shape[0] for c in comps])
comps = [c for c in comps if np.ptp(c[:, 1]) < 0.6 * W and np.ptp(c[:, 0]) < 0.6 * H]
print('wing panels:', [c.shape[0] for c in comps])

# ---------------------------------------------------------------- calibration
fore = [c for c in comps if c[:, 0].mean() < 0.55 * H]
if len(fore) != 2:
    raise SystemExit('expected 2 forewing panels, got %d' % len(fore))
x_tip_l = fore[0][:, 1].min() if fore[0][:, 1].mean() < W / 2 else fore[1][:, 1].min()
x_tip_r = fore[0][:, 1].max() if fore[0][:, 1].mean() > W / 2 else fore[1][:, 1].max()
span_px = x_tip_r - x_tip_l
px_per_mm = span_px / SPAN_MM
mm_per_px = 1.0 / px_per_mm
cx = W / 2.0
print('forewing tips x=%d..%d span=%d px -> %.4f px/mm (sheet width %.0f mm)'
      % (x_tip_l, x_tip_r, span_px, px_per_mm, W * mm_per_px))

# ---------------------------------------------------------------- panel stats
panels = []
for c in comps:
    ys_, xs_ = c[:, 0], c[:, 1]
    side = 'L' if xs_.mean() < cx else 'R'
    part = 'fore' if ys_.mean() < 0.55 * H else 'hind'
    root_x = xs_.max() if side == 'L' else xs_.min()
    r = np.abs(root_x - xs_) * mm_per_px
    dA = mm_per_px ** 2
    panels.append(dict(side=side, part=part, root_px=int(root_x),
                       area_mm2=float(c.shape[0] * dA),
                       span_mm=float(r.max()),
                       chord_mm=float((ys_.max() - ys_.min()) * mm_per_px),
                       m1_mm3=float((r * dA).sum()),
                       m2_mm4=float((r ** 2 * dA).sum()),
                       m3_mm5=float((r ** 3 * dA).sum())))
    print('%s %s: area %5.0f mm2 | root->tip %4.0f mm | chord %4.0f mm | rms r %4.0f mm'
          % (side, part, panels[-1]['area_mm2'], panels[-1]['span_mm'],
             panels[-1]['chord_mm'], (panels[-1]['m2_mm4'] / panels[-1]['area_mm2']) ** 0.5))

left = [p for p in panels if p['side'] == 'L']
A_half = sum(p['area_mm2'] for p in left) * 1e-6
I1 = sum(p['m2_mm4'] for p in left) * 1e-12
I3 = sum(p['m3_mm5'] for p in left) * 1e-15
b_half = max(p['span_mm'] for p in left) * 1e-3
print('\nhalf wing: A=%.4f m2  b=%.0f mm  I1=%.3e m4  I3=%.3e m5'
      % (A_half, b_half * 1e3, I1, I3))

# ---------------------------------------------------------------- chord profile / MAC
left_mask = np.zeros((H, W), bool)
for c in comps:
    if c[:, 1].mean() < cx:
        left_mask[c[:, 0], c[:, 1]] = True
cols = np.flatnonzero(left_mask.any(axis=0))
chord_px, yle_px = [], []
for x in cols:
    yy = np.flatnonzero(left_mask[:, x])
    if yy.size:
        chord_px.append(yy.max() - yy.min() + 1)
        yle_px.append(yy.min())
chord_px = np.array(chord_px, float)
yle_px = np.array(yle_px, float)
c_mm = chord_px * mm_per_px
MAC = float((c_mm ** 2).sum() / c_mm.sum())
ac_y_mm = float(((c_mm ** 2) * (yle_px * mm_per_px + 0.25 * c_mm)).sum() / (c_mm ** 2).sum())
y_front_mm = float(yle_px.min() * mm_per_px)
ac_from_le = ac_y_mm - y_front_mm
print('mean chord %.0f mm, MAC %.0f mm, quarter-MAC at %.0f mm behind the wing front point'
      % (c_mm.mean(), MAC, ac_from_le))


# ---------------------------------------------------------------- aero / mechanism
def analysis(mass_g, m_wing_g, phi_deg, f, Cl=0.8, up=0.65, i=4.0, eff=0.85, zeta=0.12):
    w, phi = 2 * np.pi * f, np.radians(phi_deg)
    J = (m_wing_g / 1000) / A_half * I1
    L = 2 * (0.5 * RHO * Cl * (phi * w) ** 2 / 2 * I1) * (1 + up) / 2
    M_aero = 0.5 * RHO * Cl * (phi * w) ** 2 * I3
    M_in = J * phi * w ** 2
    M_damp = 2 * zeta * M_in
    M_servo = (M_aero + M_in + M_damp) / i / eff
    travel = phi_deg / i
    rate = travel * w
    return dict(J=J, lift=L, weight=mass_g / 1000 * G, M_aero=M_aero, M_in=M_in,
                M_damp=M_damp, servo_kgcm=M_servo / 0.0980665,
                travel_deg=travel, rate_deg_s=rate)


def hover_f(mass_g, phi_deg, Cl=0.8):
    target = mass_g / 1000 * G
    lo, hi = 0.5, 15.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if analysis(mass_g, 5.5, phi_deg, mid, Cl=Cl)['lift'] < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


print('\n=== 75 g design point, i=4.0, Cl=0.8 (conservative) ===')
print('%-5s %-5s %8s %8s %10s %10s %10s %9s' %
      ('f', 'Phi', 'lift', 'weight', 'M_in', 'M_aero', 'servo', 'rate'))
for f in (5.0, 5.5, 6.0, 6.5, 7.0):
    for phi in (45, 50, 55, 60):
        r = analysis(75, 5.5, phi, f)
        print('%-5.1f %-5d %8.2f %8.2f %10.2f %10.2f %10.2f %9.0f' %
              (f, phi, r['lift'], r['weight'], r['M_in'] / 0.0980665,
               r['M_aero'] / 0.0980665, r['servo_kgcm'], r['rate_deg_s']))

print('\n=== same, Cl=1.15 (flat-plate normal force, optimistic) ===')
for f in (5.0, 5.5, 6.0):
    for phi in (50, 55, 60):
        r = analysis(75, 5.5, phi, f, Cl=1.15)
        print('%-5.1f %-5d lift=%5.2f N (need 0.74)  servo=%.2f kgcm  rate=%.0f deg/s'
              % (f, phi, r['lift'], r['servo_kgcm'], r['rate_deg_s']))

print('\n=== hover frequency needed (Cl=0.8) ===')
print('   mass   Phi=50   Phi=55   Phi=60')
for m in (65, 70, 75, 80, 85, 105):
    print('   %4d g %7.2f %8.2f %8.2f Hz' % (m, hover_f(m, 50), hover_f(m, 55), hover_f(m, 60)))

# ---------------------------------------------------------------- annotate
rgb = Image.open(SRC).convert('RGB')
d = ImageDraw.Draw(rgb)
colors = [(255, 0, 0), (0, 150, 0), (0, 0, 255), (200, 0, 200)]
for c, col in zip(comps, colors):
    for y, x in c[::40]:
        d.point((int(x), int(y)), fill=col)
d.line([(0, y_front_mm * px_per_mm), (W, y_front_mm * px_per_mm)], fill=(255, 140, 0), width=2)
d.line([(0, ac_y_mm * px_per_mm), (W, ac_y_mm * px_per_mm)], fill=(0, 160, 160), width=2)
rgb.save(ANN)
print('\nannotated ->', ANN)

# ---------------------------------------------------------------- write outputs
masses = (65, 70, 75, 80, 85, 105)
res = dict(source=SRC, span_mm=SPAN_MM, px_per_mm=px_per_mm, image=[W, H],
           panels=panels,
           half=dict(area_m2=A_half, total_area_m2=2 * A_half, b_mm=b_half * 1e3,
                     I1_m4=I1, I3_m5=I3,
                     rms_r_mm=(I1 / A_half) ** 0.5 * 1e3,
                     mean_chord_mm=A_half / b_half * 1e3,
                     MAC_mm=MAC, ac_from_le_mm=ac_from_le,
                     aspect_ratio=(2 * b_half) ** 2 / (2 * A_half)),
           hover_f=dict(mass=list(masses),
                        f50=[hover_f(m, 50) for m in masses],
                        f55=[hover_f(m, 55) for m in masses],
                        f60=[hover_f(m, 60) for m in masses]))
with open(OUT_JSON, 'w', encoding='utf-8') as fh:
    json.dump(res, fh, ensure_ascii=False, indent=2)

L = ['# 翼型实测与气动估算（据 490 mm 翼型图）', '',
     '> 由 `tools/calc/measure_wing.py` 自动生成；准定常气动，Cl=0.8、上拍升力系数 0.65、传动比 i=4.0、效率 0.85。', '',
     '## 1. 几何（像素测量，按前翅尖距 490 mm 标定）', '',
     '| 项目 | 数值 |', '|---|---|',
     '| 翼展（前翅尖距） | 490 mm |',
     '| 单侧翼面积 | %.4f m² |' % A_half,
     '| 双侧总面积 | %.4f m² |' % (2 * A_half),
     '| 单侧翼根→翼尖 b | %.0f mm |' % (b_half * 1e3),
     '| 平均弦长 A/b | %.0f mm |' % (A_half / b_half * 1e3),
     '| 平均气动弦 MAC | %.0f mm |' % MAC,
     '| 1/4 MAC 距翼最前点 | %.0f mm |' % ac_from_le,
     '| 展弦比 | %.2f |' % ((2 * b_half) ** 2 / (2 * A_half)),
     '| ∫r²dA | %.3e m⁴ |' % I1,
     '| ∫r³dA | %.3e m⁵ |' % I3, '',
     '## 2. 悬停所需拍翅频率（Cl=0.8）', '',
     '| 全重 | ±50° | ±55° | ±60° |', '|---|---|---|---|']
for m, a_, b_, c_ in zip(masses,
                         [hover_f(m, 50) for m in masses],
                         [hover_f(m, 55) for m in masses],
                         [hover_f(m, 60) for m in masses]):
    L.append('| %d g | %.2f Hz | %.2f Hz | %.2f Hz |' % (m, a_, b_, c_))
L += ['', '## 3. 75 g 设计点的舵机需求（i=4.0，效率 0.85）', '',
      '| 频率 | 幅度 | 舵机扭矩 | 舵机角速度 |', '|---|---|---|---|']
for f_ in (5.0, 5.5, 6.0, 6.5):
    for phi in (55, 60):
        r = analysis(75, 5.5, phi, f_)
        L.append('| %.1f Hz | ±%d° | %.2f kg·cm | %.0f °/s |'
                 % (f_, phi, r['servo_kgcm'], r['rate_deg_s']))
L.append('')
with open(OUT_MD, 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(L))
print('wrote', OUT_JSON, OUT_MD)
