"""Minimal wing size for a target all-up mass, servo-only, with lift margin.

  lift(s,f)          = 0.99 * s^4 * (f/7)^2     N     (s=1 => 490 mm span, +-60 deg)
  servo torque(s,f)  = 1.08 * s^5 * (f/6.5)^2   kg.cm (i=4, eff 0.85)
  wing mass          = 2 * 80 * 0.0682 * s^2    g
  servo speed limit  : f <= 0.85 * rate*i/(Phi*2*pi)

Outputs the minimal span for lift = 1.30 x weight for several all-up masses and
servo classes, plus the capability of the measured 490 mm wing. Also writes
wing_sizing_report.md.
"""
import numpy as np

RHO, G = 1.225, 9.81
K = 1.30
PHI = 60.0
I_RATIO = 4.0
L7 = 0.99
M65 = 1.08
WING_KG_M2 = 0.080
A_HALF = 0.0682
F_CAP = 9.0

SERVOS = [
    dict(name='8.5 g (KST X08)', mass=8.5, torque=2.8, sec60=0.075),
    dict(name='12 g HV', mass=12.0, torque=3.2, sec60=0.080),
    dict(name='20 g (GDW DS290MG)', mass=20.0, torque=5.6, sec60=0.052),
    dict(name='30 g HV', mass=30.0, torque=8.0, sec60=0.060),
]
TARGETS = [90, 120, 150, 200, 250]
OUT = []


def p(s=''):
    print(s)
    OUT.append(s)


def lift(s, f):
    return L7 * s ** 4 * (f / 7.0) ** 2


def servo_tq(s, f):
    return M65 * s ** 5 * (f / 6.5) ** 2


def wing_mass(s):
    return 2 * WING_KG_M2 * A_HALF * s ** 2 * 1000


def power_w(s, f):
    phi = np.radians(PHI)
    w = 2 * np.pi * f
    m_aero = 0.5 * RHO * 0.8 * (phi * w) ** 2 * 1.783e-4 * s ** 5
    return 0.5 * m_aero * (phi * w) / 0.5


def s_for(M, f):
    need = K * M / 1000 * G
    return (need / (L7 * (f / 7.0) ** 2)) ** 0.25


p('## 1. 最小翼展（升力 = 1.30 × 重量，翼角 ±60°，i=4）')
p()
p('| 全重 | 舵机 | 最小翼展 | 频率 | 舵机扭矩 | 扭矩占用 | 翼重 | 电功率 |')
p('|---|---|---|---|---|---|---|---|')
for M in TARGETS:
    for sv in SERVOS:
        f_speed = (60.0 / sv['sec60']) * I_RATIO / (PHI * 2 * np.pi)
        f_hi = min(F_CAP, 0.85 * f_speed)
        best = None
        for f in np.arange(4.0, f_hi + 0.001, 0.05):
            s = s_for(M, f)
            tq = servo_tq(s, f)
            if tq <= 0.75 * sv['torque']:
                if best is None or s < best[0]:
                    best = (s, f, tq)
        if best is None:
            p('| %d g | %s | — 不可行 | — | — | — | — | — |' % (M, sv['name']))
            continue
        s, f, tq = best
        p('| %d g | %s | **%.0f mm** | %.1f Hz | %.2f kg·cm | %.0f%% | %.1f g | %.1f W |'
          % (M, sv['name'], 490 * s, f, tq, 100 * tq / sv['torque'], wing_mass(s), power_w(s, f)))
p()

p('## 2. 实测 490 mm 翼的能力（固定频率下的推重比 T/W）')
p()
p('| 全重 | 悬停频率 | T/W@7Hz | T/W@8Hz | T/W@9Hz |')
p('|---|---|---|---|---|')
for M in (75, 90, 120, 150, 200, 250):
    f_h = 7.0 * (K * M / 1000 * G / L7) ** 0.25
    p('| %d g | %.2f Hz | %.2f | %.2f | %.2f |'
      % (M, f_h, lift(1.0, 7) / (M / 1000 * G), lift(1.0, 8) / (M / 1000 * G),
         lift(1.0, 9) / (M / 1000 * G)))

with open('wing_sizing_report.md', 'w', encoding='utf-8') as fh:
    fh.write('# 最小翼展规划（舵机-only 起飞 + 升力余量）\n\n')
    fh.write('> 由 `wing_sizing_growth.py` 自动生成。一阶模型：升力∝s⁴f²，舵机力矩∝s⁵f²，'
             '翼重∝s²，i=4、效率 0.85、翼角 ±60°、频率上限 9 Hz、速度留 15% 余量。\n\n')
    fh.write('\n'.join(OUT))
    fh.write('\n')
print('\nwrote wing_sizing_report.md')
