"""Sizing pass 2: candidate airframes + servo demand with gear ratio and efficiency."""
import numpy as np

RHO, G = 1.225, 9.81


def geom(span, body, cr, ct, mw_g):
    b = (span - body) / 2 / 1000
    cr, ct = cr / 1000, ct / 1000
    r = np.linspace(0, b, 500)
    c = cr + (ct - cr) * r / b
    S = np.trapezoid(c, r)
    mw = mw_g / 1000
    J = mw / S * np.trapezoid(r**2 * c, r)
    return dict(b=b, S=S, J=J, I1=np.trapezoid(r**2 * c, r), I3=np.trapezoid(r**3 * c, r))


def lift(g, phi_deg, f, Cl=0.8, up=0.65):
    w, phi = 2 * np.pi * f, np.radians(phi_deg)
    return 2 * (0.5 * RHO * Cl * (phi * w) ** 2 / 2 * g['I1']) * (1 + up) / 2


def servo(g, phi_deg, f, i=3.5, eff=0.85, Cl=0.8, spring=0.0):
    """spring: fraction of inertia torque cancelled by spring assist (0..1)"""
    w, phi = 2 * np.pi * f, np.radians(phi_deg)
    m_aero = 0.5 * RHO * Cl * (phi * w) ** 2 * g['I3']
    m_in = g['J'] * phi * w ** 2 * (1 - spring)
    m_tot = m_aero + m_in
    m_s = m_tot / i / eff
    travel = phi_deg / i
    rate = travel * w * np.pi / 180 * 180 / np.pi
    return dict(m_s=m_s / 0.0980665, travel=travel, rate=rate, wing_rate=phi_deg * w * np.pi / 180)


print('=== candidate airframes (mass 105 g if span 700, 120 g if span 800) ===')
cands = [
    ('60cm/120g', 600, 40, 170, 90, 6.0, 120, 50, 6),
    ('70cm/105g', 700, 40, 230, 120, 7.0, 105, 55, 6),
    ('80cm/120g', 800, 40, 260, 130, 8.0, 120, 55, 5),
]
for name, span, body, cr, ct, mw, mass, phi, f in cands:
    g = geom(span, body, cr, ct, mw)
    L = lift(g, phi, f)
    W = mass / 1000 * G
    s = servo(g, phi, f)
    print('%s: area=%5.0f cm2 wing_loading=%.2f kg/m2 | lift@%dHz/%.0fdeg=%.2f N vs weight %.2f N | servo %.2f kgcm, %.1f deg, %.0f deg/s'
          % (name, 2 * g['S'] * 1e4, mass / 1000 / (2 * g['S']), f, phi, L, W, s['m_s'], s['travel'], s['rate']))

print()
print('=== recommended 70cm/105g: servo demand table (i=3.5, eff=0.85) ===')
g = geom(700, 40, 230, 120, 7.0)
print('  %5s %6s %12s %12s %12s %12s' % ('f', 'Phi', 'servoM(kgcm)', 'travel(deg)', 'rate(deg/s)', 'wing rate'))
for f in (5, 6, 7, 8):
    for phi in (50, 55, 60):
        s = servo(g, phi, f)
        print('  %4dHz %5d %11.2f %11.1f %11.0f %10.0f' % (f, phi, s['m_s'], s['travel'], s['rate'], s['wing_rate']))

print()
print('=== spring assist effect at 70cm, f=6Hz, Phi=55, i=3.5 ===')
for sp in (0, 0.3, 0.5, 0.7):
    s = servo(g, 55, 6, spring=sp)
    print('  spring assist %3d%%: servo torque %.2f kgcm' % (sp * 100, s['m_s']))

print()
print('=== lift margin check (quasi-steady, upstroke loss 0.65) ===')
for f in (5, 6, 7):
    for phi in (50, 55, 60):
        print('  f=%dHz Phi=%2d: lift=%.3f N, weight=%.3f N, ratio=%.2f'
              % (f, phi, lift(g, phi, f), 0.105 * G, lift(g, phi, f) / (0.105 * G)))
