"""Bionic butterfly - first-order estimates for aero / mechanism / servo sizing.

All printed text is ASCII so it can run on a GBK Windows console; the report is
written to result.md as UTF-8.
"""
import numpy as np

RHO = 1.225
G = 9.81
OUT = []


def p(s=''):
    OUT.append(s)
    print(s)


def wing(cr, ct, b, n=400):
    r = np.linspace(0, b, n)
    c = cr + (ct - cr) * r / b
    return r, c


def geometry(span_mm, body_mm, cr_mm, ct_mm, m_wing_g):
    b = (span_mm - body_mm) / 2 / 1000.0
    cr, ct = cr_mm / 1000.0, ct_mm / 1000.0
    r, c = wing(cr, ct, b)
    S = np.trapezoid(c, r)
    mw = m_wing_g / 1000.0
    J = mw / S * np.trapezoid(r ** 2 * c, r)
    I1 = np.trapezoid(r ** 2 * c, r)
    I3 = np.trapezoid(r ** 3 * c, r)
    return dict(b=b, cr=cr, ct=ct, S=S, J=J, I1=I1, I3=I3, mw=mw)


def lift_two(g, phi_deg, f, Cl=0.8, upstroke_loss=0.65):
    w = 2 * np.pi * f
    phi = np.radians(phi_deg)
    l1 = 0.5 * RHO * Cl * (phi * w) ** 2 / 2 * g['I1']
    return 2 * l1 * (1 + upstroke_loss) / 2


def moments(g, phi_deg, f, Cl=0.8, zeta=0.12, i_ratio=3.0, Q=4.0):
    w = 2 * np.pi * f
    phi = np.radians(phi_deg)
    m_aero = 0.5 * RHO * Cl * (phi * w) ** 2 * g['I3']
    m_in = g['J'] * phi * w ** 2
    m_damp = 2 * zeta * g['J'] * phi * w ** 2
    k_spring = g['J'] * w ** 2
    m_servo = (m_aero + m_damp) / i_ratio
    th_s = np.radians(phi_deg) / (i_ratio * Q)
    return dict(m_aero=m_aero, m_in=m_in, m_damp=m_damp, k_spring=k_spring,
                m_servo=m_servo, th_s_deg=np.degrees(th_s),
                rate_s_deg=th_s * w * 180 / np.pi)


def kgcm(nm):
    return nm / 0.0980665


p('=' * 92)
p('A. Baseline config (user default: span 600 mm, all-up mass 120 g)')
A = geometry(span_mm=600, body_mm=40, cr_mm=170, ct_mm=90, m_wing_g=6.0)
p('   semi-span b=%.0f mm, wing area=%.1f cm2 each, total=%.1f cm2'
  % (A['b'] * 1000, A['S'] * 1e4, 2 * A['S'] * 1e4))
p('   wing loading @120 g = %.2f kg/m2' % (0.120 / (2 * A['S'])))
p('')
p('   mean lift (N, both wings), need >= %.2f N' % (0.120 * G))
p('   %6s %9s %9s %9s' % ('f', 'Phi=40', 'Phi=50', 'Phi=60'))
for f in (3, 4, 5, 6, 7, 8, 9, 10):
    p('   %4dHz %9.3f %9.3f %9.3f' % (f, lift_two(A, 40, f), lift_two(A, 50, f), lift_two(A, 60, f)))

p('')
p('B. Recommended config (span 700 mm, all-up mass 100 g)')
B = geometry(span_mm=700, body_mm=40, cr_mm=200, ct_mm=110, m_wing_g=7.0)
p('   semi-span b=%.0f mm, wing area=%.1f cm2 each, total=%.1f cm2'
  % (B['b'] * 1000, B['S'] * 1e4, 2 * B['S'] * 1e4))
p('   wing loading = %.2f kg/m2, weight = %.2f N' % (0.100 / (2 * B['S']), 0.100 * G))
p('   %6s %9s %9s %9s' % ('f', 'Phi=40', 'Phi=50', 'Phi=60'))
for f in (3, 4, 5, 6, 7, 8, 9, 10):
    p('   %4dHz %9.3f %9.3f %9.3f' % (f, lift_two(B, 40, f), lift_two(B, 50, f), lift_two(B, 60, f)))

p('')
p('=' * 92)
p('C. Hinge moment / servo demand (config B, Phi=50 deg, i=3, Q=4, zeta=0.12)')
p('   %5s %11s %11s %9s %13s %10s %8s %9s' %
  ('f', 'M_aero', 'M_inertia', 'M_damp', 'spring k', 'servo M', 'srv travel', 'srv rate'))
for f in (4, 5, 6, 7, 8, 9):
    m = moments(B, 50, f)
    p('   %3dHz %7.2fkgcm %7.2fkgcm %5.2fkgcm %8.2fNm/rad %6.2fkgcm %6.1fdeg %6.0fdeg/s'
      % (f, kgcm(m['m_aero']), kgcm(m['m_in']), kgcm(m['m_damp']),
         m['k_spring'], kgcm(m['m_servo']), m['th_s_deg'], m['rate_s_deg']))

p('')
p('D. Without spring (servo must also carry inertia) vs with spring, config B, Phi=50')
for f in (4, 5, 6, 7):
    m = moments(B, 50, f)
    total = (m['m_in'] + m['m_aero']) / 3.0
    p('   %dHz: no-spring servo %5.2f kgcm  |  with spring %5.2f kgcm'
      % (f, kgcm(total), kgcm(m['m_servo'])))

p('')
p('E. Equivalent linear spring rate (tension spring at r=10 mm from wing root)')
for f in (6, 7, 8):
    k = B['J'] * (2 * np.pi * f) ** 2
    p('   f=%dHz: torsion %.3f N.m/rad  ->  linear %.0f N/m at 10 mm' % (f, k, k / 0.010 ** 2))

p('')
p('F. Servo speed reference')
p('   0.06 s/60deg = %.0f deg/s ; 0.09 s/60deg = %.0f deg/s ; 0.12 s/60deg = %.0f deg/s'
  % (1000, 666, 500))
p('   servo peak rate = wing peak rate / (i * Q)')

with open('result.md', 'w', encoding='utf-8') as fh:
    fh.write('# Flapping / servo sizing report (generated)\n\n```\n')
    fh.write('\n'.join(OUT))
    fh.write('\n```\n')
