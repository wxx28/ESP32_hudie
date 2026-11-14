# Flapping / servo sizing report (generated)

```
============================================================================================
A. Baseline config (user default: span 600 mm, all-up mass 120 g)
   semi-span b=280 mm, wing area=364.0 cm2 each, total=728.0 cm2
   wing loading @120 g = 1.65 kg/m2

   mean lift (N, both wings), need >= 1.18 N
        f    Phi=40    Phi=50    Phi=60
      3Hz     0.056     0.088     0.127
      4Hz     0.100     0.157     0.225
      5Hz     0.157     0.245     0.352
      6Hz     0.225     0.352     0.507
      7Hz     0.307     0.479     0.690
      8Hz     0.401     0.626     0.902
      9Hz     0.507     0.792     1.141
     10Hz     0.626     0.978     1.409

B. Recommended config (span 700 mm, all-up mass 100 g)
   semi-span b=330 mm, wing area=511.5 cm2 each, total=1023.0 cm2
   wing loading = 0.98 kg/m2, weight = 0.98 N
        f    Phi=40    Phi=50    Phi=60
      3Hz     0.111     0.174     0.250
      4Hz     0.198     0.309     0.444
      5Hz     0.309     0.482     0.694
      6Hz     0.444     0.694     1.000
      7Hz     0.605     0.945     1.361
      8Hz     0.790     1.235     1.778
      9Hz     1.000     1.563     2.250
     10Hz     1.235     1.929     2.778

============================================================================================
C. Hinge moment / servo demand (config B, Phi=50 deg, i=3, Q=4, zeta=0.12)
       f      M_aero   M_inertia    M_damp      spring k    servo M srv travel  srv rate
     4Hz    0.91kgcm    1.22kgcm  0.29kgcm     0.14Nm/rad   0.40kgcm    4.2deg    105deg/s
     5Hz    1.43kgcm    1.91kgcm  0.46kgcm     0.21Nm/rad   0.63kgcm    4.2deg    131deg/s
     6Hz    2.05kgcm    2.75kgcm  0.66kgcm     0.31Nm/rad   0.90kgcm    4.2deg    157deg/s
     7Hz    2.79kgcm    3.74kgcm  0.90kgcm     0.42Nm/rad   1.23kgcm    4.2deg    183deg/s
     8Hz    3.65kgcm    4.88kgcm  1.17kgcm     0.55Nm/rad   1.61kgcm    4.2deg    209deg/s
     9Hz    4.62kgcm    6.18kgcm  1.48kgcm     0.69Nm/rad   2.03kgcm    4.2deg    236deg/s

D. Without spring (servo must also carry inertia) vs with spring, config B, Phi=50
   4Hz: no-spring servo  0.71 kgcm  |  with spring  0.40 kgcm
   5Hz: no-spring servo  1.11 kgcm  |  with spring  0.63 kgcm
   6Hz: no-spring servo  1.60 kgcm  |  with spring  0.90 kgcm
   7Hz: no-spring servo  2.18 kgcm  |  with spring  1.23 kgcm

E. Equivalent linear spring rate (tension spring at r=10 mm from wing root)
   f=6Hz: torsion 0.309 N.m/rad  ->  linear 3087 N/m at 10 mm
   f=7Hz: torsion 0.420 N.m/rad  ->  linear 4202 N/m at 10 mm
   f=8Hz: torsion 0.549 N.m/rad  ->  linear 5488 N/m at 10 mm

F. Servo speed reference
   0.06 s/60deg = 1000 deg/s ; 0.09 s/60deg = 666 deg/s ; 0.12 s/60deg = 500 deg/s
   servo peak rate = wing peak rate / (i * Q)
```
