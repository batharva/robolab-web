#!/usr/bin/env python3
"""Independent cross-check of the JavaScript engine using NumPy (development only; not needed at runtime).
Usage:  python3 tests/reference_check.py      (requires node and numpy)"""
import json, subprocess, pathlib, sys
import numpy as np
root = pathlib.Path(__file__).resolve().parent
data = json.loads(subprocess.check_output(['node', str(root / 'dump.js')], text=True))
d2r = np.deg2rad

def dh(a, al, d, th):
    ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
    return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0, 0, 0, 1]])

def fk(joints, q):
    T = np.eye(4)
    for j, qi in zip(joints, q):
        th = d2r(j['theta']) + (qi if j['type'] == 'R' else 0); dd = j['d'] + (qi if j['type'] == 'P' else 0)
        T = T @ dh(j['a'], d2r(j['alpha']), dd, th)
    return T

def jac_fd(joints, q, h=1e-6):
    J = np.zeros((6, len(q)))
    for i in range(len(q)):
        qp, qm = np.array(q, float), np.array(q, float); qp[i] += h; qm[i] -= h
        Tp, Tm = fk(joints, qp), fk(joints, qm)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2*h)
        dR = Tp[:3, :3] @ Tm[:3, :3].T
        J[3:, i] = np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]]) / 2 / (2*h)
    return J

ok = True
for c in data['cases']:
    eT = np.abs(fk(c['joints'], c['q']) - np.array(c['T'])).max(); eJ = np.abs(jac_fd(c['joints'], c['q']) - np.array(c['J'])).max()
    good = eT < 1e-12 and eJ < 1e-6; ok &= good
    print(f"{c['id']:8s} FK max diff {eT:.2e}   Jacobian vs NumPy finite-diff {eJ:.2e}   {'PASS' if good else 'FAIL'}")

D = data['dyn']; m1, m2 = D['m']; l1, l2 = D['l']; c1, c2 = D['lc']; I1, I2 = D['I']; q1, q2 = D['q']; v1, v2 = D['qd']; a1, a2 = D['qdd']; g = 9.81
C2, S2 = np.cos(q2), np.sin(q2); h = -m2*l1*c2*S2
M = np.array([[m1*c1**2 + I1 + m2*(l1**2 + c2**2 + 2*l1*c2*C2) + I2, m2*(c2**2 + l1*c2*C2) + I2], [m2*(c2**2 + l1*c2*C2) + I2, m2*c2**2 + I2]])
Cq = np.array([h*v2*v1 + h*(v1+v2)*v2, -h*v1*v1])
G = np.array([(m1*c1 + m2*l1)*g*np.cos(q1) + m2*c2*g*np.cos(q1+q2), m2*c2*g*np.cos(q1+q2)])
tau = M @ np.array([a1, a2]) + Cq + G; eD = np.abs(tau - np.array(D['tau'])).max()
good = eD < 1e-7; ok &= good
print(f"planar2  inverse dynamics torque vs textbook (NumPy) max diff {eD:.2e}   {'PASS' if good else 'FAIL'}")
sys.exit(0 if ok else 1)
