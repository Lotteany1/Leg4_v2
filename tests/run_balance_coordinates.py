"""Check generated plants against independent wheel-to-hip kinematics.

Run after full LQR/MPC generation. This uses the original wheel-coordinate
Newton equations, then a state similarity transform; it does not reuse either
generator's acceleration assembly. Q/R are not involved in this contract.
"""
from pathlib import Path
import numpy as np
from scipy.io import loadmat
from scipy.linalg import expm

root = Path(__file__).resolve().parents[2]
lqr = loadmat(root / 'Matlab/lqr/build/wbr/geometry_cache.mat', simplify_cells=True)
mpc = loadmat(root / 'Matlab/balance_mpc/build/tinympc_cache.mat', simplify_cells=True)
p = lqr['parameters']


def wheel_plant(left, right):
    rw, tr, lc = p['R_wheel'], p['half_track'], p['l_c']
    mw, ml, mb = p['m_wheel'], p['m_leg'], p['m_body']
    iw, ib, iz, g = p['I_wheel'], p['I_body'], p['I_yaw'], p['g']
    l, lw, lb, il = left
    r, wr, rb, ir = right
    mass = np.zeros((5, 5))
    gravity = np.zeros((5, 3))
    inputs = np.zeros((5, 4))
    mass[0, 0] = iw*l/rw + mw*rw*l + ml*rw*lb
    mass[0, 2] = ml*lw*lb-il
    gravity[0, 0] = -(ml*lw+mb*l/2)*g
    inputs[0] = [1+l/rw, 0, -1, 0]
    mass[1, 1] = iw*r/rw + mw*rw*r + ml*rw*rb
    mass[1, 3] = ml*wr*rb-ir
    gravity[1, 1] = -(ml*wr+mb*r/2)*g
    inputs[1] = [0, 1+r/rw, 0, -1]
    mass[2, :2] = -(mw*rw**2+iw+ml*rw**2+mb*rw**2/2)
    mass[2, 2:4] = [-rw*(ml*lw+mb*l/2), -rw*(ml*wr+mb*r/2)]
    inputs[2, :2] = -1
    mass[3, :2] = mw*rw*lc+iw*lc/rw+ml*rw*lc
    mass[3, 2:] = [ml*lw*lc, ml*wr*lc, -ib]
    gravity[3, 2] = -mb*g*lc
    inputs[3] = [lc/rw, lc/rw, 1, 1]
    mass[4, :4] = [iz*rw/(2*tr)+iw*tr/rw, -iz*rw/(2*tr)-iw*tr/rw,
                    iz*l/(2*tr), -iz*r/(2*tr)]
    inputs[4, :2] = [tr/rw, -tr/rw]
    transform = np.array([[rw/2, rw/2, 0, 0, 0],
                          [-rw/(2*tr), rw/(2*tr), -l/(2*tr), r/(2*tr), 0],
                          [0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
    a, b = np.zeros((10, 10)), np.zeros((10, 4))
    a[0::2, 1::2] = np.eye(5)
    a[1::2, 4::2] = transform @ np.linalg.solve(mass, gravity)
    b[1::2] = transform @ np.linalg.solve(mass, inputs)
    return a, b


worst = 0.0
n = len(lqr['Leg_data'])
for il, left in enumerate(lqr['Leg_data']):
    for ir, right in enumerate(lqr['Leg_data']):
        aw, bw = wheel_plant(left, right)
        # x_hip = x_wheel + mean(L*theta), v_hip = v_wheel + mean(L*theta_dot).
        s = np.eye(10)
        s[0, [4, 6]] = [left[0]/2, right[0]/2]
        s[1, [5, 7]] = [left[0]/2, right[0]/2]
        a = s @ aw @ np.linalg.inv(s)
        b = s @ bw
        index = il*n+ir
        np.testing.assert_allclose(lqr['A_all'][:, :, index], a, atol=1e-9)
        np.testing.assert_allclose(lqr['B_all'][:, :, index], b, atol=1e-9)
        for ts, actual_a, actual_b in [
            (lqr['Ts'], lqr['Ad_all'][:, :, index], lqr['Bd_all'][:, :, index]),
            (mpc['Ts'], mpc['Ad_num'][:, :, il, ir], mpc['Bd_num'][:, :, il, ir]),
        ]:
            e = expm(np.block([[a, b], [np.zeros((4, 14))]])*ts)
            np.testing.assert_allclose(actual_a, e[:10, :10], atol=1e-10)
            np.testing.assert_allclose(actual_b, e[:10, 10:], atol=1e-10)
            worst = max(worst, np.max(np.abs(actual_a-e[:10, :10])))

# Differentiate the exact body.c kinematics, including extension, independently.
t = 0.13
length, rate = np.array([0.23, 0.31]), np.array([0.04, -0.03])
angle, omega = np.array([0.08, -0.12]), np.array([0.2, -0.1])
def hip_position(time):
    return 0.7*time + np.mean((length+rate*time)*np.sin(angle+omega*time))
step = 1e-5
numeric = (hip_position(t+step)-hip_position(t-step))/(2*step)
analytic = 0.7 + np.mean(rate*np.sin(angle+omega*t)
                         + (length+rate*t)*omega*np.cos(angle+omega*t))
np.testing.assert_allclose(numeric, analytic, atol=1e-9)
print(f'PASS: {n*n} LQR/MPC plants in hip coordinates, worst Ad error {worst:.3e}; '
      'exact variable-length velocity derivative PASS')
