"""Compare the C RLS tool with independent batch weighted least squares (NumPy)."""
import argparse
import ctypes as ct
import os
from pathlib import Path
import subprocess

import numpy as np


class Rls(ct.Structure):
    _fields_ = [('num', ct.c_uint8), ('lam', ct.c_float), ('p0', ct.c_float),
                ('theta', ct.c_double * 4), ('p', (ct.c_double * 4) * 4),
                ('y_hat', ct.c_double), ('error', ct.c_double)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True, help='Windows host GCC executable')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root / 'build/rls_test'
    output.mkdir(parents=True, exist_ok=True)
    compiler = Path(args.cc).resolve()
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    dll = output / 'rls_test.dll'
    subprocess.run([str(compiler), '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-ICode/Tool', 'Code/Tool/rls.c', '-o', str(dll)],
                   cwd=root, env=env, check=True)
    lib = ct.CDLL(str(dll))
    lib.Rls_Init.argtypes = [ct.POINTER(Rls), ct.c_uint8, ct.c_float, ct.c_float]
    lib.Rls_Reset.argtypes = [ct.POINTER(Rls)]
    lib.Rls_Update.argtypes = [ct.POINTER(Rls), ct.POINTER(ct.c_float), ct.c_float]
    for name in ('Rls_Init', 'Rls_Reset', 'Rls_Update'):
        getattr(lib, name).restype = None

    def update(state, phi, y):
        lib.Rls_Update(ct.byref(state), (ct.c_float * len(phi))(*phi), float(y))

    def batch(phi, y, lam, p0):
        weights = lam ** np.arange(len(y) - 1, -1, -1)
        information = phi.T @ (weights[:, None] * phi) + lam ** len(y) / p0 * np.eye(phi.shape[1])
        theta = np.linalg.solve(information, phi.T @ (weights * y))
        return theta, np.linalg.inv(information)

    rng = np.random.default_rng(20261030)
    for n in range(1, 5):
        phi = rng.normal(size=(400, n)).astype(np.float32).astype(float)
        target = np.array([.4, -.7, 1.2, .3])[:n]
        y = (phi @ target + rng.normal(scale=.01, size=len(phi))).astype(np.float32).astype(float)
        for forgetting in (1., .999, .98):
            state = Rls()
            lib.Rls_Init(ct.byref(state), n, forgetting, 100.)
            for row, obs in zip(phi, y):
                update(state, row, obs)
            theta, covariance = batch(phi, y, state.lam, state.p0)
            np.testing.assert_allclose(np.array(state.theta)[:n], theta, rtol=1e-9, atol=1e-10)
            np.testing.assert_allclose(np.array(state.p)[:n, :n], covariance, rtol=1e-9, atol=1e-10)
            previous, _ = batch(phi[:-1], y[:-1], state.lam, state.p0)
            np.testing.assert_allclose(state.y_hat, phi[-1] @ previous, atol=1e-10)
            np.testing.assert_allclose(state.error, y[-1] - state.y_hat, atol=1e-10)
            np.linalg.cholesky(np.array(state.p)[:n, :n])
    print('PASS: dimensions 1-4, three forgetting factors, independent batch theta/P and prediction residual')

    # Full-rank friction features exercise both directions at different speeds and loads.
    state = Rls()
    lib.Rls_Init(ct.byref(state), 4, .98, 100.)
    targets = [np.array([.2, -.4, .6, .8]), np.array([-.5, .3, -.2, .1])]
    for target in targets:
        for _ in range(1200):
            force, speed = rng.uniform(-1, 1, 2)
            phi = np.array([1., force, speed, np.sign(speed)], dtype=np.float32)
            update(state, phi, np.float32(phi @ target))
        np.testing.assert_allclose(np.array(state.theta), target, atol=1e-5)
        np.linalg.cholesky(np.array(state.p))
    print('PASS: four-parameter friction identification and tracking after parameter change')

    snapshot = bytes(state)
    for phi, y in [([1., 0., float('nan'), 1.], 0.),
                   ([1., 0., 0., float('inf')], 0.), ([1., 0., 0., 0.], float('nan')),
                   ([1., 0., 0., 0.], float('inf'))]:
        update(state, phi, y)
        assert bytes(state) == snapshot
    lib.Rls_Update(ct.byref(state), None, 0.)
    assert bytes(state) == snapshot
    for n, lam, p0 in [(0, .99, 1.), (5, .99, 1.), (4, 0., 1.), (4, 1.1, 1.),
                       (4, float('nan'), 1.), (4, .99, 0.), (4, .99, float('inf'))]:
        lib.Rls_Init(ct.byref(state), n, lam, p0)
        assert bytes(state) == snapshot
    config = state.num, state.lam, state.p0
    lib.Rls_Reset(ct.byref(state))
    assert (state.num, state.lam, state.p0) == config
    np.testing.assert_array_equal(state.theta, np.zeros(4))
    np.testing.assert_array_equal(state.p, np.eye(4) * state.p0)
    assert state.y_hat == state.error == 0.
    lib.Rls_Init(None, 4, .99, 1.)
    lib.Rls_Reset(None)
    lib.Rls_Update(None, None, 0.)
    print('PASS: invalid samples/configuration preserve state; reset preserves configuration')


if __name__ == '__main__':
    main()
