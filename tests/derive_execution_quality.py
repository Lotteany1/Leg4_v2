"""Derive conservative frozen-model averaging bounds without deploying matrices.

The augmented logarithm is used ONLY as an offline kernel reference. No 1 ms
matrix or replacement plant model is deployed. The factor-two grid envelope is
validated numerically, not a formal continuous-geometry or hardware guarantee.
"""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
import argparse
import importlib.util
import json
import re
import numpy as np
from scipy.linalg import logm, expm

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT.parent / 'research/actuator_coordination_20261101_01/multirate_experiment.py'
spec = importlib.util.spec_from_file_location('quality_reference', MODEL_PATH)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


def coefficient(ad, bd, model_dt, interval, bins, row):
    nx, nu = bd.shape
    augmented = np.block([[ad, bd], [np.zeros((nu, nx)), np.eye(nu)]])
    continuous = logm(augmented) / model_dt
    assert np.max(np.abs(continuous.imag)) < 1e-7
    ac = continuous.real[:nx, :nx]
    bc = continuous.real[:nx, nx:]
    # |d exp(A*t)B/dt| <= |A| exp(|A|T) |B|.
    derivative = np.abs(ac) @ expm(np.abs(ac) * interval) @ np.abs(bc)
    return bins * (interval / bins)**2 / 6 * derivative[row]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--write-config', '--write-header', dest='write_config', action='store_true',
                        help='Write an isolated config fragment; never modify production configuration')
    parser.add_argument('--output', type=Path, default=ROOT / 'build/distributor_check/execution_quality_generated.h')
    args = parser.parse_args()
    maximum = np.zeros(4)
    for left in np.linspace(.16, .34, 13):
        for right in np.linspace(.16, .34, 13):
            ad, bd = reference.balance_model(left, right)
            maximum = np.maximum(maximum, coefficient(ad, bd, .002, .004, 2, 8))
    ad, bd = reference.leg_model()
    roll = coefficient(ad, bd, .010, .010, 1, 0)
    height = coefficient(ad, bd, .010, .010, 1, 2)
    values = dict(pitch=(2*maximum).tolist(), roll=(2*roll).tolist(), height=(2*height).tolist())
    # Independently integrate bounded event sequences with the continuous offline kernel.
    rng = np.random.default_rng(20261101)
    ratios = []
    for case in range(80):
        balance = case % 2 == 0
        ad, bd = reference.balance_model(*rng.uniform(.16, .34, 2)) if balance else reference.leg_model()
        dt = .002 if balance else .010
        bins = 2 if balance else 1
        nx, nu = bd.shape
        augmented = np.block([[ad, bd], [np.zeros((nu, nx)), np.eye(nu)]])
        generator = logm(augmented).real / dt
        kernel = expm(generator * dt / 8)
        exact = np.zeros(nx); mean_state = np.zeros(nx)
        values_in = rng.uniform(-100, 100, (bins, 8, nu))
        for node in range(bins):
            for command in values_in[node]: exact = kernel[:nx,:nx] @ exact + kernel[:nx,nx:] @ command
            mean_state = ad @ mean_state + bd @ values_in[node].mean(axis=0)
        ranges = np.ptp(values_in.reshape(-1, nu), axis=0)
        checks = [(8, values['pitch'])] if balance else [(0, values['roll']), (2, values['height'])]
        for row, coefficients in checks:
            bound = np.dot(coefficients, ranges)
            ratio = abs(exact[row] - mean_state[row]) / bound
            assert ratio <= 1.0, (case, row, ratio)
            ratios.append(float(ratio))
    output = ROOT / 'build/distributor_check'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'averaging_bounds.json').write_text(json.dumps(dict(**values, event_sequence_cases=80, max_error_bound_ratio=max(ratios)), indent=2))
    config_path = ROOT / 'Code/Control/distributor.h'
    config = config_path.read_text(encoding='utf-8')
    fragment = '// Frozen-model bounds from derive_execution_quality.py; review before production sync.\n'
    for name, data in values.items():
        macro = 'EXECUTION_' + name.upper() + '_BOUND'
        body = re.search(r'#define\s+' + macro + r'\s+\{([^}]+)\}', config)[1]
        actual = np.array([float(x.rstrip('f')) for x in body.split(',')])
        assert np.allclose(actual, data, rtol=1e-9, atol=0), (name, actual, data)
        fragment += '#define ' + macro + ' {' + ', '.join(f'{v:.9e}f' for v in data) + '}\n'
    if args.write_config:
        # All generated output stays under build; aliases cannot target a production file.
        destination = args.output.resolve()
        assert destination.is_relative_to((ROOT / 'build').resolve()), 'Generated config must remain under build/'
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(fragment, encoding='utf-8')
    print(json.dumps(values))

if __name__ == '__main__':
    main()
