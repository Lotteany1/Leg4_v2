"""Identify passive sagittal hip damping from timestamped JustFloat captures."""
import argparse
from pathlib import Path

import numpy as np
from scipy.integrate import trapezoid
from scipy.linalg import logm, null_space
from scipy.optimize import lsq_linear

import run_pso as files


STATE_ROWS = [1, 5, 7, 9]
OUT = files.HERE / "build/hip_vofa_replay"


def continuous_models(data):
    models = []
    for point in data["models"]:
        system = np.eye(20)
        system[:14, :14], system[:14, 14:] = point["A"], point["B"]
        continuous = logm(system).real / data["dt"]
        a, b = continuous[STATE_ROWS, :14], continuous[STATE_ROWS, 14:]
        # Positive commanded hip torque accelerates the corresponding
        # leg-relative-to-body angle, so passive viscous torque opposes it.
        assert np.all(np.diag(b[1:3, 2:4] - b[3, 2:4]) > 0)
        models.append((point["height"], a, b, np.asarray(point["xb"]), np.asarray(point["u0"])))
    return models


def read_capture(path):
    samples = np.genfromtxt(path, delimiter=",", skip_header=1)
    if samples.ndim != 2 or samples.shape[1] != 54:
        raise ValueError(f"{path}: expected 54 channels")
    samples = samples[(samples[:, 53] == 5402) & np.isfinite(samples).all(axis=1)]
    if not len(samples) or np.any(np.diff(samples[:, 0]) <= 0):
        raise ValueError(f"{path}: no usable monotonic 5402 capture")
    ages = samples[:, 0, None] - samples[:, 43:45]
    return samples, ages


def collect(samples, ages, models, width, shift, project, band, coulomb):
    t = (samples[:, 0] - samples[0, 0]) / 1000
    cuts = np.r_[0, np.cumsum(np.diff(t) > .08)]
    length = samples[:, 45:47].mean(axis=1)
    rate = samples[:, 33:35]
    x = np.zeros((len(t), 14))
    x[:, [1, 3, 4, 6, 8, 9, 10, 11]] = samples[:, [8, 48, 11, 12, 10, 35, 47, 52]]
    x[:, [5, 7]] = rate + x[:, 9, None]
    usable = ((length >= band[0]) & (length <= band[1])
              & (abs(samples[:, 48]) <= .20) & (abs(samples[:, 47]) <= .06)
              & (abs(samples[:, 45] - samples[:, 46]) <= .035)
              & (abs(samples[:, 11:13]).max(axis=1) <= .30)
              & (abs(samples[:, 25:27]).max(axis=1) < 4.90)
              & (samples[:, 29] == 1) & (samples[:, 27] == 63))
    if not project:
        usable &= ((ages >= 0) & (ages <= 4)).all(axis=1) & (samples[:, 43:45] > 0).all(axis=1)
    windows = []
    for block in np.unique(cuts):
        indexes = np.flatnonzero(cuts == block)
        t_block = t[indexes]
        if len(indexes) < 5:
            continue
        raw = samples[indexes]
        torque = np.column_stack([
            np.interp(t_block + offset, (raw[:, tick] - samples[0, 0]) / 1000, raw[:, channel])
            if np.all(np.diff(raw[:, tick]) > 0) and (raw[:, tick] > 0).all() else raw[:, channel]
            for channel, tick, offset in [(41, 43, shift), (42, 44, shift),
                                           (21, 36, shift), (22, 36, shift)]
        ])
        i = 1
        while i < len(indexes) - 1:
            end = np.searchsorted(t_block, t_block[i] + width)
            if end >= len(indexes) - 1:
                break
            ix = indexes[i:end + 1]
            if (not usable[ix].all() or np.max(np.diff(t[ix])) > .035
                    or np.max(abs(rate[ix])) < .20 or np.ptp(samples[ix, 49]) > .031):
                i += 1
                continue
            h, a, b, xb, u0 = min(models, key=lambda item: abs(item[0] - samples[ix, 49].mean()))
            states = x[ix].copy()
            states[:, 12] = xb[12] + samples[ix, 49] - h
            inputs = np.column_stack((torque[i:end + 1], samples[ix, 50:52]))
            basis = null_space(b[:, :2].T).T if project else np.eye(4)
            response = (states[-1, STATE_ROWS] - states[0, STATE_ROWS]) @ basis.T
            nominal = (states - xb) @ a.T + (inputs - u0) @ b.T
            hip = b[:, 2:4]
            terms = [-rate[ix] @ hip.T, states[:, 4, None] * hip[:, 0],
                     states[:, 6, None] * hip[:, 1], np.broadcast_to(hip.sum(axis=1), nominal.shape)]
            if coulomb:
                terms.append(-np.tanh(rate[ix] / .08) @ hip.T)
            design = basis @ trapezoid(np.stack(terms, axis=-1), t[ix], axis=0)
            response -= basis @ trapezoid(nominal, t[ix], axis=0)
            windows.append((float(t[ix[0]]), design, response))
            i = end + 1
    return windows


def solve(windows, fixed_zero=False):
    x = np.concatenate([w[1] for w in windows])
    y = np.concatenate([w[2] for w in windows])
    if fixed_zero:
        return np.r_[0., np.linalg.lstsq(x[:, 1:], y, rcond=None)[0]]
    return lsq_linear(x, y, bounds=(np.r_[0., np.full(x.shape[1] - 1, -np.inf)], np.inf)).x


def error(windows, coefficients):
    residual = np.concatenate([design @ coefficients - y for _, design, y in windows])
    return float(np.sqrt(np.mean(residual**2)))


def identify(paths, data):
    models = continuous_models(data)
    captures = [read_capture(path) for path in paths]
    sizes = [{"file": str(p), "sha256": files.digest(p), "frames": len(s),
              "time_s": float((s[-1, 0] - s[0, 0]) / 1000),
              "wheel_age_ms": np.percentile(a, [0, 50, 100]).tolist(),
              "wheel_stamp_valid": float(np.mean((s[:, 43:45] > 0) & (a >= 0) & (a <= 4)))}
             for p, (s, a) in zip(paths, captures, strict=True)]
    trials = []
    for band in ((.155, .205), (.235, .275)):
        for width in (.10, .14):
            for shift in (-.008, 0, .008):
                for project in (False, True):
                    for coulomb in (False, True):
                        groups = [collect(s, a, models, width, shift, project, band, coulomb)
                                  for s, a in captures]
                        groups = [g for g in groups if len(g) >= 5]
                        independent = len(groups) >= 2
                        if len(groups) == 1:
                            rows = groups[0]
                            midpoint = (rows[-1][0] + rows[0][0]) / 2
                            groups = [[w for w in rows if w[0] < midpoint - .18],
                                      [w for w in rows if w[0] >= midpoint + .18]]
                        if len(groups) < 2 or min(map(len, groups)) < 5:
                            continue
                        folds = []
                        for index, test in enumerate(groups):
                            train = [w for j, g in enumerate(groups) if j != index for w in g]
                            coefficients = solve(train)
                            folds.append({"windows": len(test), "damping": float(coefficients[0]),
                                          "fit_rmse": error(test, coefficients),
                                          "zero_rmse": error(test, solve(train, True))})
                        trials.append({"length": band, "window_s": width, "shift_s": shift,
                                       "projection": project, "coulomb": coulomb,
                                       "independent_capture": independent, "folds": folds})
    return {"captures": sizes, "trials": trials}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="+", type=Path)
    parser.add_argument("--model", type=Path, default=OUT / "model.json")
    parser.add_argument("--output", type=Path, default=OUT / "identification.json")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    files.write_json(args.output, identify(args.capture, files.read_json(args.model)))


if __name__ == "__main__":
    main()
