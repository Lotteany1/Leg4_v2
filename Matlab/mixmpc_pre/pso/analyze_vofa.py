"""Use board clocks to audit the historical TinyMPC14 height/roll captures."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

FILES = ("vofa_012529.csv", "vofa_014327.csv", "vofa_020655.csv", "vofa_022412.csv")


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def read(path):
    raw = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    keep = np.isfinite(raw).all(axis=1) & (raw[:, 0] > 0) & (abs(raw).max(axis=1) < 1e8)
    rows = raw[keep]
    if len(rows) < 2 or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError(f"{path.name}: need strictly increasing board ticks")
    return rows, len(raw) - len(rows)


def mean_clock(t, x):
    return float(np.trapezoid(x, t) / (t[-1] - t[0]))


def plateau_metrics(rows):
    time = rows[:, 2] / 1000
    cuts = np.r_[0, np.flatnonzero(np.diff(rows[:, 27])) + 1, len(rows)]
    result = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        target = rows[a, 27]
        if b - a < 3 or time[b-1] - time[a] < .5 or target not in (.17, .33, .34):
            continue
        w, t = rows[a:b], time[a:b]
        if np.any(np.diff(t) <= 0):
            continue
        err = w[:, 28] - target
        error_roll = w[:, 5] - w[:, 9]
        # A finite recovery needs at least a 0.25 s observed tail inside the band.
        bad = np.flatnonzero(abs(err) > .01)
        recovered = 0 if not len(bad) else bad[-1] + 1
        roll_bad = np.flatnonzero((abs(error_roll) > .02) | (abs(w[:, 7]) > .15))
        roll_recovered = 0 if not len(roll_bad) else roll_bad[-1] + 1
        s = {"start_s": t[0], "end_s": t[-1], "reference_m": target,
             "height_mean_m": mean_clock(t, w[:, 28]), "height_rmse_m": rms(err),
             "raw_height_mean_m": mean_clock(t, w[:, 11]),
             "height_settle_s": float(t[recovered]-t[0]) if recovered < len(t) and t[-1]-t[recovered] >= .25 else None,
             "roll_recovery_s": float(t[roll_recovered]-t[0]) if roll_recovered < len(t) and t[-1]-t[roll_recovered] >= .25 else None,
             "roll_rms_rad": rms(error_roll), "roll_peak_rad": float(max(abs(error_roll))),
             "roll_rate_rms_rad_s": rms(w[:, 7]),
             "fallback_fraction": float((w[-1, 22]-w[0, 22])/(w[-1, 0]-w[0, 0]))}
        for name, common, diff in [("request", w[:, 32], w[:, 13]),
                                   ("accepted", w[:, 34], w[:, 15]),
                                   ("estimated", w[:, 35]+w[:, 36], w[:, 16]+w[:, 17])]:
            s[name] = {"sum_mean_n": mean_clock(t, common), "diff_rms_n": rms(diff),
                       "one_leg_near_20_n": mean_clock(t, (((common-abs(diff))/2) <= 20.15).astype(float))}
        s["acceptance_max_sum_delta_n"] = float(max(abs(w[:, 34]-w[:, 32])))
        # The input is a ramp. Bracket onset between observed samples and
        # report crossing only on a measurable excursion beyond CSV rounding.
        if a and result and target != result[-1]["reference_m"]:
            before = result[-1]
            i = int(np.searchsorted(time, before["end_s"]))
            onset = time[max(0, i)]
            segment = rows[i:b]
            direction = np.sign(target - before["reference_m"])
            x0 = rows[i, 28]
            span = abs(target - x0)
            reference_10 = np.flatnonzero(direction*(segment[:, 27]-before["reference_m"]) >= .1*abs(target-before["reference_m"]))
            obs_10 = np.flatnonzero(direction*(segment[:, 28]-x0) >= max(.02, .1*span))
            if len(reference_10) and len(obs_10):
                s["height_detected_delay_s"] = float((segment[obs_10[0], 2]-segment[reference_10[0], 2])/1000)
            else:
                s["height_detected_delay_s"] = None
            s["ref_transition_first_bracket_s"] = [float(onset), float(time[min(i+1, b-1)])]
            s["height_overshoot_m"] = float(max(0, max(direction*(segment[:, 28]-target))))
        result.append(s)
    return result


def harmonic(time, values, frequency):
    # Use absolute board time for phase alignment across async clocks.
    theta = 2*np.pi*frequency*time
    design = np.column_stack((np.ones(len(time)), time-time.mean(), np.cos(theta), np.sin(theta)))
    weight = np.sqrt(np.minimum(np.gradient(time), .04))
    coefficient = np.linalg.lstsq(design*weight[:, None], values*weight, rcond=None)[0]
    return complex(coefficient[2], -coefficient[3]), np.linalg.norm(weight*(design@coefficient-values))


def window(rows, lo, hi):
    def series(clock, values):
        t = rows[:, clock]/1000
        ids = np.flatnonzero((lo <= t) & (t < hi))
        _, keys = np.unique(t[ids], return_index=True)
        ids = ids[keys]
        return t[ids], values[ids]

    t, gyro = series(0, rows[:, 6])
    frequency = min(np.arange(8, 14, .01), key=lambda f: harmonic(t, gyro, f)[1])
    def amp(clock, value):
        return harmonic(*series(clock, value), frequency)[0]

    source = amp(40, rows[:, 42])
    motor = amp(0, rows[:, 16]+rows[:, 17])
    ratio = motor/source
    roll_ratio = amp(0, rows[:, 7])/amp(0, rows[:, 6])
    height_ratio = amp(2, rows[:, 28])/amp(2, rows[:, 11])
    return {"board_window_s": [lo, hi], "frequency_hz": float(frequency),
            "feedback_minus_can_deg": float(np.angle(ratio, deg=True)),
            "feedback_to_can_gain": abs(ratio),
            "roll_filter_deg": float(np.angle(roll_ratio, deg=True)),
            "height_filter_deg": float(np.angle(height_ratio, deg=True)),
            "apparent_transport_plus_response_ms": float(-1000*np.angle(ratio)/(2*np.pi*frequency)),
            "roll_filter_effective_delay_ms": float(-1000*np.angle(roll_ratio)/(2*np.pi*frequency))}


def replay_plant(rows, calibration, interval=None):
    """Short held-out segments with measured CAN input and no fitted plant gains."""
    time = rows[:, 0]*.001
    observation = rows[:, 2]*.001
    if rows.shape[1] >= 50:
        clocks = np.hstack((rows[:, 40], rows[:, 43]))*.001
        common = np.hstack((rows[:, 41], rows[:, 44]))
        diff = np.hstack((rows[:, 42], rows[:, 45]))
        stamps, ids = np.unique(clocks, return_index=True)
        keep = stamps > 0
        clocks, common, diff = stamps[keep], common[ids[keep]], diff[ids[keep]]
    else:
        clocks, ids = np.unique(observation, return_index=True)
        common, diff = rows[ids, 34], rows[ids, 15]
    tau = calibration["estimated_tau_ms"]*.001
    lag = calibration["assumed_transport_ms"]*.001
    gain = calibration["fitted_gain"]
    step = .001
    horizon = .12
    results = []
    start, end = interval or (time[0], time[-1])
    for a in np.arange(start+.05, end-horizon, .15):
        b = a+horizon
        inside = (time >= a-.03) & (time <= b+.03)
        if np.count_nonzero(inside) < 6 or np.max(np.diff(time[inside])) > .04:
            continue
        mean_force = np.interp(a-lag, clocks, common)
        diff_force = gain*np.interp(a-lag, clocks, diff)
        h = np.interp(a, time, rows[:, 11])
        dh = np.interp(a, observation, rows[:, 29])
        roll = np.interp(a, time, rows[:, 5])
        droll = np.interp(a, time, rows[:, 6])
        mix = -np.expm1(-step/tau)
        for t in np.arange(a, b-step/2, step):
            mean_force += mix*(np.interp(t-lag, clocks, common)-mean_force)
            diff_force += mix*(gain*np.interp(t-lag, clocks, diff)-diff_force)
            psi = [np.interp(t, observation, rows[:, k]) for k in (46, 47)] if rows.shape[1] >= 50 else [0., 0.]
            axial = np.array([(mean_force+diff_force)/2, (mean_force-diff_force)/2])
            h_arm = 1/(np.cos(roll)*np.cos(psi))
            r_arm = h_arm*(h*np.tan(roll)+np.array([.212755, -.212755])/np.cos(roll))
            heave_force = h_arm@axial-144.53-8*dh
            roll_moment = r_arm@axial-144.53*h*np.tan(roll)+.5*16.2*9.81*(h+.01863)*roll
            dh += step*heave_force/14.2903
            droll += step*roll_moment/.578
            h += step*dh
            roll += step*droll
        results.append((h-np.interp(b, time, rows[:, 11]), roll-np.interp(b, time, rows[:, 5])))
    errors = np.asarray(results)
    return {"horizon_s": horizon, "windows": len(results),
            "height_rmse_m": rms(errors[:, 0]) if len(errors) else None,
            "roll_rmse_rad": rms(errors[:, 1]) if len(errors) else None}

def analyze(folder):
    result = {"captures": {}, "calibration": None, "heldout": None}
    for name in FILES:
        path = folder / name
        if not path.is_file():
            continue
        rows, bad = read(path)
        result["captures"][name] = {
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "channels": rows.shape[1],
            "valid_rows": len(rows), "discarded": bad,
            "cadence_ms_median": float(np.median(np.diff(rows[:, 0]))),
            "fallback_fraction": float((rows[-1, 22]-rows[0, 22])/(rows[-1, 0]-rows[0, 0])),
            "force_paired_fraction": float(np.mean((rows[:, 20].astype(np.int64) & (2**20+2**21)) == (2**20+2**21))),
            "leg_lengths_m": [float(np.min(rows[:, 30:32])), float(np.max(rows[:, 30:32]))],
            "plateaus": plateau_metrics(rows)}
        if name == "vofa_022412.csv" and rows.shape[1] >= 50:
            cal = window(rows, 28.725, 30.465)
            test = window(rows, 26.535, 27.985)
            # One frequency cannot separately identify gain, dead time and tau.
            # Fix transport to a fixture setting and estimate the remaining phase.
            transport = .004
            omega = 2*np.pi*cal["frequency_hz"]
            tau = np.tan(-np.radians(cal["feedback_minus_can_deg"])-omega*transport)/omega
            dc_gain = cal["feedback_to_can_gain"] * abs(1+1j*omega*tau)
            w = 2*np.pi*test["frequency_hz"]
            predicted = dc_gain*np.exp(-1j*w*transport)/(1+1j*w*tau)
            result["calibration"] = dict(cal, assumed_transport_ms=1000*transport,
                                         estimated_tau_ms=float(1000*tau), fitted_gain=float(dc_gain))
            result["heldout"] = dict(test, predicted_deg=float(np.angle(predicted, deg=True)),
                                     predicted_gain=abs(predicted),
                                     phase_error_deg=float(np.angle(predicted, deg=True)-test["feedback_minus_can_deg"]),
                                     gain_error=float(abs(predicted)-test["feedback_to_can_gain"]))
    if result["calibration"]:
        latest, _ = read(folder / "vofa_022412.csv")
        result["plant_replay"] = {
            "vofa_022412_heldout_high": replay_plant(latest, result["calibration"], (26.535, 27.985))}
        previous = folder / "vofa_020655.csv"
        if previous.is_file():
            rows, _ = read(previous)
            result["plant_replay"]["vofa_020655"] = replay_plant(rows, result["calibration"])
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    opts = parser.parse_args()
    print(json.dumps(analyze(opts.capture_dir), ensure_ascii=False, indent=2))


