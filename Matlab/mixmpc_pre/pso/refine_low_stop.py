"""Refine s59_trial01 in isolated nonlinear MPC trials."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.stats import qmc

from fit_hip_vofa import OUT as PRIOR
import run_pso as files
from verify_hip_vofa_ppo import compile_replay


OUT = files.HERE / "build/low_stop_refine"
AXES = ((0,), (1,), (4, 6), (5, 7), (8,), (9,), (14, 15), (16, 17))
# Height, hip damping, speed request, parking offset, 2-ms motor delay,
# added load, pitch kick and roll kick. Indices for the two mid checks follow.
TRAIN = [
    [.19, 2.1, -.71, 0, 6, .15, -.10, .05],
    [.25, .6, .59, 0, 4, .20, .10, .05],
    [.26, 2.3, -.72, 0, 6, .20, -.12, .05],
]
HELD = [
    [.19, 1.30, -.66, 0, 5, .10, -.075, -.045],
    [.19, 2.65, -.79, 0, 7, .23, -.13, .060],
    [.185, 3.8, -.68, 0, 5, .16, -.10, .030],
    [.195, .75, -.74, 0, 6, .19, -.13, -.030],
    [.17, 0, -.52, 0, 3, .05, -.09, .05],
    [.18, .9, .61, 0, 4, .20, .11, .05],
    [.17, 5.4, .63, 0, 9, .25, .13, .05],
    [.18, 7.5, 0, -.10, 5, .10, -.12, .05],
    [.19, 1.6, 0, .11, 4, .15, .12, .05],
    [.24, 1.05, .55, 0, 4, .13, .09, -.04],
    [.255, 1.9, -.67, 0, 5, .24, -.11, .04],
    [.265, 3.0, -.76, 0, 7, .19, -.13, .05],
    [.24, 5.0, 0, -.10, 7, .10, -.10, .05],
    [.25, 0, 0, .12, 3, .15, .09, .05],
]
HORIZON = 3.2
RELEASE = 1.52
DWELL = .20
ATT_TOL = .002


def weights(row):
    return np.asarray(row["Q_DIAG"] + row["R_DIAG"])


def candidate(name, center, z):
    w = center.copy()
    for group, value in zip(AXES, z, strict=True):
        w[list(group)] *= np.exp(value)
    return {"id": name, "Q_DIAG": w[:14].tolist(), "R_DIAG": w[14:].tolist()}


def proposals(center, seed, size, radius):
    sampler = qmc.LatinHypercube(len(AXES), seed=seed)
    moves = (2 * sampler.random(size) - 1) * np.log(radius)
    return [candidate(f"seed{seed}_{i:02d}", center, z) for i, z in enumerate(moves, 1)]


def braking_proposals(center):
    # Vary leg-rate weighting separately from the speed tracking and
    # wheel/hip effort tradeoff. Keep the selected PPO point as the anchor.
    combinations = [
        (.25, 1.00, 1.00, 1.00), (.45, 1.00, 1.00, 1.00),
        (.60, 1.00, 1.00, 1.00), (.52, 1.28, 1.00, 1.00),
        (.52, 1.00, .72, 1.00), (.52, 1.00, 1.00, .72),
        (.52, 1.00, 1.00, 1.35), (.50, 1.26, .79, 1.12),
        (.36, 1.38, 1.12, .81), (.70, 1.36, 1.23, .73),
        (.35, 1.15, .79, 1.29), (.72, 1.08, .86, .76),
    ]
    rows = []
    for index, multipliers in enumerate(combinations, 1):
        z = np.zeros(len(AXES))
        z[[3, 1, 6, 7]] = np.log(multipliers)
        rows.append(candidate(f"brake{index:02d}", center, z))
    return rows


def attitude_proposals(center):
    # The braking branch recovered stopping by lowering the leg-rate cost.
    # Test whether added leg-angle cost can trim the mid-height lean peak.
    combinations = [
        (.25, 1.8, 1., 1., 1.), (.25, 2.5, 1., 1., 1.),
        (.25, 3.3, 1., 1., 1.), (.36, 1.8, 1., 1., 1.),
        (.36, 2.5, 1., 1., 1.), (.45, 1.8, 1., 1., 1.),
        (.45, 2.5, 1., 1., 1.), (.33, 1.9, .75, 1., 1.15),
        (.33, 2.6, .8, 1., 1.15), (.40, 2.8, .9, .92, 1.22),
        (.30, 2.2, .70, 1.12, 1.10), (.26, 3.1, .85, 1.10, 1.22),
    ]
    rows = []
    for index, multipliers in enumerate(combinations, 1):
        z = np.zeros(len(AXES))
        z[[3, 2, 0, 1, 7]] = np.log(multipliers)
        rows.append(candidate(f"lean{index:02d}", center, z))
    return rows


def tracking_proposals(center):
    # More speed-error authority can recover mid-height braking after
    # the larger angle cost trims the peak lean.
    combinations = [
        (.30, 2.5, .75, 1.70, 1., 1.15), (.30, 3.3, .75, 2.30, 1., 1.15),
        (.24, 3.8, .85, 2.75, 1., 1.15), (.26, 2.9, .65, 1.90, .72, 1.15),
        (.26, 3.6, .75, 2.35, .78, 1.25), (.20, 3.9, .75, 2.20, .8, 1.20),
        (.28, 2.6, .75, 1.58, .9, .50), (.26, 3.3, .75, 2.05, .9, .52),
        (.31, 2.7, .85, 1.8, .68, 1.5), (.38, 2.8, .70, 1.75, .78, 1.1),
        (.20, 3.0, .67, 1.6, .85, .72), (.14, 2.8, .65, 1.62, 1., 1.4),
    ]
    rows = []
    for index, multipliers in enumerate(combinations, 1):
        z = np.zeros(len(AXES))
        z[[3, 2, 0, 1, 6, 7]] = np.log(multipliers)
        rows.append(candidate(f"track{index:02d}", center, z))
    return rows


def assessment(row, baseline, center, focus, mid):
    m = row["cases"]
    if len(m) != len(baseline["cases"]) or not all(c["finite"] for c in m):
        return {"pass": False, "rank": [10, 10, 10, 10], "failures": ["diverged"]}
    cb, cc = baseline["cases"], center["cases"]
    failures = []
    attitude = max(c["attitude_peak"] - (min(cb[i]["attitude_peak"], cc[i]["attitude_peak"])
                   if i in focus else cb[i]["attitude_peak"]) for i, c in enumerate(m))
    if attitude > ATT_TOL:
        failures.append("attitude")
    unstable = sum(not m[i]["locked"] or m[i]["late_speed_peak"] > .06
                   or m[i]["speed_stop_s"] > HORIZON - RELEASE - DWELL for i in focus)
    if unstable:
        failures.append("stop")
    track = max(m[i]["speed_rms"] - cb[i]["speed_rms"] for i in mid)
    if track > 1e-6 or any(m[i]["speed_rise_lag_s"] > cb[i]["speed_rise_lag_s"] + .0021
                           or m[i]["speed_stop_s"] > cb[i]["speed_stop_s"] + .0021 for i in mid):
        failures.append("mid_tracking")
    if any(m[i]["fallback"] > cb[i]["fallback"] + .01 or m[i]["box"] > cb[i]["box"] + .02
           for i in range(len(m))):
        failures.append("limits")
    residual = max(m[i]["late_speed_peak"] for i in focus)
    stop = max(m[i]["speed_stop_s"] for i in focus)
    rank = [int(attitude > ATT_TOL), len(failures), unstable, residual, stop, attitude, track]
    return {"pass": not failures, "rank": rank, "failures": failures,
            "focus_speed_peak": residual, "focus_stop_s": stop,
            "attitude_regression_rad": attitude, "mid_speed_rms_delta": track}


def replay(args, out, stage, cases, rows, swing=False):
    folder = out / stage
    folder.mkdir(exist_ok=True)
    helper = compile_replay(stage, out)
    source = (helper / "hip_vofa_replay.m").read_text(encoding="utf-8")
    tail = source[source.index("function m = simulate("):]
    marker = "m.late_speed_rms=rms(trace(2,late));"
    assert tail.count(marker) == 1
    tail = tail.replace(marker, marker + "\nm.late_speed_peak=max(abs(trace(2,late)));")
    if swing:
        tail = tail.replace("ref(2)=s(8)*ramp;", """ref(2)=s(8)*ramp;
    if p.SWING_REVERSE
        turn_blend=min(1,max(0,(t(k)-.74)/.28));
        ref(2)=ref(2)*(2*turn_blend-1);
    end
   """)
        tail = tail.replace(marker, """m.leg_peak=max(abs(trace([5 7],j)),[],'all');
off=j(t(j)>=1.52);
if isempty(off), m.leg_release_peak=m.leg_peak;
else, m.leg_release_peak=max(abs(trace([5 7],off)),[],'all'); end
""" + marker)
    (helper / "low_stop_replay.m").write_text(
        (files.HERE / "matlab/low_stop_replay.m.txt").read_text(encoding="utf-8") + "\n" + tail,
        encoding="utf-8")
    plan = folder / "plan.json"
    payload = {"scenes": cases, "rows": rows}
    if plan.is_file() and files.read_json(plan) != payload:
        raise ValueError(f"Run folder has a different plan: {folder}")
    files.write_json(plan, payload)
    mix = out / "src/Matlab/mixmpc_pre"
    def remaining():
        return [i for i, row in enumerate(rows, 1) if not (folder / f"{row['id']}.json").is_file()]

    pending = remaining()
    batches = [pending[i::args.workers] for i in range(args.workers)]

    def run(indices):
        selection = ",".join(map(str, indices))
        command = (f"mixmpc_paths();addpath({files.mq(helper)});"
                   f"low_stop_replay({files.mq(folder)},[{selection}]);")
        return files.matlab(args, mix, command, folder / f"matlab_{indices[0]:03d}.log")

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        codes = list(pool.map(run, filter(None, batches)))
    pending = remaining()
    if pending and run(pending):
        raise RuntimeError(f"MATLAB stage {stage} returned {codes}")
    if remaining():
        raise RuntimeError(f"MATLAB stage {stage} has missing rows")
    return [files.read_json(folder / f"{r['id']}.json") for r in rows]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab", default="matlab")
    parser.add_argument("--size", type=int, default=18, help="Latin hypercube samples per restart")
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--workers", type=int, default=4)
    branch = parser.add_mutually_exclusive_group()
    branch.add_argument("--directed", action="store_true", help="Run the braking-focused branch")
    branch.add_argument("--attitude", action="store_true", help="Balance recovered braking against mid-height lean")
    branch.add_argument("--tracking", action="store_true", help="Recover mid-height braking with lean constrained")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    mix = out / "src/Matlab/mixmpc_pre"
    if not mix.is_dir():
        files.prepare(out)
    base_data = files.read_json(PRIOR / "mpc/baseline.json")
    center_data = files.read_json(PRIOR / "mpc/s59_trial01.json")
    origin = weights(center_data)
    base = candidate("baseline", weights(base_data), np.zeros(len(AXES)))
    center = candidate("s59_trial01", origin, np.zeros(len(AXES)))
    model_config = (out / "baseline.m.txt").read_bytes()
    if model_config != (PRIOR / "baseline.m.txt").read_bytes():
        raise ValueError("Active MATLAB configuration differs from the VOFA replay baseline")
    directions = np.eye(len(AXES)) * np.log(1.20)
    candidates = [candidate(f"axis{i:02d}_{sign}", origin, delta * sign)
                  for i, delta in enumerate(directions, 1) for sign in (-1, 1)]
    candidates += proposals(origin, 1931, args.size, 1.42)
    runtime = SimpleNamespace(matlab=args.matlab, workers=args.workers)
    ranked = []
    stage = "screen"
    if args.directed:
        stage = "braking"
        candidates = braking_proposals(origin)
    if args.attitude:
        stage = "attitude"
        candidates = attitude_proposals(origin)
    if args.tracking:
        stage = "tracking"
        candidates = tracking_proposals(origin)
    first = replay(runtime, out, stage, TRAIN, [base, center, *candidates])
    for row in first[2:]:
        ranked.append((row, assessment(row, first[0], first[1], [0], [1, 2])))
    ranked.sort(key=lambda item: item[1]["rank"])
    files.write_json(out / f"{stage}_rank.json", [{"id": r["id"], **v} for r, v in ranked])
    if stage == "screen":
        best = weights(ranked[0][0])
        refinements = proposals(best, 3907, args.size, 1.18)
        second = replay(runtime, out, "refine", TRAIN, refinements)
        ranked.extend((row, assessment(row, first[0], first[1], [0], [1, 2])) for row in second)
        ranked.sort(key=lambda item: item[1]["rank"])
        files.write_json(out / "refine_rank.json", [{"id": r["id"], **v} for r, v in ranked])
    count = 6 if stage == "screen" else 3
    shortlist = [candidate(r["id"], weights(r), np.zeros(len(AXES))) for r, _ in ranked[:count]]
    held_stage = "heldout" if stage == "screen" else f"{stage}_heldout"
    held = replay(runtime, out, held_stage, [*TRAIN, *HELD], [base, center, *shortlist])
    focus, mid = [0, 3, 4, 5, 6], [1, 2, 12, 13, 14]
    results = [{"id": row["id"], **assessment(row, held[0], held[1], focus, mid)}
               for row in held[2:]]
    results.sort(key=lambda item: item["rank"])
    result_name = "results.json" if stage == "screen" else f"{stage}_results.json"
    files.write_json(out / result_name, {"screened": len(ranked), "focus_indices": focus,
                     "mid_indices": mid, "heldout": results})
    print(results, flush=True)


if __name__ == "__main__":
    main()
