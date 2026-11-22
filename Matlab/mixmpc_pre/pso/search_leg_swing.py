"""Screen forward/reverse leg swing around the installed brake01 firmware."""
import argparse
import re
from pathlib import Path

import numpy as np
from scipy.stats import qmc

import run_pso as files
from refine_low_stop import AXES, candidate, replay, weights


OUT = files.HERE / "build/leg_swing"
TRAIN = [
    [.175, 2.1, -1.55, 0, 6, .15, -.10, .04, 0],
    [.185, 1.1, 1.28, 0, 5, .15, .08, -.04, 1],
    [.25, .60, .70, 0, 4, .20, .10, .05, 0],
]
HOLD = [
    [.17, .75, -1.58, 0, 5, .11, -.11, .045, 0],
    [.185, 3.35, -1.40, 0, 7, .24, -.15, -.06, 1],
    [.195, 5.2, 1.26, 0, 8, .24, .15, .05, 0],
    [.19, 2.1, -.71, 0, 6, .15, -.10, .05, 0],
    [.25, 1.2, -.95, 0, 4, .20, -.10, .04, 0],
    [.265, 2.60, .78, 0, 6, .23, .13, -.05, 0],
]
CHALLENGE = [
    [.18, 1.6, -1.52, 0, 6, .19, -.13, .055, 1],
    [.175, 3.1, 1.48, 0, 7, .25, .12, -.045, 0],
    [.19, 1.25, -.76, 0, 5, .17, -.11, .02, 0],
    [.255, 2.0, -.86, 0, 6, .20, -.09, -.04, 0],
]


def active_cost(path):
    source = path.read_text(encoding="utf-8")
    return [[float(x.strip().removesuffix("f")) for x in re.search(
        rf"tinympc14_{key}_diag\[\d+\]\s*=\s*\{{([^}}]+)\}}", source)[1].split(",")]
            for key in ("q", "r")]


def generate(origin):
    # Position, speed, leg angle, leg-rate, wheel effort and hip effort.
    fixed = [
        (1, 1, 1.7, 1, 1, 1), (1, 1, 2.5, 1, 1, 1),
        (1, 1, 3.3, 1, 1, 1), (1, 1, 4.5, 1, 1, 1),
        (.75, 1, 3.0, 1, 1, 1), (.75, 1.5, 3.5, 1, 1, 1),
        (.8, 1.8, 4.2, 1, .85, .8), (.8, 1, 3.1, 1.6, .9, .65),
        (.7, .85, 2.3, 1.8, 1, .5), (.85, 1.12, 2.8, 2.5, 1, .65),
        (.8, .9, 3.8, 1.25, .8, .75), (1.1, 1, 2.0, 1.45, 1.2, 1.4),
    ]
    lo = [.65, .72, 1.55, .8, .75, .50]
    hi = [1.25, 1.85, 4.6, 2.8, 1.3, 1.4]
    moves = qmc.scale(qmc.LatinHypercube(6, seed=841).random(12), np.log(lo), np.log(hi))
    for i, delta in enumerate([*np.log(fixed), *moves], 1):
        z = np.zeros(len(AXES))
        z[[0, 1, 2, 3, 6, 7]] = delta
        yield candidate(f"swing{i:02d}", origin, z)


def refine(first, second):
    delta = np.log(second / first)
    moves = [(x, 0., 0., 0.) for x in (.22, .34, .46, .58, .70)]
    moves += [(.4, .05, -.05, -.08), (.55, .08, -.06, .05),
              (.35, .04, .06, -.10), (.45, -.04, -.06, .10)]
    for i, (blend, speed, wheel, hip) in enumerate(moves, 1):
        center = first * np.exp(blend * delta)
        z = np.zeros(len(AXES))
        z[[1, 6, 7]] = [speed, wheel, hip]
        yield candidate(f"trim{i:02d}", center, z)


def assess(row, base):
    result, reference = row["cases"], base["cases"]
    if len(result) != len(reference):
        return {"pass": False, "failures": ["missing_cases"], "rank": [99]}
    failures = []
    unstable = sum(not x["finite"] for x in result)
    if unstable:
        failures.append("diverged")
    angles = [x["leg_peak"] for x in result]
    old_angles = [x["leg_peak"] for x in reference]
    max_delta = max(a-b for a,b in zip(angles,old_angles,strict=True))
    peak_ratio = max(angles)/max(old_angles)
    if peak_ratio > .90 or max_delta > .004:
        failures.append("leg_peak")
    speed, stop = [], []
    for a, b in zip(result, reference, strict=True):
        if b["finite"]:
            speed.append(a["speed_rms"] - b["speed_rms"])
            stop.append(a["speed_stop_s"] - b["speed_stop_s"])
            if a["pitch_peak_error"] > b["pitch_peak_error"] + .025 or a["roll_peak"] > b["roll_peak"] + .01:
                if "other_attitude" not in failures:
                    failures.append("other_attitude")
            if a["fallback"] > b["fallback"] + .03:
                if "fallback" not in failures:
                    failures.append("fallback")
    speed_delta = max(speed, default=0.)
    stop_delta = max(stop, default=0.)
    if speed_delta > .035:
        failures.append("tracking")
    if stop_delta > .25:
        failures.append("stop")
    rank = [unstable, len(failures), peak_ratio, max_delta, stop_delta, speed_delta]
    return {"pass": not failures, "failures": failures, "rank": rank,
            "leg_peak_ratio": peak_ratio, "max_leg_peak_rad": max(angles),
            "speed_rms_delta": speed_delta, "stop_delay_delta_s": stop_delta}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--matlab", default="matlab")
    parser.add_argument("--refine", action="store_true")
    parser.add_argument("--stress", action="store_true", help="Check higher angle costs on the reversal challenge")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if not (out/"src").exists():
        files.prepare(out)
    verification = files.read_json(files.HERE/"build/brake01_test/verification.json")
    snapshot = out/"src/Wheel_Leg_Code/Code/Model/tinympc14_data.c"
    assert np.array_equal(np.array(active_cost(snapshot)[0],dtype="f"), np.array(verification["Q_DIAG"],dtype="f"))
    assert np.array_equal(np.array(active_cost(snapshot)[1],dtype="f"), np.array(verification["R_DIAG"],dtype="f"))
    baseline = {"id":"brake01", "Q_DIAG":verification["Q_DIAG"], "R_DIAG":verification["R_DIAG"]}
    if args.stress:
        points = [files.read_json(out/"holdout"/f"{tag}.json")
                  for tag in ("swing02","swing03","swing14")]
        staged = [{k:point[k] for k in ("id","Q_DIAG","R_DIAG")} for point in points]
        first, second = weights(points[0]), weights(points[-1])
        for i, move in enumerate((.25, .50, .75), 1):
            w = first * np.exp(move * np.log(second / first))
            staged.append(candidate(f"stress{i:02d}",w,np.zeros(len(AXES))))
        rows = replay(args,out,"stress",TRAIN+HOLD+CHALLENGE,[baseline,*staged],swing=True)
        ranked = sorted(({"id":r["id"],**assess(r,rows[0])} for r in rows[1:]),
                        key=lambda row:row["rank"])
        files.write_json(out/"stress_results.json",ranked)
        print(ranked,flush=True)
        return
    if args.refine:
        first, second = (files.read_json(out/"holdout"/f"{tag}.json") for tag in ("swing12","swing02"))
        staged = list(refine(weights(first),weights(second)))
        screen, heldout, cases = "trim", "trim_holdout", TRAIN+HOLD
    else:
        staged = list(generate(weights(verification)))
        screen, heldout, cases = "screen", "holdout", TRAIN
    rows = replay(args,out,screen,cases,[baseline,*staged],swing=True)
    ranked = sorted(((row,assess(row,rows[0])) for row in rows[1:]), key=lambda pair:pair[1]["rank"])
    files.write_json(out/f"{screen}_rank.json",[{"id":r["id"],**a} for r,a in ranked])
    selected = ranked[:3] if args.refine else ranked[:4] + [
        min(ranked,key=lambda pair:pair[1]["rank"][2] if len(pair[1]["rank"])>2 else 99)]
    finalists = {row["id"]: {k:row[k] for k in ("id","Q_DIAG","R_DIAG")} for row,_ in selected}
    held = replay(args,out,heldout,TRAIN+HOLD+(CHALLENGE if args.refine else []),
                  [baseline,*finalists.values()],swing=True)
    final = [{"id":row["id"],**assess(row,held[0])} for row in held[1:]]
    final.sort(key=lambda row:row["rank"])
    files.write_json(out/("trim_results.json" if args.refine else "results.json"),
                     {"screened":len(ranked),"holdout":final})
    print(final,flush=True)


if __name__=="__main__":
    main()
