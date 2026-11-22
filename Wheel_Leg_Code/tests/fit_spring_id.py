"""Build a lookup table from protocol 7 or fit a polynomial from legacy protocol 6."""
import argparse
import json
from pathlib import Path
import numpy as np


def prepare(data, phase):
    rows = data[(data[:, 7] == phase)]
    if len(rows) < 100:
        raise ValueError("Insufficient rows for both directions")
    # Both motor counters must advance before consuming another paired measurement.
    keep = np.zeros(len(rows), dtype=bool)
    last = (-1, -1)
    for i, row in enumerate(rows):
        pair = tuple(row[15:17])
        if pair[0] != last[0] and pair[1] != last[1]:
            keep[i] = True
            last = pair
    rows = rows[keep]
    times = rows[:, 9] * 0.001
    lengths = rows[:, 2]
    # Estimate mean motion over 0.5 s, since exported instantaneous speed may round to zero.
    before = np.searchsorted(times, times - 0.25)
    after = np.minimum(np.searchsorted(times, times + 0.25), len(times) - 1)
    duration = times[after] - times[before]
    speed = (lengths[after] - lengths[before]) / np.maximum(duration, 1e-6)
    sign = -1 if phase == 1 else 1
    valid = (rows[:, 8] == 1) & (duration >= 0.4)
    valid &= (sign * speed > 0.0005) & (sign * speed < 0.0015)
    valid &= np.isfinite(rows).all(axis=1)
    return rows[valid]


def fit(data, degree):
    if data.shape[1] < 18 or not np.all(data[:, 17] == 6):
        raise ValueError("Expected protocol 6 CSV with at least 18 channels")
    side = "right"
    if data.shape[1] >= 27:
        sides = np.unique(data[:, 26])
        if len(sides) != 1 or sides[0] not in (0, 1):
            raise ValueError("Expected a single acquisition side, I26=0 left or 1 right")
        side = "left" if sides[0] == 0 else "right"
    candidates = []
    for run in np.unique(data[:, 10]):
        rows = data[data[:, 10] == run]
        try:
            compression, extension = prepare(rows, 1), prepare(rows, 3)
        except ValueError:
            continue
        bins = []
        for mm in np.arange(175, 340, 5):
            legs = []
            for samples in (compression, extension):
                subset = samples[(samples[:, 2] >= mm / 1000) &
                                 (samples[:, 2] < (mm + 5) / 1000)]
                if len(subset) < 15:
                    break
                legs.append(subset)
            if len(legs) == 2:
                inward, outward = legs
                inward_force = np.median(inward[:, 6])
                outward_force = np.median(outward[:, 6])
                bins.append([
                    (np.median(inward[:, 2]) + np.median(outward[:, 2])) / 2,
                    (inward_force + outward_force) / 2,
                    (inward_force - outward_force) / 2,
                    len(inward), len(outward),
                ])
        if len(bins) >= max(8, degree + 3):
            candidates.append((run, np.array(bins)))
    if not candidates:
        raise ValueError("Insufficient overlapping slow-motion samples. Keep the original CSV and check tracking, saturation and both directions.")
    run, bins = max(candidates, key=lambda item: len(item[1]))
    low, high = bins[:, 0].min(), bins[:, 0].max()
    if high - low < 0.08:
        raise ValueError("Measured overlap is under 80 mm. Full-stroke fit is unsupported.")
    center, scale = (low + high) / 2, (high - low) / 2
    x = (bins[:, 0] - center) / scale
    spring = np.polynomial.polynomial.polyfit(x, bins[:, 1], degree)
    friction = np.polynomial.polynomial.polyfit(x, bins[:, 2], degree)
    residual = np.polynomial.polynomial.polyval(x, spring) - bins[:, 1]
    return bins, dict(leg=side, run=int(run), degree=degree, length_min_m=low, length_max_m=high,
                      center_m=center, scale_m=scale, spring_coefficients=spring.tolist(),
                      friction_coefficients=friction.tolist(),
                      bin_rmse_N=float(np.sqrt(np.mean(residual ** 2))),
                      definition="x=(length_m-center_m)/scale_m; coefficients ascending; spring is passive extension-positive force; friction is (compression-extension)/2")



def make_table(data, output):
    if data.shape[1] < 31 or not np.all(data[:, 17] == 7):
        raise ValueError("Expected 31-channel step-table protocol 7")
    sides = np.unique(data[:, 26])
    if len(sides) != 1 or sides[0] not in (0, 1):
        raise ValueError("Mixed acquisition sides")
    candidates = []
    for run in np.unique(data[:, 10]):
        rows = data[(data[:, 10] == run) & (data[:, 8] == 1)]
        table = []
        for goal in np.unique(np.round(rows[:, 27], 6)):
            directions = []
            for phase in (2, 4):
                group = rows[(rows[:, 7] == phase) & (np.abs(rows[:, 27] - goal) < 1e-6)]
                group = group[np.isfinite(group).all(axis=1)]
                keep, last = [], (-1, -1)
                for row in group:
                    pair = tuple(row[15:17])
                    if pair[0] != last[0] and pair[1] != last[1]:
                        keep.append(row)
                        last = pair
                if len(keep) < 30:
                    break
                group = np.array(keep)
                # A resumed node starts a new settling interval. Use its latest contiguous block.
                cuts = np.flatnonzero((np.diff(group[:, 9]) > 200) |
                                      (np.diff(group[:, 29]) < 0)) + 1
                group = np.split(group, cuts)[-1]
                if len(group) < 30 or np.ptp(group[:, 9]) < 400:
                    break
                directions.append(group)
            if len(directions) != 2:
                continue
            inward, outward = directions
            inward_force, outward_force = np.mean(inward[:, 6]), np.mean(outward[:, 6])
            table.append([goal, (np.mean(inward[:, 2]) + np.mean(outward[:, 2])) / 2,
                          inward_force, outward_force, (inward_force + outward_force) / 2,
                          (inward_force - outward_force) / 2, np.std(inward[:, 6]),
                          np.std(outward[:, 6]), len(inward), len(outward)])
        if len(table) >= 2:
            candidates.append((run, np.array(table)))
    if not candidates:
        raise ValueError("Need at least two nodes with valid settled samples in both directions")
    run, table = max(candidates, key=lambda item: len(item[1]))
    lengths = table[:, 1]
    if not np.all(np.diff(lengths) > 0):
        raise ValueError("Measured node lengths are not strictly increasing")
    result = dict(leg="left" if sides[0] == 0 else "right", run=int(run),
                  model="piecewise linear lookup of settled bidirectional means",
                  length_min_m=float(lengths[0]), length_max_m=float(lengths[-1]),
                  node_count=len(table), nominal_step_m=0.010,
                  nodes=table.tolist(),
                  definition="columns match lookup_table.csv; directional half-gap includes static friction and hysteresis")
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output / "lookup_table.csv", table, delimiter=",", comments="",
               header="goal_m,length_m,compression_N,extension_N,mean_N,half_gap_N,compression_std_N,extension_std_N,compression_count,extension_count")
    (output / "fit.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    nodes = ("," + chr(10)).join(f"    {{{row[1]:.9e}f, {row[4]:.9e}f}}" for row in table)
    code = f"""// Measured {result['leg']} leg. Static, settled bidirectional mean, extension-positive N.
// Query by measured leg length in m. Clamp to the measured coverage.
static const float spring_table[][2] = {{
{nodes}
}};
float Spring_Identified(float length_m)
{{
    const unsigned int count = sizeof(spring_table) / sizeof(spring_table[0]);
    if (length_m <= spring_table[0][0]) return spring_table[0][1];
    for (unsigned int i = 1u; i < count; i++)
    {{
        if (length_m <= spring_table[i][0])
        {{
            const float fraction = (length_m - spring_table[i-1u][0]) /
                                   (spring_table[i][0] - spring_table[i-1u][0]);
            return spring_table[i-1u][1] +
                   fraction * (spring_table[i][1] - spring_table[i-1u][1]);
        }}
    }}
    return spring_table[count-1u][1];
}}
"""
    (output / "spring_identified.c").write_text(code, encoding="utf-8")
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, default=Path("build/spring_id_fit"))
    parser.add_argument("--degree", type=int, choices=(1, 2, 3), default=3)
    args = parser.parse_args()
    data = np.atleast_2d(np.genfromtxt(args.csv, delimiter=",", skip_header=1))
    if np.all(data[:, 17] == 7):
        result = make_table(data, args.output)
        result["source_csv"] = str(args.csv.resolve())
        (args.output / "fit.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
        return
    bins, result = fit(data, args.degree)
    result["source_csv"] = str(args.csv.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "fit.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    np.savetxt(args.output / "length_bins.csv", bins, delimiter=",",
               header="length_m,spring_N,friction_N,compression_samples,extension_samples", comments="")
    coefficients = result["spring_coefficients"]
    polynomial = f"{coefficients[-1]:.9e}f"
    for coefficient in reversed(coefficients[:-1]):
        polynomial = f"({polynomial} * x + {coefficient:.9e}f)"
    code = f"""// Measured {result["leg"]} leg, compression/extension averaged; verify opposite-leg agreement before sharing.
// Outside measured coverage, use the nearest measured endpoint.
float Spring_Identified(float length_m)
{{
    if (length_m < {result['length_min_m']:.9f}f) length_m = {result['length_min_m']:.9f}f;
    if (length_m > {result['length_max_m']:.9f}f) length_m = {result['length_max_m']:.9f}f;
    const float x = (length_m - {result['center_m']:.9f}f) / {result['scale_m']:.9f}f;
    return {polynomial};
}}
"""
    (args.output / "spring_identified.c").write_text(code, encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
