"""Analyze the 13-channel velocity-braking VOFA export; write metrics and a plot."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, default=Path('diagnostics/velocity_20261102'))
    args = parser.parse_args()
    x = np.genfromtxt(args.csv, delimiter=',', skip_header=1)
    if x.ndim != 2 or x.shape[1] != 13 or not np.isfinite(x).all():
        parser.error('Expected finite I0..I12 velocity diagnostic channels')
    t = x[:, 12] / 1000.0
    if np.any(np.diff(t) <= 0):
        parser.error('Timestamps must increase; split captures at timestamp resets')
    args.output.mkdir(parents=True, exist_ok=True)
    # Keep short RC release bounces separate; do not attribute later commands to a release.
    released = (np.abs(x[:, 1]) < 20) & (x[:, 2] == 0)
    edges = np.diff(np.r_[False, released, False].astype(int))
    windows = []
    for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        if start == 0 or t[end - 1] - t[start] < 0.3:
            continue
        v = x[start:end, 4]
        sign = np.sign(v[0])
        slow = np.flatnonzero(np.abs(v) <= 0.1)
        crossings = np.flatnonzero(v * sign <= 0)
        windows.append(dict(start_s=float(t[start]), duration_s=float(t[end - 1] - t[start]),
                            initial_speed_mps=float(v[0]),
                            low_speed_delay_s=float(t[start + slow[0]] - t[start]) if len(slow) else None,
                            zero_cross_delay_s=float(t[start + crossings[0]] - t[start]) if len(crossings) else None,
                            reverse_peak_mps=float(max(0, -np.min(sign * v))),
                            position_peak_m=float(np.max(np.abs(x[start:end, 5]))),
                            pitch_peak_deg=float(np.max(np.abs(x[start:end, 6])) * 180 / np.pi)))
    metrics = dict(samples=len(x), duration_s=float(t[-1] - t[0]),
                   timestamp_delta_ms_range=[float(np.min(np.diff(x[:, 12]))), float(np.max(np.diff(x[:, 12])))],
                   control_period_window_max_us=float(np.max(x[:, 9])),
                   control_overrun_window_max=float(np.max(x[:, 10])),
                   pitch_peak_deg=float(np.max(np.abs(x[:, 6])) * 180 / np.pi),
                   speed_fusion_rms_difference_mps=float(np.sqrt(np.mean((x[:, 3] - x[:, 4]) ** 2))),
                   wheel_request_at_5Nm_fraction=[float(np.mean(np.abs(x[:, k]) >= 4.999)) for k in (7, 8)],
                   release_windows=windows)
    (args.output / 'metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex='col', layout='constrained')
    for col in (0, 1):
        axes[0, col].plot(t, x[:, 2], label='Speed reference', linewidth=1)
        axes[0, col].plot(t, x[:, 4], label='Fused speed', linewidth=1)
        axes[0, col].plot(t, x[:, 3], label='Kinematic speed', linewidth=0.7, alpha=0.55)
        axes[1, col].plot(t, x[:, 5], label='Position feedback')
        axes[2, col].plot(t, np.rad2deg(x[:, 6]), label='Body pitch')
        axes[3, col].plot(t, x[:, 7], label='Left wheel request', linewidth=0.8)
        axes[3, col].plot(t, x[:, 8], label='Right wheel request', linewidth=0.8)
        for row, unit in enumerate(('m/s', 'm', 'deg', 'Nm')):
            axes[row, col].set_ylabel(unit)
            axes[row, col].grid(alpha=0.25)
            axes[row, col].axhline(0, color='black', linewidth=0.5)
            axes[row, col].legend(loc='best', fontsize=8)
        axes[3, col].set_xlabel('Device timestamp (s)')
    axes[0, 0].set_title('Recorded response (no simulated improvement)')
    axes[0, 1].set_title('Final release: stop followed by reverse motion')
    axes[0, 0].set_xlim(t[0], t[-1])
    if windows:
        start = windows[-1]['start_s']
        axes[0, 1].set_xlim(start - 0.2, t[-1])
        for ax in axes[:, 1]:
            ax.axvline(start, color='red', linestyle='--', linewidth=0.8)
    fig.savefig(args.output / 'response.png', dpi=160)
    plt.close(fig)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
