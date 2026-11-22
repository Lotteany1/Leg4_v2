"""Independent posture and zero-reference dwell metrics for the 20261119 trial."""
import numpy as np

ANGLE_LIMITS = np.deg2rad([10., 30., 30.])
TOL = .002


def summarize(t, x, reference, torque, fallback, weight, finite=True):
    t, x, reference = np.asarray(t), np.asarray(x), np.asarray(reference)
    dt = float(np.median(np.diff(t)))
    posture = x[:, [8, 4, 6]]
    rates = x[:, [9, 5, 7]]
    reasons = []
    angles = {}
    zeros = np.flatnonzero((abs(reference[1:]) < 1e-9) & (abs(reference[:-1]) >= 1e-9)) + 1
    for axis, name in enumerate(('pitch', 'left_leg', 'right_leg')):
        value = abs(posture[:, axis])
        over = value > ANGLE_LIMITS[axis] + TOL
        k = int(np.argmax(value))
        releases = []
        for start in zeros:
            end = next((j for j in range(start + 1, len(t)) if abs(reference[j]) >= 1e-9), len(t))
            segment = value[start:end]
            peak = start + int(np.argmax(segment))
            outside = np.flatnonzero(segment > ANGLE_LIMITS[axis] + TOL)
            recovery = 0. if not len(outside) else (None if outside[-1] == len(segment)-1 else float(t[start+outside[-1]+1]-t[start]))
            releases.append(dict(zero_s=float(t[start]), peak_rad=float(value[peak]), peak_time_s=float(t[peak]),
                                 over_duration_s=float(over[start:end].sum()*dt), recovery_s=recovery))
        angles[name] = dict(peak_rad=float(value[k]), peak_time_s=float(t[k]),
                            over_duration_s=float(over.sum()*dt), rate_peak_rad_s=float(abs(rates[:,axis]).max()),
                            recovery_s=(0. if not over.any() else None if over[-1] else float(t[np.flatnonzero(over)[-1]+1])),
                            releases=releases)
        if over.any(): reasons.append(name + '_angle')
    stops = []
    for start in zeros:
        end = next((j for j in range(start+1,len(t)) if abs(reference[j]) >= 1e-9),len(t))
        dwell = int(np.ceil(.20/dt))
        settled = next((j for j in range(start,end-dwell+1) if np.all(abs(x[j:j+dwell,1]) <= .06)),None)
        stops.append(dict(zero_s=float(t[start]), stop_s=None if settled is None else float(t[settled]-t[start]),
                          rebound=bool(settled is not None and np.any(abs(x[settled+dwell:end,1]) > .06)),
                          truncated=end<len(t) or not finite, duration_s=float(t[end-1]-t[start]+dt),
                          drift_m=float(x[end-1,0]-x[start,0]), position_span_m=float(np.ptp(x[start:end,0])),
                          prerelease_speed_m_s=float(x[start-1,1]),
                          final_park_weight=float(weight[end-1])))
    late = t >= t[-1]-.6
    # Reject a persistent non-decaying oscillation only in the final zero-command window.
    oscillation = False
    if len(zeros) and t[-1]-t[zeros[-1]] >= 1.2 and abs(reference[-1]) < 1e-9:
        prev = (t >= t[-1]-1.2) & (t < t[-1]-.6)
        for axis in range(3):
            a, b = posture[late,axis], posture[prev,axis]
            crossings = np.count_nonzero(np.diff(np.sign(rates[late,axis])))
            oscillation |= bool(np.ptp(a) > np.deg2rad(3) and np.ptp(a) >= .85*np.ptp(b) and crossings >= 2)
    if oscillation: reasons.append('sustained_oscillation')
    if not finite or not np.isfinite(x).all(): reasons.append('nonfinite_or_early_termination')
    torque = np.asarray(torque)
    sat = abs(torque[:,:2]) >= 16384/3326.29 - .01
    flips = [int(np.count_nonzero(np.diff(np.sign(torque[sat[:,i],i])))) for i in range(2)]
    return dict(angles=angles, stops=stops, failures=reasons, passed=not reasons,
                worst_margin_rad=float(np.min(ANGLE_LIMITS-abs(posture).max(axis=0))),
                speed_rms=float(np.sqrt(np.mean((x[:,1]-reference)**2))),
                late_speed_rms=float(np.sqrt(np.mean(x[late,1]**2))),
                wheel_saturation=float(np.mean(np.any(sat,axis=1))), reverse_saturation_counts=flips,
                hip_peak_nm=float(abs(torque[:,2:4]).max()), fallback=float(np.mean(fallback)),
                late_rate_rms=np.sqrt(np.mean(rates[late]**2,axis=0)).tolist())
