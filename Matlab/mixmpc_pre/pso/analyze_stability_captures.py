"""Archive-aware capture evidence; no capture is claimed as independent model validation."""
import numpy as np
from fit_hip_vofa import read_capture
from run_stability_ppo import OUT, files


def main():
    results=[]
    for name in ('brake01','swing14'):
        path=OUT/'captures'/f'{name}_capture.csv'
        a,ages=read_capture(path)
        t=(a[:,0]-a[0,0])/1000
        transitions=np.flatnonzero((abs(a[1:,9])<.001)&(abs(a[:-1,9])>=.001))+1
        segments=[]
        for start in transitions:
            end=next((j for j in range(start+1,len(t)) if abs(a[j,9])>=.001 or a[j,3] not in (0,7)),len(t))
            if end-start<3:continue
            reason='record_end' if end==len(t) else 'new_command' if abs(a[end,9])>=.001 else 'mode_exit'
            stop=None
            for j in range(start,end):
                k=np.searchsorted(t,t[j]+.2)
                if k<end and np.max(np.diff(t[j:k+1]))<=.08 and np.all(abs(a[j:k+1,8])<=.06):
                    stop=j;break
            peaks=[]
            for axis in (10,11,12):
                k=start+int(np.argmax(abs(a[start:end,axis])))
                peaks.append(dict(peak_deg=float(np.rad2deg(abs(a[k,axis]))),time_s=float(t[k])))
            prior=(t>=t[start]-.15)&(t<t[start])
            segments.append(dict(zero_s=float(t[start]),end_s=float(t[end-1]),end_reason=reason,
                stop_s=None if stop is None else float(t[stop]-t[start]),
                rebound=None if stop is None else bool(np.any(abs(a[np.searchsorted(t,t[stop]+.2):end,8])>.06)),
                observation_truncated=True,
                max_gap_s=float(np.max(np.diff(t[start:end]))),
                has_gap_over_80ms=bool(np.any(np.diff(t[start:end])>.08)),
                pitch_left_right=peaks,
                prerelease_height_m=np.median(a[prior,45:47],axis=0).tolist(),
                prerelease_speed_m_s=float(np.median(a[prior,8])),
                prerelease_yaw_rate_rad_s=float(np.median(a[prior,48])),
                final_park_weight=float(a[end-1,6]),
                position_drift_m=float(a[end-1,4]-a[start,4])))
        results.append(dict(capture=name,sha256=files.digest(path),layout=5402,valid_frames=len(a),
            provenance='existing named archive; layout identifies channels, not unique firmware binary; no new capture firmware claim',
            feedback_age_ms=[float(ages.min()),float(ages.max())],
            gaps_over_80ms=int(np.sum(np.diff(t)>.08)),maximum_gap_s=float(np.max(np.diff(t))),segments=segments))
    files.write_json(OUT/'capture_analysis.json',results)
    print('saved capture_analysis.json')


if __name__=='__main__':main()
