"""Inspect saturation trajectories and retain boundary-crossing evidence."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from run_stability_ppo import OUT, files


def main():
    best=files.read_json(OUT/'closest_candidate.json')['id']
    folder=OUT/'nonlinear_final_1ms'
    plots=OUT/'plots';plots.mkdir(exist_ok=True)
    diagnostics=[]
    for case in ('train_01','independent_01','independent_02'):
        fig,axes=plt.subplots(5,1,figsize=(11,12),sharex=True)
        for name,color in (('baseline','tab:blue'),(best,'tab:orange')):
            row=files.read_json(folder/f'{name}__{case}.json');raw=row['metrics']['raw']
            t=np.array(raw['t']);x=np.array(raw['x']);u=np.array(raw['torque']);ref=np.array(raw['reference'])
            axes[0].plot(t,x[:,1],color=color,label=name)
            if name=='baseline':axes[0].plot(t,ref,'k--',label='reference')
            axes[1].plot(t,np.rad2deg(x[:,8]),color=color)
            axes[2].plot(t,np.rad2deg(x[:,4]),color=color,label=name+' left')
            axes[2].plot(t,np.rad2deg(x[:,6]),color=color,linestyle=':',label=name+' right')
            axes[3].plot(t,u[:,0],color=color)
            axes[3].plot(t,u[:,1],color=color,linestyle=':')
            axes[4].plot(t,.27+x[:,12],color=color)
            saturated=np.any(abs(u[:,:2])>=16384/3326.29-.01,axis=1)
            tail=t>=t[-1]-.12
            reasons=[]
            if not row['metrics']['finite']:
                if np.max(abs(x[-1,[4,6,8,10]]))>.6:reasons.append('attitude_above_0.6rad')
                if .27+x[-1,12]<.12:reasons.append('height_below_0.12m')
                if .27+x[-1,12]>.42:reasons.append('height_above_0.42m')
                if not np.isfinite(x[-1]).all():reasons.append('nonfinite_state')
            diagnostics.append(dict(candidate=name,case=case,terminated=not row['metrics']['finite'],
                end_s=float(t[-1]),termination_boundaries=reasons,last_height_m=float(.27+x[-1,12]),
                final_120ms_saturated_samples=int(saturated[tail].sum()),final_120ms_samples=int(tail.sum()),
                reverse_saturation_counts=[int(np.count_nonzero(np.diff(np.sign(u[abs(u[:,j])>=16384/3326.29-.01,j])))) for j in (0,1)],
                note='Boundary exit is a simulation/model failure, not a measured physical fall.'))
        for a,bound in ((axes[1],10),(axes[2],30),(axes[3],16384/3326.29)):
            a.axhline(bound,color='r',alpha=.4,linestyle='--');a.axhline(-bound,color='r',alpha=.4,linestyle='--')
        axes[4].axhline(.12,color='r',alpha=.4,linestyle='--');axes[4].axhline(.42,color='r',alpha=.4,linestyle='--')
        for ax,label in zip(axes,('Speed (m/s)','Pitch (deg)','Leg angles (deg)','Wheel torque (Nm)','Height (m)')):
            ax.set_ylabel(label);ax.grid(alpha=.25)
        axes[0].legend();axes[2].legend(fontsize=8);axes[-1].set_xlabel('Time (s)')
        fig.suptitle(case+' | 1 ms execution; traces end at failure boundary')
        fig.tight_layout();fig.savefig(plots/f'{case}.png',dpi=140);plt.close(fig)
    files.write_json(OUT/'saturation_failure_diagnostics.json',diagnostics)
    print('three comparison plots and saturation diagnostics saved')


if __name__=='__main__':main()
