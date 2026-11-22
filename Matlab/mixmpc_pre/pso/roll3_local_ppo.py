"""Local nonlinear PPO on roll3_10. Run from the repo root with python -B."""
import csv
import shutil
import numpy as np
import smooth_accel_ppo as s
c,f=s.core,s.f
prior=f.HERE/'build/ppo_roll3_20261119'
out=f.HERE/'build/ppo_roll3_local_20261119'
out.mkdir(exist_ok=True)
s.OUT=c.OUT=out
for name in ('src','helper'):
 if not (out/name).exists():shutil.copytree(prior/name,out/name)
helper=out/'helper'
base=next(p for p in f.read_json(prior/'finalists.json') if p['id']=='roll3_10')
old=[dict(sorted(r['case'].items())) for r in f.read_json(prior/'validation/enriched_summary.json')]
by={r['id']:r for r in old}
focus=[by['direct3'],by['mid3']]
dev=[by[n] for n in ('reverse3','biased3','opposite_bias','independent_02')]
held=[dict(by[n],id='local_hold_'+str(i),height=h,damping=d,delay_s=lag,bias=b,load=l) for i,(n,h,d,lag,b,l) in enumerate([('direct3',.187,4.2,.019,0,.23),('mid3',.245,2.7,.015,-3,.17),('reverse3',.197,5.6,.021,2,.26),('biased_park',.235,3.6,.017,-7,.21)])]
c.FOCUS=focus
original=c.point
c.point=lambda name,w:original(name.replace('smooth_s','r3local_s'),w)
basic=s.metrics
def measure(folder):
 rows=basic(folder)
 for r in rows:
    case=r['case'];raw=f.read_json(folder/(r['candidate']+'__'+case['id']+'.json'))['metrics']['raw']
    t=np.array(raw['t']);x=np.array(raw['x']);mask=(t>=1.7)&(t<2.2)
    r['roll_deg']=float(np.rad2deg(abs(x[t>t[-1]-.5,10]).mean()))
    r['hold_rms']=float(np.sqrt(np.mean((x[mask,1]-case['velocity'])**2))) if mask.any() else 10.
    if case['bias'] and r['roll_deg']>1:r['failures'].append('roll_residual')
    if abs(case['velocity'])==3 and r['hold_rms']>.35:r['failures'].append('3mps_tracking')
    b=s.BASE_METRICS.get(case['id'])
    if b and case['bias'] and r['roll_deg']>b['roll_deg']+.01:r['failures'].append('roll_regression')
    if b and r['smooth']['hip_delta_peak_nm']>b['smooth']['hip_delta_peak_nm']*1.02+.01:r['failures'].append('hip_step_regression')
    r['passed']=not r['failures']
 f.write_json(folder/'enriched_summary.json',rows);return rows
c.read_metrics=measure
def loss(rows):
 vals=[]
 for r in rows:
    b=s.BASE_METRICS[r['case']['id']]
    ratios=[r['angles'][a]['peak_rad']/np.deg2rad(limit) for a,limit in [('pitch',10),('left_leg',30),('right_leg',30)]]+[r['height_excursion_m']/.062]
    penalty=max(ratios)+sum(max(0,v-1) for v in ratios)
    vals.append((1+penalty**2+.3*r['smooth']['hip_delta_peak_nm']/max(.01,b['smooth']['hip_delta_peak_nm'])+.3*r['hold_rms']+.1*r['roll_deg'])*(1+10*(not r['passed']))**2)
 return float(np.mean(vals))
s.score=c.score=loss
def main():
 before=s.protected();f.write_json(out/'before.json',before)
 f.write_json(out/'experiment_config.json',dict(baseline=base,training=focus,selection=dev,regression=old,independent=held,steps=32,seeds=[19,59,117],radius=1.3,axes=c.LABELS,angle_deg=[10,30,30],tol_rad=.002,height_m=.062,roll_deg=1,hold_rms=.35,hip_guard='baseline *1.02 +0.01 Nm',roll_guard='baseline +0.01 deg',stop_guard='previous +0.1s and no new rebound'))
 base_rows=c.batch('baseline',[base],old+held,helper);s.BASE_METRICS.update({r['case']['id']:r for r in base_rows})
 c.search(helper,base,32);pts=f.read_json(out/'shortlist.json')
 selected=c.batch('selection',pts,dev,helper)
 pts.sort(key=lambda p:s.rank([r for r in selected if r['candidate']==p['id']]))
 pts=pts[:3];f.write_json(out/'finalists.json',pts)
 rows=c.batch('validation',pts,old+held,helper)
 s.report(base,base_rows,pts,rows)
 path=out/'REPORT.md';path.write_text(path.read_text(encoding='utf8').replace('smooth_accel_ppo.py','roll3_local_ppo.py').replace('Acceleration / smoothness PPO','Local 3 m/s PPO on roll3_10'),encoding='utf8')
if __name__=='__main__':
 f.write_json(out/'protected_sha256.json',s.protected())
 main()

