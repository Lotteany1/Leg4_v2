"""Reversal diagnosis and local PPO around smooth_s19_001, with independent holdouts."""
import shutil
import numpy as np
import smooth_accel_ppo as s
c=s.core
f=s.f
prior=s.OUT
out=f.HERE/'build/ppo_smooth_reversal_20261119'
out.mkdir(exist_ok=True)
s.OUT=c.OUT=c.verification.OUT=out
src=out/'src'
if not src.exists(): shutil.copytree(prior/'src',src)
helper=out/'verify_stability_1ms'
if not helper.exists(): shutil.copytree(prior/'verify_stability_1ms',helper)
helper=next(helper.rglob('stability_verify.m')).parent
center={k:v for k,v in f.read_json(prior/'closest_candidate.json').items() if k in ('id','Q_DIAG','R_DIAG')}
priorrows=[r for r in f.read_json(prior/'validation/enriched_summary.json') if r['candidate']==center['id']]
s.BASE_METRICS.update({r['case']['id']:r for r in priorrows})
old=[r['case'] for r in priorrows]
def get(name):return next(x for x in old if x['id']==name)
focus=[get('independent_02'),get('smooth_holdout_01')]
c.FOCUS=focus
held=[dict(get(n),id=f'reversal_holdout_{i}',height=h,delay_s=d,damping=b,velocity=v,load=l) for i,(n,h,d,b,v,l) in enumerate([
('independent_02',.188,.021,5.9,-1.72,.26),('train_02',.198,.019,4.6,1.39,.22),('smooth_holdout_01',.184,.022,5.8,-1.58,.21),('selection_01',.177,.015,2.8,-1.79,.16),('smooth_select_03',.252,.017,3.7,.94,.19),('smooth_train_00',.243,.011,1.7,-.81,.14)])]
dev=[get(n) for n in ['new_holdout_01','selection_01','smooth_select_03','smooth_train_00']]
f.write_json(out/'experiment_config.json',dict(center=center,train=focus,selection=dev,held=held,seeds=[19,59,117],steps=32,radius=1.3,pitch_deg=10,leg_deg=30,tolerance_rad=.002,height_limit_m=.062,ranking='Physical violations first; reward includes normalized hip step, stop time, tracking and rebound.'))
def loss(rows):
 vals=[]
 for r in rows:
    b=s.BASE_METRICS[r['case']['id']]
    ratio=lambda key:r['smooth'][key]/max(.001,b['smooth'][key])
    stop=lambda x:max([a.get('stop_s') if a.get('stop_s') is not None else 5 for a in x['stops']]+[.05])
    p=r['angles']['pitch']['peak_rad']/np.deg2rad(10);h=r['height_excursion_m']/.062
    vals.append((1+p*p+h*h+.40*ratio('hip_delta_peak_nm')+.40*ratio('hip_slew20_rms_nm_s')+.35*stop(r)/stop(b)+.2*r['speed_rms']/max(.01,b['speed_rms'])+.2*r['smooth']['post_dwell_peak_m_s']/.06)*(1+8*(not r['passed'])+5*max(0,p-1,h-1))**2)
 return float(np.mean(vals))
s.score=c.score=loss
if __name__=='__main__':
 ab=[dict(focus[0],id='delay_4ms',delay_s=.004),dict(focus[0],id='damping_2',damping=2.),dict(focus[0],id='load_zero',load=0.),dict(focus[0],id='no_reverse',kind=0)]
 diag=c.batch('mechanism',[center],focus+ab,helper)
 for r in diag:print(r['case']['id'],r['angles']['pitch']['peak_rad']*180/np.pi,r['height_excursion_m'],flush=True)
 base=c.batch('baseline',[center],old+held,helper)
 s.BASE_METRICS.update({r['case']['id']:r for r in base})
 c.search(helper,center,32)
 pts=f.read_json(out/'shortlist.json')
 rows=c.batch('selection',pts,dev,helper)
 pts.sort(key=lambda p:s.rank([r for r in rows if r['candidate']==p['id']]))
 pts=pts[:3];f.write_json(out/'finalists.json',pts)
 rows=c.batch('validation',pts,old+held,helper)
 # New candidate search protects the deliberately installed C package and original MATLAB.
 backup= f.HERE/'build/smooth_s19_001_c_test'
 assert f.read_json(backup/'verification.json')['c_installed']
 hashes=s.protected()
 for name,sha in hashes.items():
    path=f.ROOT/name
    expected=(backup/'before'/path.name if path.suffix=='.m' else backup/'packages/full'/path.name)
    assert f.digest(expected)==sha
 f.write_json(out/'protected_sha256.json',hashes)
 s.report(center,base,pts,rows)

