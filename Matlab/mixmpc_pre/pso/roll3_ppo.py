"""Discrete PPO over measured nonlinear responses for roll bias and 3 m/s steps."""
import shutil,itertools
import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO
import smooth_accel_ppo as s
c=s.core;f=s.f
out=f.HERE/'build/ppo_roll3_20261119';out.mkdir(exist_ok=True)
before=s.protected();f.write_json(out/'live_before.json',before)
prior=f.HERE/'build/ppo_smooth_reversal_20261119'
s.OUT=c.OUT=out
if not (out/'src').exists():shutil.copytree(prior/'src',out/'src')
helper=out/'helper'
if not helper.exists():shutil.copytree(next((prior/'verify_stability_1ms').rglob('stability_verify.m')).parent,helper)
p=helper/'stability_verify.m';src=p.read_text(encoding='utf8')
if 'scene.bias' not in src:
 src=src.replace('config.COMMAND_KIND=scene.kind;', 'config.COMMAND_KIND=scene.kind; config.BIAS=scene.bias;').replace('u=[motor_delayed;', 'if t(k)>=.4, turn=turn+p.BIAS; end\n    u=[motor_delayed;')
 p.write_text(src,encoding='utf8')
base=f.read_json(f.HERE/'build/ppo_smooth_accel_20261119/closest_candidate.json')
w=np.r_[base['Q_DIAG'],base['R_DIAG']]
base=c.point('installed_smooth_s19_001',w)
def case(name,h,v,b,d,lag,kind=0):return dict(id=name,height=h,velocity=v,bias=b,damping=d,delay_s=lag,load=.2,kind=kind)
train=[case('biased_park',.19,0,6,2,.012),case('direct3',.18,3,0,3,.016)]
held=[case('opposite_bias',.255,0,-8,4,.018),case('mid3',.25,3,0,4,.018),case('reverse3',.19,-3,0,5,.022),case('biased3',.185,3,4,2.5,.014)]
reg=[dict(r['case'],bias=0) for r in f.read_json(prior/'baseline/enriched_summary.json')]
points=[]
for i,(qr,force,speed) in enumerate(itertools.product([2.,6.,12.],[.65,1.],[.65,1.])):
 a=w.copy();a[10]*=qr;a[11]*=qr**.5;a[18:]*=force;a[1]*=speed;points.append(c.point(f'roll3_{i:02}',a))
rev=f.read_json(prior/'finalists.json')[0]
for multiplier in [2.,6.,12.]:
 a=np.r_[rev['Q_DIAG'],rev['R_DIAG']];a[10]*=multiplier;a[11]*=multiplier**.5;points.append(c.point(f'reversal_roll_{int(multiplier)}',a))
f.write_json(out/'experiment_config.json',dict(train=train,held=held,points=points,roll_residual_deg=1.,speed_hold_rms=.35,bias_unit='opposed N per leg injected in plant from 0.4s onward; model stress assumption',seeds=[19,59,117]))
def assess(stage,ps,cases):
 rows=c.batch(stage,ps,[dict(sorted(z.items())) for z in cases],helper)
 for r in rows:
    z=f.read_json(out/stage/(r['candidate']+'__'+r['case']['id']+'.json'))['metrics']['raw'];x=np.array(z['x']);t=np.array(z['t']);u=np.array(z['torque'])
    r['roll_res_deg']=np.rad2deg(np.mean(abs(x[t>t[-1]-.5,10])));r['force_diff_N']=np.mean((u[:,4]-u[:,5])[t>t[-1]-.5])
    mask=(t>=1.7)&(t<2.2);r['hold_rms']=float(np.sqrt(np.mean((x[mask,1]-r['case']['velocity'])**2))) if mask.any() else 10.
    if r['case']['bias'] and r['roll_res_deg']>1:r['failures'].append('roll_residual')
    if abs(r['case']['velocity'])==3 and r['hold_rms']>.35:r['failures'].append('3mps_tracking')
    r['passed']=not r['failures']
 f.write_json(out/stage/'enriched_summary.json',rows);return rows
base_rows=assess('baseline',[base],train+held+reg)
s.BASE_METRICS.update({r['case']['id']:r for r in base_rows})
rows=assess('screen',points,train)
def loss(rs):return (1+10*sum(not r['passed'] for r in rs))*(1+sum(r['roll_res_deg']+max(0,-r['worst_margin_rad'])*20+r['height_excursion_m']*10+r['hold_rms']+.02*r['smooth']['hip_delta_peak_nm'] for r in rs))
scores=[loss([r for r in rows if r['candidate']==p['id']]) for p in points]
class E(gym.Env):
 observation_space=gym.spaces.Box(0,1,(1,),np.float32);action_space=gym.spaces.Discrete(len(points))
 def reset(self,*,seed=None,options=None):super().reset(seed=seed);return np.zeros(1,np.float32),{}
 def step(self,a):return np.zeros(1,np.float32),-np.log1p(scores[a]),True,False,{}
ids=[]
for seed in (19,59,117):
 model=PPO('MlpPolicy',E(),seed=seed,n_steps=32,batch_size=32,gamma=0,learning_rate=.002,device='cpu');model.learn(1024);model.save(out/f'policy_{seed}');ids.append(int(model.predict(np.zeros(1,np.float32),deterministic=True)[0].item()))
chosen=[points[i] for i in dict.fromkeys(ids+[int(np.argmin(scores))])];f.write_json(out/'finalists.json',chosen)
rs=assess('validation',chosen,held+reg+train)
f.write_json(out/'result.json',dict(selections=ids,score=scores,passes={p['id']:sum(r['passed'] for r in rs if r['candidate']==p['id']) for p in chosen},total=len(train+held+reg)))

assert before==s.protected()
f.write_json(out/'live_after.json',s.protected())
import csv
with (out/'comparison.csv').open('w',newline='',encoding='utf-8-sig') as stream:
 writer=csv.writer(stream);writer.writerow(['candidate','case','pass','failures','pitch_deg','left_deg','right_deg','height_mm','roll_residual_deg','left_minus_right_N','hold_speed_rms','wheel_sat','hip_sat','fallback','hip_step_Nm','stops'])
 for r in base_rows+rs:
    writer.writerow([r['candidate'],r['case']['id'],r['passed'],','.join(r['failures']),*[np.rad2deg(r['angles'][axis]['peak_rad']) for axis in ('pitch','left_leg','right_leg')],r['height_excursion_m']*1000,r['roll_res_deg'],r['force_diff_N'],r['hold_rms'],r['wheel_saturation'],r['hip_saturation'],r['fallback'],r['smooth']['hip_delta_peak_nm'],r['stops']])


