"""Frozen-current-baseline, grouped log-action PPO with independent final scenarios."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import argparse
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are
import gymnasium as gym
from stable_baselines3 import PPO
import torch
import run_pso as files
from stability_metrics import summarize, ANGLE_LIMITS, TOL

OUT = Path(os.environ.get('TINY_STABILITY_OUT', str(files.HERE/'build/ppo_stability_20261119'))).resolve()
AXES = ((1,), (4,6), (5,7), (8,), (9,), (14,15), (16,17))


def scenarios(split):
    # height, damping, velocity, delay seconds, added load kg, command kind
    tables = {
        'train': [(.17,.5,-1.9,.004,0.,0),(.18,2.1,-1.8,.012,.15,1),
                  (.19,5.5,1.5,.024,.25,2),(.25,.6,.75,.008,.1,0),
                  (.26,3.2,-.95,.016,.2,2),(.18,2.,0.,.012,.15,3)],
        'selection': [(.175,1.2,-1.85,.010,.1,0),(.185,3.4,-1.7,.018,.2,1),
                      (.195,5.,1.45,.020,.25,2),(.24,1.3,-.85,.010,.15,0),
                      (.255,2.5,.9,.014,.2,2),(.19,.75,0.,.008,.1,3)],
        'independent': [(.175,0.,-2.,.006,.05,1),(.185,5.23,-1.9,.022,.23,1),
                        (.195,7.5,1.6,.024,.3,2),(.17,2.07,-1.95,.014,.18,0),
                        (.265,4.5,-1.,.020,.25,2),(.24,.51,.8,.006,.12,0),
                        (.255,2.08,0.,.018,.22,3)]}
    return [dict(id=f'{split}_{i:02d}',height=h,damping=d,velocity=v,delay_s=lag,load=load,kind=kind)
            for i,(h,d,v,lag,load,kind) in enumerate(tables[split])]


def command(t, case):
    v, kind = case['velocity'],case['kind']
    if t < .2 or t >= 2.2: return 0.
    if kind == 1 and 1.9 <= t < 2.04: return -.35*v
    if kind == 1 and 2.04 <= t: return 0.
    if kind == 2 and t >= 1.2: return -v
    return v


class Suite:
    def __init__(self,data,cases):
        self.data,self.cases=data,cases
        self.models={round(m['height'],5):m for m in data['models']}

    def evaluate(self,w,save=None):
        results=[]
        gains={}
        for case in self.cases:
            m=self.models[round(case['height'],5)]
            a,b=np.array(m['A']),np.array(m['B'])
            a,b=a@a,a@b+b
            dt=.004
            h=case['height']
            if h not in gains:
                q,r=np.diag(w[:14]),np.diag(w[14:])
                p=solve_discrete_are(a,b,q,r)
                gains[h]=np.linalg.solve(r+b.T@p@b,b.T@p@a)
            gain=gains[h]
            u0=np.array(m['u0'])
            lo,hi=np.array(self.data['umin'])-u0,np.array(self.data['umax'])-u0
            x=np.zeros(14)
            queue=np.zeros((max(1,round(case['delay_s']/dt)),6))
            locked=case['kind']==3
            origin=0.; weight=.5 if locked else 0.
            xx=[]; tt=[]; rr=[]; uu=[]; ww=[]
            finite=True
            for k,t in enumerate(np.arange(0,5.,dt)):
                ref=command(t,case)
                if k==round((2.8 if case['kind']==3 else .65)/dt):
                    x[9]+=.12; x[11]+=.04
                    if case['kind']==3: x[1]+=.15
                legs_ok=np.max(abs(x[[4,6]]))<=.15
                if ref != 0: locked=False;weight=0.
                elif not locked and abs(x[1])<=.1 and legs_ok:
                    locked=True;origin=x[0];weight=.5
                if not legs_ok: weight=0.
                elif locked: weight=min(.5,weight+dt*.5/.2)
                obs=x.copy();obs[0]=weight*(x[0]-origin);obs[1]-=ref
                u=np.clip(-gain@obs,lo,hi)
                applied=queue[k%len(queue)].copy();queue[k%len(queue)]=u
                applied[2:4]-=case['damping']*(x[[5,7]]-x[9])
                # Frozen linear screening only; load is tested with nonlinear dynamics later.
                x=a@x+b@applied
                tt.append(t+dt);xx.append(x.copy());rr.append(ref);uu.append(u+u0);ww.append(weight)
                if not np.isfinite(x).all() or max(abs(x[[4,6,8,10]]))>.6:
                    finite=False;break
            metrics=summarize(tt,xx,rr,uu,np.zeros(len(tt)),ww,finite)
            metrics['case']=case
            metrics['screen_limitations']=['linear plant','no MPC slew or fallback','load deferred to nonlinear verification']
            results.append(metrics)
            if save:
                save.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(save/(case['id']+'.npz'),t=tt,x=xx,reference=rr,torque=uu,weight=ww)
        return results


def score(rows):
    violation=max(max(0.,-r['worst_margin_rad']) for r in rows)
    failed=sum(not r['passed'] for r in rows)
    posture=max(1-r['worst_margin_rad']/ANGLE_LIMITS[0] for r in rows)
    other=np.mean([r['speed_rms']+.3*r['wheel_saturation']+r['late_speed_rms'] for r in rows])
    # Multiplication makes hard posture/oscillation failures expensive; final selection hard rejects.
    return float((1+max(0,posture)+.15*other)*(1+20*failed+100*violation)**2)


class Tuning(gym.Env):
    observation_space=gym.spaces.Box(-1,1,(1,),np.float32)
    action_space=gym.spaces.Box(-1,1,(len(AXES),),np.float32)
    def __init__(self,suite,base,seed,radius):
        self.suite,self.base,self.seed_value,self.radius=suite,base,seed,radius
        self.trials=[]
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed);return np.zeros(1,np.float32),{}
    def weights(self,action):
        w=self.base.copy()
        for group,z in zip(AXES,action):w[list(group)]*=np.exp(np.log(self.radius)*z)
        return w
    def step(self,action):
        w=self.weights(action)
        row=dict(id=f's{self.seed_value}_{len(self.trials):04d}',seed=self.seed_value,
                 Q_DIAG=w[:14].tolist(),R_DIAG=w[14:].tolist(),action=action.tolist())
        try:
            row['cases']=self.suite.evaluate(w);row['score']=score(row['cases'])
            row['failures']={r['case']['id']:r['failures'] for r in row['cases'] if r['failures']}
        except (ValueError,np.linalg.LinAlgError) as e:
            row.update(score=1e9,failures={'solver':str(e)},cases=[])
        self.trials.append(row)
        return np.zeros(1,np.float32),-float(np.log1p(row['score'])),True,False,{}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps',type=int,default=192)
    parser.add_argument('--baseline-only',action='store_true')
    parser.add_argument('--sensitivity',action='store_true')
    args=parser.parse_args()
    data=files.read_json(OUT/'model.json');base=np.r_[data['q'],data['r']]
    config=dict(angle_limits_rad=ANGLE_LIMITS.tolist(),angle_tolerance_rad=TOL,
                failure_boundaries=dict(firmware_pitch_rad=1.,firmware_legs_rad=.8,firmware_dwell_s=.2,simulation_angle_rad=.6),
                rationale='pitch 10deg versus stable 4.58deg; legs 30deg versus stable 25.21deg and driving 28.07deg; NOT fall limits',
                stop=dict(speed_m_s=.06,dwell_s=.20,lock_m_s=.10),seeds=[19,59,117],axes=AXES,
                oscillation=dict(final_window_s=.6,comparison_window_s=.6,peak_to_peak_deg=3.,decay_ratio=.85,min_rate_sign_changes=2),
                firmware_contract=dict(control_ms=1,predictor_ms=2,solve_ms=4,s_curve=False,park_leg_rad=.15,
                                       park_weight=.5,park_recovery_ramp_ms=200,wheel_limit_nm=16384/3326.29,hip_limit_nm=40.),
                nonlinear_boundary_height_m=[.12,.42],
                damping_basis='VOFA model-dependent fits .51-2.08 and 2.07-5.23; stress 0 to 7.5',
                delay_basis='4-24ms uncertainty, not measured 0-1ms feedback age',
                load_basis='0-.30kg engineering uncertainty, not an identified load',
                scenarios={s:scenarios(s) for s in ('train','selection','independent')})
    files.write_json(OUT/'experiment_config.json',config)
    suites={s:Suite(data,scenarios(s)) for s in ('train','selection','independent')}
    if not (OUT/'baseline_fast.json').exists():
        baseline={s:suite.evaluate(base,OUT/'traces/baseline_fast'/s) for s,suite in suites.items()}
        files.write_json(OUT/'baseline_fast.json',baseline)
        print('baseline',[(s,sum(r['passed'] for r in rows),len(rows)) for s,rows in baseline.items()],flush=True)
    if args.baseline_only:return
    if args.sensitivity:
        suite=suites['train']; nominal=score(suite.evaluate(base));rows=[]
        for name,axes in [('position',(0,)),('yaw',(2,)),('yaw_rate',(3,)),('roll',(10,)),
                          ('roll_rate',(11,)),('height',(12,)),('height_rate',(13,)),('force',(18,19))]:
            for factor in (.8,1.25):
                w=base.copy();w[list(axes)]*=factor;metrics=suite.evaluate(w)
                rows.append(dict(group=name,axes=axes,multiplier=factor,score=score(metrics),
                                 failures=sum(not m['passed'] for m in metrics),Q_DIAG=w[:14].tolist(),R_DIAG=w[14:].tolist()))
        files.write_json(OUT/'remaining_axes_sensitivity.json',dict(baseline_score=nominal,rows=rows,
            decision='Keep seven groups: best remaining-axis improvement about 5%, no failed-case count reduction.'))
        return
    torch.set_num_threads(1)
    selected=[]
    for seed in (19,59,117):
        path=OUT/f'ppo_seed{seed}.json'
        if path.exists():trials=files.read_json(path)
        else:
            env=Tuning(suites['train'],base,seed,2.25)
            agent=PPO('MlpPolicy',env,n_steps=32,batch_size=32,n_epochs=6,learning_rate=.002,
                      gamma=0.,seed=seed,device='cpu',policy_kwargs=dict(net_arch=[32,32],log_std_init=-1.2))
            agent.learn(total_timesteps=args.steps)
            agent.save(OUT/f'policy_{seed}')
            trials=env.trials;files.write_json(path,trials)
        selected.extend(sorted(trials,key=lambda r:r['score'])[:3])
        print('seed',seed,'trials',len(trials),'best',min(r['score'] for r in trials),flush=True)
    for row in selected:
        w=np.r_[row['Q_DIAG'],row['R_DIAG']]
        row['selection']=suites['selection'].evaluate(w)
        row['selection_score']=score(row['selection'])
    # For feasible candidates posture margin is lexicographically primary.
    selected.sort(key=lambda r:(0,-min(m['worst_margin_rad'] for m in r['selection']),r['selection_score'])
                  if all(m['passed'] for m in r['selection']) else (1,r['selection_score'],0))
    files.write_json(OUT/'selected_fast.json',selected)
    # Freeze finalists before evaluating any independent scenarios.
    finalists=[{k:r[k] for k in ('id','seed','Q_DIAG','R_DIAG')} for r in selected[:3]]
    files.write_json(OUT/'finalists.json',finalists)
    print('frozen finalists',[r['id'] for r in finalists],flush=True)


if __name__=='__main__':main()
