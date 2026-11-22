"""Nonlinear local PPO around s117_0185; retain the production baseline and gates."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time
from types import SimpleNamespace
import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO
import torch
import run_pso as files
import verify_stability_ppo as verification
from run_stability_ppo import scenarios
from stability_metrics import ANGLE_LIMITS, TOL
from search_leg_swing import active_cost

PRIOR=Path(os.environ.get('TINY_LOCAL_PITCH_PRIOR',str(files.HERE/'build/ppo_stability_20261119'))).resolve()
OUT=Path(os.environ.get('TINY_LOCAL_PITCH_OUT',str(files.HERE/'build/ppo_local_pitch_20261119'))).resolve()
AXES=((1,),(4,6),(5,7),(8,),(9,),(14,15),(16,17),(12,),(13,),(18,19))
LABELS=('speed','leg_angle','leg_rate','pitch','pitch_rate','wheel_R','hip_R','height','height_rate','force_R')
OLD=sum([scenarios(s) for s in ('train','selection','independent')],[])
FOCUS=[next(c for c in OLD if c['id']==name) for name in ('independent_01','independent_02')]


def fresh_cases():
    values=[(.18,6.,-1.75,.018,.27,1),(.19,6.8,1.52,.022,.28,2),
            (.195,7.2,-1.45,.020,.24,2),(.175,1.4,-1.85,.010,.1,0),
            (.25,3.5,-.9,.018,.22,2),(.265,1.5,.82,.012,.18,0),(.24,4.2,0.,.020,.25,3)]
    return [dict(id=f'new_holdout_{i:02d}',height=h,damping=d,velocity=v,delay_s=lag,load=load,kind=kind)
            for i,(h,d,v,lag,load,kind) in enumerate(values)]


def point(name,w):
    return dict(id=name,Q_DIAG=w[:14].tolist(),R_DIAG=w[14:].tolist())


def prepare():
    OUT.mkdir(exist_ok=True)
    if not (OUT/'src').exists():
        if os.environ.get('TINY_LOCAL_PITCH_SOURCE'):
            frozen=Path(os.environ['TINY_LOCAL_PITCH_SOURCE']).resolve()
            shutil.copytree(frozen,OUT/'src')
            shutil.copy2(OUT/'src/Matlab/mixmpc_pre/tinympc14_config.m',OUT/'baseline.m.txt')
        else:files.prepare(OUT)
    current=active_cost(OUT/'src/Wheel_Leg_Code/Code/Model/tinympc14_data.c')
    base=files.read_json(PRIOR/'baseline.json')
    assert current==[base['Q_DIAG'],base['R_DIAG']], 'Production drift: reconcile new baseline before using old evidence'
    assert (OUT/'baseline.m.txt').read_bytes()==(PRIOR/'baseline.m.txt').read_bytes()
    center=files.read_json(PRIOR/'closest_candidate.json')
    files.write_json(OUT/'baseline.json',base)
    files.write_json(OUT/'center.json',center)
    if not (OUT/'source_sha256.json').exists():
        files.write_json(OUT/'source_sha256.json',{str(p.relative_to(OUT/'src')):files.digest(p) for p in (OUT/'src').rglob('*') if p.is_file()})
    config=files.read_json(PRIOR/'experiment_config.json')
    config.update(center='s117_0185',axes=AXES,axis_labels=LABELS,seeds=[19,59,117],radius=1.75,
                  training_cases=FOCUS,selection_cases=OLD,new_independent_cases=fresh_cases(),
                  provenance='All old independent cases are now development data; new_holdout is untouched until finalists freeze.',
                  added_height_gate=dict(max_tracking_error_m=.06,tolerance_m=.002,
                    rationale='Fixed-height commands: do not accept a large jump solely because it stays below the 0.42m fault boundary; stricter new local-search guard.'),
                  reward='Actual constrained nonlinear 1ms closed-loop, multiplicative posture/fault penalty plus height excursion cost')
    files.write_json(OUT/'experiment_config.json',config)
    verification.OUT=OUT
    helper=verification.generate(True)
    # Extra diagnostics only: delayed plant inputs before passive hip damping, LPV clamp and request.
    path=helper/'stability_verify.m'
    source=path.read_text(encoding='utf-8')
    source=source.replace('torques=zeros(6,n);','torques=zeros(6,n); requests=zeros(6,n); applied_trace=zeros(6,n); lpv_clamp=false(1,n);')
    source=source.replace('torques(:,k)=accepted;',
        'torques(:,k)=accepted; requests(:,k)=requested; applied_trace(:,k)=u; lpv_clamp(k)=any(l<.16 | l>.34) || abs(mean(x([5 7])))>.3;')
    source=source.replace("'torque',torques(:,j)', 'fallback'","'requested',requests(:,j)', 'applied',applied_trace(:,j)', 'lpv_clamped',lpv_clamp(j), 'torque',torques(:,j)', 'fallback'")
    path.write_text(source,encoding='utf-8')
    return helper,base,center


def read_metrics(folder):
    rows=verification.collect(folder)
    for r in rows:
        raw=files.read_json(folder/f"{r['candidate']}__{r['case']['id']}.json")
        if 'error' in raw:
            raise RuntimeError(raw['error'])
        trace=raw['metrics']['raw'];x=np.array(trace['x']);t=np.array(trace['t']);u=np.array(trace['torque'])
        height=.27+x[:,12]
        r.update(height_min_m=float(height.min()),height_max_m=float(height.max()),
                 height_excursion_m=float(np.max(abs(height-r['case']['height']))),
                 height_rms_m=float(np.sqrt(np.mean((height-r['case']['height'])**2))),
                 vertical_rate_peak=float(np.max(abs(x[:,13]))),
                 axial_upper_fraction=float(np.mean(np.any(u[:,4:]>=150.-.01,axis=1))),
                 axial_lower_fraction=float(np.mean(np.any(u[:,4:]<=20.+.01,axis=1))),
                 end_s=float(t[-1]),lpv_clamped_fraction=float(np.mean(trace.get('lpv_clamped',[False]))))
        if r['height_excursion_m']>.062:
            r['failures'].append('height_tracking_over_60mm_plus_2mm_tolerance')
            r['passed']=False
    files.write_json(folder/'enriched_summary.json',rows)
    return rows


def score(rows):
    failed=sum(not r['passed'] for r in rows)
    violation=max(max(0.,-r['worst_margin_rad']) for r in rows)
    pitch=max(r['angles']['pitch']['peak_rad']/ANGLE_LIMITS[0] for r in rows)
    height=max(r['height_excursion_m']/.05 for r in rows)
    other=np.mean([r['speed_rms']+r['wheel_saturation']+r['fallback'] for r in rows])
    return float((1+pitch**2+.5*height**2+.1*other)*(1+20*failed+100*violation)**2)


def rank_group(rows):
    failed=sum(not r['passed'] for r in rows)
    if failed:return (1,failed,score(rows),0,0)
    return (0,-min(r['worst_margin_rad'] for r in rows),max(r['wheel_saturation'] for r in rows),
            max(r['fallback'] for r in rows),np.mean([r['speed_rms'] for r in rows]))


def batch(stage,rows,cases,helper):
    folder=OUT/stage;folder.mkdir(exist_ok=True)
    plan=dict(rows=rows,scenes=cases)
    if (folder/'plan.json').exists():assert files.read_json(folder/'plan.json')==plan
    files.write_json(folder/'plan.json',plan)
    provenance=dict(helper_sha256=files.digest(helper/'stability_verify.m'),
                    config_sha256=files.digest(OUT/'src/Matlab/mixmpc_pre/tinympc14_config.m'))
    if (folder/'execution_provenance.json').exists():assert files.read_json(folder/'execution_provenance.json')==provenance
    files.write_json(folder/'execution_provenance.json',provenance)
    if stage=='final':
        previous=OUT/'selection'
        assert files.read_json(previous/'execution_provenance.json')==provenance
        reused=[]
        for row in rows:
            for case in cases:
                name=f"{row['id']}__{case['id']}.json"
                if (previous/name).exists() and not (folder/name).exists():
                    result=files.read_json(previous/name)
                    assert result['candidate']==row and result['case']==case
                    shutil.copy2(previous/name,folder/name);reused.append(name)
        files.write_json(folder/'reused_selection_cases.json',reused)
    cmd=f'mixmpc_paths();addpath({files.mq(helper)});stability_verify({files.mq(folder)});'
    code=files.matlab(SimpleNamespace(matlab='matlab'),OUT/'src/Matlab/mixmpc_pre',cmd,folder/'matlab.log',timeout=86400)
    if code:raise RuntimeError(f'{stage} MATLAB failed {code}')
    values=read_metrics(folder)
    assert len(values)==len(rows)*len(cases)
    return values


def diagnose(helper,center):
    cases=[c for c in OLD if c['id'] in ('train_02','selection_02','independent_01','independent_02')]
    ablations=[]
    for c in cases:
        for key,value in (('damping',2.1),('delay_s',.004),('load',0.)):
            ablations.append(dict(c,**{key:value},id=c['id']+'_'+key))
    clean={k:center[k] for k in ('id','Q_DIAG','R_DIAG')}
    batch('mechanism',[clean],cases+ablations,helper)
    w=np.r_[center['Q_DIAG'],center['R_DIAG']]
    points=[clean]
    for axis,(label,group) in enumerate(zip(LABELS,AXES)):
        for multiplier in (.75,1.3333333333333333):
            trial=w.copy();trial[list(group)]*=multiplier
            points.append(point(f'{label}_{multiplier:.2f}',trial))
    batch('sensitivity',points,FOCUS,helper)


class Bridge:
    def __init__(self,helper):
        self.channel=OUT/'bridge';self.channel.mkdir(exist_ok=True)
        # Old request is archived; never replay an incomplete request accidentally on restart.
        for name in ('request.json','done.json','ready.json','stop'):
            path=self.channel/name
            if path.exists():path.replace(path.with_name(name+'.previous'))
        script='''function local_bridge(channel)
fid=fopen(fullfile(channel,'ready.json'),'w'); fprintf(fid,'{}'); fclose(fid);
while ~isfile(fullfile(channel,'stop'))
    request=fullfile(channel,'request.json');
    if ~isfile(request), pause(.1); continue; end
    job=jsondecode(fileread(request)); delete(request);
    try
        stability_verify(job.path);
        result=struct('ok',true,'path',job.path);
    catch err
        result=struct('ok',false,'error',getReport(err,'extended','hyperlinks','off'));
    end
    temporary=fullfile(channel,'done.tmp');fid=fopen(temporary,'w');fprintf(fid,'%s',jsonencode(result));fclose(fid);
    movefile(temporary,fullfile(channel,'done.json'),'f');
end
end
'''
        (helper/'local_bridge.m').write_text(script,encoding='utf-8')
        self.log=(self.channel/'matlab.log').open('wb')
        cmd=f'mixmpc_paths();addpath({files.mq(helper)});local_bridge({files.mq(self.channel)});'
        self.process=subprocess.Popen(['matlab','-batch',cmd],cwd=OUT/'src/Matlab/mixmpc_pre',stdout=self.log,stderr=subprocess.STDOUT)
        self.wait(self.channel/'ready.json',180)
    def wait(self,path,timeout):
        start=time.monotonic()
        while not path.exists():
            if time.monotonic()-start>timeout:raise TimeoutError(str(path))
            time.sleep(.1)
    def evaluate(self,row):
        folder=OUT/'ppo_trials'/row['id'];folder.mkdir(parents=True,exist_ok=True)
        plan=dict(rows=[row],scenes=FOCUS)
        if (folder/'plan.json').exists():assert files.read_json(folder/'plan.json')==plan,'Existing trial has different Q/R or scenarios'
        files.write_json(folder/'plan.json',plan)
        if not all((folder/f"{row['id']}__{c['id']}.json").exists() for c in FOCUS):
            done=self.channel/'done.json'
            if done.exists():done.unlink()
            files.write_json(self.channel/'request.json',dict(path=folder.as_posix()))
            self.wait(done,300)
            result=files.read_json(done)
            if not result['ok']:raise RuntimeError(result['error'])
        return read_metrics(folder)
    def close(self):
        (self.channel/'stop').write_text('stop')
        self.process.wait(timeout=180);self.log.close()


class Env(gym.Env):
    observation_space=gym.spaces.Box(-1,1,(1,),np.float32)
    action_space=gym.spaces.Box(-1,1,(len(AXES),),np.float32)
    def __init__(self,bridge,center,seed):
        self.bridge,self.center,self.seed_id=bridge,center,seed
        self.trials=[]
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed);return np.zeros(1,np.float32),{}
    def step(self,action):
        w=self.center.copy()
        for axes,a in zip(AXES,action):w[list(axes)]*=np.exp(np.log(1.75)*a)
        row=point(f'local_s{self.seed_id}_{len(self.trials):03d}',w)
        metrics=self.bridge.evaluate(row)
        result=dict(**row,seed=self.seed_id,action=action.tolist(),score=score(metrics),cases=metrics)
        self.trials.append(result)
        files.write_json(OUT/'ppo_trials'/row['id']/'trial.json',result)
        print(row['id'],f"score={result['score']:.2f}",f"pitch={max(np.rad2deg(r['angles']['pitch']['peak_rad']) for r in metrics):.2f}",
              f"height={max(r['height_max_m'] for r in metrics):.3f}",flush=True)
        return np.zeros(1,np.float32),-float(np.log1p(result['score'])),True,False,{}


def search(helper,center,steps):
    torch.set_num_threads(1)
    bridge=Bridge(helper);selected=[]
    try:
        for seed in (19,59,117):
            path=OUT/f'ppo_seed{seed}.json'
            if path.exists():trials=files.read_json(path)
            else:
                env=Env(bridge,np.r_[center['Q_DIAG'],center['R_DIAG']],seed)
                agent=PPO('MlpPolicy',env,n_steps=16,batch_size=16,n_epochs=6,learning_rate=.003,gamma=0.,seed=seed,
                          device='cpu',policy_kwargs=dict(net_arch=[32,32],log_std_init=-.7))
                agent.learn(total_timesteps=steps);agent.save(OUT/f'policy_{seed}')
                trials=env.trials;files.write_json(path,trials)
            selected+=sorted(trials,key=lambda r:r['score'])[:2]
    finally:bridge.close()
    files.write_json(OUT/'shortlist.json',[{k:r[k] for k in ('id','Q_DIAG','R_DIAG')} for r in selected])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['diagnose','search','select','final'])
    parser.add_argument('--steps',type=int,default=32)
    args=parser.parse_args()
    helper,base,center=prepare()
    if args.stage=='diagnose':diagnose(helper,center)
    elif args.stage=='search':search(helper,center,args.steps)
    elif args.stage=='select':
        points=files.read_json(OUT/'shortlist.json')
        rows=batch('selection',points,OLD,helper)
        ranked=sorted(points,key=lambda p:rank_group([r for r in rows if r['candidate']==p['id']]))
        files.write_json(OUT/'selection_rank.json',[dict(id=p['id'],rank=rank_group([r for r in rows if r['candidate']==p['id']])) for p in ranked])
        files.write_json(OUT/'finalists.json',ranked[:3])
    else:
        points=[{k:r[k] for k in ('id','Q_DIAG','R_DIAG')} for r in (base,center)]
        points+=files.read_json(OUT/'finalists.json')
        batch('final',points,OLD+fresh_cases(),helper)


if __name__=='__main__':main()
