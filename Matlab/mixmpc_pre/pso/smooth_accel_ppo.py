"""Isolated nonlinear PPO for acceleration adaptation and torque smoothness."""
import re, json, shutil, argparse
from pathlib import Path
import numpy as np
import local_pitch_ppo as core
from stable_baselines3 import PPO
import torch
f=core.files
OUT=f.HERE/'build/ppo_smooth_accel_20261119'
core.OUT=OUT
original_metrics=core.read_metrics
BASE_METRICS={}

def cases():
    def make(prefix,vals):
        return [dict(id=f'{prefix}_{i:02d}',height=h,damping=d,velocity=v,delay_s=lag,load=load,kind=k) for i,(h,d,v,lag,load,k) in enumerate(vals)]
    train=make('smooth_train',[(.18,2.1,-1.65,.012,.15,4),(.25,3.2,.9,.016,.2,5)])
    dev=make('smooth_select',[(.175,1.2,-1.85,.010,.1,5),(.195,5.,1.45,.020,.25,4),(.24,1.3,-.85,.010,.15,4),(.255,2.5,.9,.014,.2,5)])
    held=make('smooth_holdout',[(.182,4.4,-1.77,.017,.19,4),(.192,6.3,1.37,.023,.26,5),(.173,.8,-1.92,.007,.08,5),(.247,2.8,-.87,.013,.17,4),(.262,4.1,1.02,.021,.24,5),(.238,.7,.78,.009,.13,4)])
    return train,dev,held

def protected():
    paths=[f.ROOT/'Matlab/mixmpc_pre/tinympc14_config.m']+[f.ROOT/'Wheel_Leg_Code/Code/Model'/n for n in f.MODEL_FILES]
    return {str(p.relative_to(f.ROOT)):f.digest(p) for p in paths}

def prepare():
    OUT.mkdir(exist_ok=True)
    if not (OUT/'protected_sha256.json').exists():f.write_json(OUT/'protected_sha256.json',protected())
    assert protected()==f.read_json(OUT/'protected_sha256.json'),'Production changed during experiment'
    if not (OUT/'src').exists():f.prepare(OUT)
    src=(OUT/'baseline.m.txt').read_text(encoding='utf-8-sig')
    weights=[np.array([float(x) for x in re.search(r'cfg\.'+k+r'\s*=\s*\[([^]]+)\]',src).group(1).replace(',',' ').split()]) for k in ('Q_DIAG','R_DIAG')]
    cw=core.active_cost(OUT/'src/Wheel_Leg_Code/Code/Model/tinympc14_data.c')
    assert all(np.array_equal(a.astype(np.float32),np.array(b,dtype=np.float32)) for a,b in zip(weights,cw))
    base=core.point('baseline_local_s19_031',np.r_[*weights]);f.write_json(OUT/'baseline.json',base)
    train,dev,held=cases();core.FOCUS=train
    config=dict(baseline=base,seeds=[19,59,117],steps_per_seed=32,log_radius=1.3,axes=core.AXES,labels=core.LABELS,
        angle_limits_deg=[10,30,30],angle_tolerance_rad=.002,height_error_limit_m=.062,
        stop=dict(speed=.06,dwell=.2,parking_lock_speed=.1),fault=dict(sim_angle_rad=.6,height_m=[.12,.42],firmware_pitch_roll_rad=1.,firmware_leg_rad=.8,firmware_debounce_ms=200),
        train=train,selection=dev,regression=core.OLD+core.fresh_cases(),independent=held,
        tracking_guard='same-case speed RMS <= baseline*1.03+0.005 m/s; each uncensored stop <= baseline+0.10s; no new post-dwell rebound',
        reward='multiplicative hard failure penalty; normalized wheel/hip 1ms delta peak and 20ms slew RMS, release rebound, tracking and stop time',
        profiles={'4':'reference ramp .2-.7, hold to1.3, ramp to35% by1.65, hold to2, ramp tozero2.4', '5':'step .2, half speed1.0, full speed1.35, reverse -25%1.85, zero2.05'},
        model='1ms execution, 2ms predictor, 4ms solve; constrained TinyMPC14; pointwise DARE; nonlinear RK4. Command profiles are test inputs, firmware shaping unchanged.',
        evidence='No new matched hardware capture supplied; qualitative favorable user test only. Old holdouts treated as regression development cases. New smooth_holdout unseen until shortlist frozen.',
        robustness='Inherited damping 0..7.5 Nms/rad, transport delay4..24ms, extra load0..0.30kg; damping is model-dependent uncertainty, not uniquely identified; feedback age0..1ms is not transport delay.')
    f.write_json(OUT/'experiment_config.json',config)
    core.verification.OUT=OUT;helper=core.verification.generate(True)
    path=helper/'stability_verify.m';s=path.read_text(encoding='utf-8')
    anchor='    err=zeros(p.NX,1);'
    # Insert new reference profiles immediately after the legacy command block.
    old='        if p.COMMAND_KIND==2 && t(k)>=1.2, ref(2)=-s(8); end\n    end'
    assert s.count(old)==1
    s=s.replace(old,old+'''
    if p.COMMAND_KIND==4
        ref(2)=s(8)*interp1([0 .2 .7 1.3 1.65 2 2.4 5],[0 0 1 1 .35 .35 0 0],t(k),'linear');
    elseif p.COMMAND_KIND==5
        ref(2)=0;
        if t(k)>=.2 && t(k)<2.05
            ref(2)=s(8);
            if t(k)>=1 && t(k)<1.35, ref(2)=.5*s(8); end
            if t(k)>=1.85, ref(2)=-.25*s(8); end
        end
    end''')
    path.write_text(s,encoding='utf-8')
    return helper,base

def metrics(folder):
    rows=original_metrics(folder)
    for r in rows:
        trace=f.read_json(folder/f"{r['candidate']}__{r['case']['id']}.json")['metrics']['raw']
        t=np.asarray(trace['t']);x=np.asarray(trace['x']);u=np.asarray(trace['torque']);ref=np.asarray(trace['reference']);dt=np.median(np.diff(t))
        smooth={}
        for name,inds in [('wheel',[0,1]),('hip',[2,3])]:
            a=u[:,inds];delta=np.diff(a,axis=0);n=round(.020/dt);slew=(a[n:]-a[:-n])/(n*dt)
            smooth.update({name+'_delta_peak_nm':float(abs(delta).max()),name+'_slew20_rms_nm_s':float(np.sqrt(np.mean(slew**2))),name+'_tv_nm_s':float(abs(delta).sum()/2/(t[-1]-t[0]))})
        active=abs(ref)>1e-9
        smooth['active_speed_rms']=float(np.sqrt(np.mean((x[active,1]-ref[active])**2))) if active.any() else 0.
        rebound=[]
        for stop in r['stops']:
            if stop.get('stop_s') is None:continue
            start=stop['zero_s']+stop['stop_s']+.2;end=stop['zero_s']+stop['duration_s'];mask=(t>=start)&(t<end)
            rebound.append(float(abs(x[mask,1]).max()) if mask.any() else 0.)
        smooth['post_dwell_peak_m_s']=max(rebound,default=0.)
        r['smooth']=smooth;r['physical_passed']=r['passed']
        b=BASE_METRICS.get(r['case']['id'])
        if b and r['candidate']!=b['candidate']:
            if r['speed_rms']>b['speed_rms']*1.03+.005:r['failures'].append('tracking_regression')
            for i,stop in enumerate(r['stops']):
                if i>=len(b['stops']):break
                bs=b['stops'][i]
                if stop['truncated'] or bs['truncated']:continue
                if bs.get('stop_s') is not None and (stop.get('stop_s') is None or stop['stop_s']>bs['stop_s']+.10):r['failures'].append('stop_regression')
                if stop['rebound'] and not bs['rebound']:r['failures'].append('new_rebound')
            r['passed']=not r['failures']
    f.write_json(folder/'enriched_summary.json',rows)
    return rows
core.read_metrics=metrics

def score(rows):
    loss=[]
    for r in rows:
        b=BASE_METRICS[r['case']['id']];a=r['smooth'];bs=b['smooth']
        terms=[a[k]/max(bs[k],.01) for k in ('wheel_delta_peak_nm','hip_delta_peak_nm','wheel_slew20_rms_nm_s','hip_slew20_rms_nm_s')]
        loss.append(np.mean(terms)+r['speed_rms']/max(b['speed_rms'],.01)+.3*a['post_dwell_peak_m_s']/.06)
    failed=sum(not r['passed'] for r in rows);violation=max(max(0.,-r['worst_margin_rad']) for r in rows)
    return float(np.mean(loss)*(1+20*failed+100*violation)**2)
core.score=score

class Env(core.Env):
    def step(self,action):
        w=self.center.copy()
        for axes,a in zip(core.AXES,action):w[list(axes)]*=np.exp(np.log(1.3)*a)
        row=core.point(f'smooth_s{self.seed_id}_{len(self.trials):03d}',w);m=self.bridge.evaluate(row)
        result=dict(**row,seed=self.seed_id,action=action.tolist(),score=score(m),cases=m);self.trials.append(result)
        f.write_json(OUT/'ppo_trials'/row['id']/'trial.json',result)
        print(row['id'],f"score={result['score']:.4f} passes={sum(r['passed'] for r in m)}/{len(m)}",flush=True)
        return np.zeros(1,np.float32),-float(np.log1p(result['score'])),True,False,{}
core.Env=Env

def rank(rows):
    failed=sum(not r['passed'] for r in rows)
    if failed:return (1,failed,score(rows))
    return (0,-min(r['worst_margin_rad'] for r in rows),score(rows))

def main():
    helper,base=prepare();train,dev,held=cases()
    baseline=core.batch('baseline',[base],train+dev+core.OLD+core.fresh_cases()+held,helper)
    BASE_METRICS.update({r['case']['id']:r for r in baseline})
    # Baseline runs first; independent baseline results never enter PPO or shortlist ranking.
    core.search(helper,base,32)
    points=f.read_json(OUT/'shortlist.json')
    selected=core.batch('selection',points,dev,helper)
    ordered=sorted(points,key=lambda p:rank([r for r in selected if r['candidate']==p['id']]))
    finalists=ordered[:3];f.write_json(OUT/'finalists.json',finalists)
    final=core.batch('validation',finalists,train+dev+core.OLD+core.fresh_cases()+held,helper)
    report(base,baseline,finalists,final)

def report(base,baseline,points,rows):
    import csv
    allrows=baseline+rows
    ranked=sorted(points,key=lambda p:rank([r for r in rows if r['candidate']==p['id']]))
    best=ranked[0];group=[r for r in rows if r['candidate']==best['id']]
    f.write_json(OUT/'closest_candidate.json',dict(**best,eligible=all(r['passed'] for r in group)))
    columns=['candidate','case','passed','failures','pitch_deg','left_deg','right_deg','height_error_mm','speed_rms','wheel_saturation','hip_saturation','fallback','wheel_delta','hip_delta','wheel_slew20','hip_slew20','stop_s','rebound','drift_m']
    with (OUT/'case_comparison.csv').open('w',encoding='utf-8-sig',newline='') as fp:
        writer=csv.DictWriter(fp,fieldnames=columns);writer.writeheader()
        for r in allrows:
            sm=r['smooth'];st=r['stops'];writer.writerow(dict(candidate=r['candidate'],case=r['case']['id'],passed=r['passed'],failures=';'.join(r['failures']),pitch_deg=np.rad2deg(r['angles']['pitch']['peak_rad']),left_deg=np.rad2deg(r['angles']['left_leg']['peak_rad']),right_deg=np.rad2deg(r['angles']['right_leg']['peak_rad']),height_error_mm=r['height_excursion_m']*1000,speed_rms=r['speed_rms'],wheel_saturation=r['wheel_saturation'],hip_saturation=r['hip_saturation'],fallback=r['fallback'],wheel_delta=sm['wheel_delta_peak_nm'],hip_delta=sm['hip_delta_peak_nm'],wheel_slew20=sm['wheel_slew20_rms_nm_s'],hip_slew20=sm['hip_slew20_rms_nm_s'],stop_s=json.dumps([s.get('stop_s') for s in st]),rebound=json.dumps([s['rebound'] for s in st]),drift_m=json.dumps([s.get('drift_m') for s in st])))
    text=['# Acceleration / smoothness PPO','',f'Baseline: {base["id"]}; closest: {best["id"]}. Eligible: {all(r["passed"] for r in group)}.','', '|Metric (worst case unless stated)|Baseline|Closest|','|---|---:|---:|']
    extract={'Passed cases':lambda g:sum(r['passed'] for r in g),'Pitch deg':lambda g:max(np.rad2deg(r['angles']['pitch']['peak_rad']) for r in g),'Left leg deg':lambda g:max(np.rad2deg(r['angles']['left_leg']['peak_rad']) for r in g),'Right leg deg':lambda g:max(np.rad2deg(r['angles']['right_leg']['peak_rad']) for r in g),'Height error mm':lambda g:max(r['height_excursion_m']*1000 for r in g),'Wheel saturation %':lambda g:max(r['wheel_saturation']*100 for r in g),'Fallback %':lambda g:max(r['fallback']*100 for r in g),'Wheel step Nm':lambda g:max(r['smooth']['wheel_delta_peak_nm'] for r in g),'Hip step Nm':lambda g:max(r['smooth']['hip_delta_peak_nm'] for r in g),'Mean wheel slew20 Nm/s':lambda g:np.mean([r['smooth']['wheel_slew20_rms_nm_s'] for r in g]),'Mean hip slew20 Nm/s':lambda g:np.mean([r['smooth']['hip_slew20_rms_nm_s'] for r in g]),'Slowest observed stop s':lambda g:max(s['stop_s'] for r in g for s in r['stops'] if s.get('stop_s') is not None),'Rebound windows':lambda g:sum(s['rebound'] for r in g for s in r['stops']),'Middle-leg worst speed RMS':lambda g:max(r['speed_rms'] for r in g if r['case']['height']>=.23)}
    for key,fun in extract.items():text.append(f'|{key}|{fun(baseline):.6g}|{fun(group):.6g}|')
    text+=['','## Closest candidate failures']+[f'- {r["case"]["id"]}: '+', '.join(r['failures']) for r in group if not r['passed']]
    text+=['','## Reproduce','Run from repository root: `python -B Matlab/mixmpc_pre/pso/smooth_accel_ppo.py`.', 'Existing traces are resumed only with matching immutable plans and helper/config hashes. For a clean rerun copy the frozen harness and change OUT to an empty experiment directory using the frozen src as input.', 'No candidate was published. Production MATLAB/C hashes are checked. No firmware build or board flash performed; eligibility gates precede firmware packaging.','', 'Full per-case posture/release peaks, peak times, violation durations, recovery, angular rates, stop truncation, parking weight, drift, reverse saturation and failure reasons: baseline/enriched_summary.json and validation/enriched_summary.json.']
    (OUT/'REPORT.md').write_text('\n'.join(text),encoding='utf-8')
    after=protected();before=f.read_json(OUT/'protected_sha256.json');assert after==before
    f.write_json(OUT/'verification.json',dict(production_unchanged=True,sha256=after,eligible=all(r['passed'] for r in group),trials=96,validation_records=len(rows),baseline_records=len(baseline),firmware_built=False,board_flashed=False))
    harness=OUT/'harness';harness.mkdir(exist_ok=True)
    for p in f.HERE.glob('*.py'):shutil.copy2(p,harness/p.name)
    for p in f.HERE.glob('*.m'):shutil.copy2(p,harness/p.name)
    f.write_json(OUT/'source_sha256.json',{str(p.relative_to(OUT)):f.digest(p) for root in (OUT/'src',harness) for p in root.rglob('*') if p.is_file()})
    print('\n'.join(text),flush=True)

if __name__=='__main__':main()
