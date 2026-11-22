"""Constrained TinyMPC14, pointwise Riccati, nonlinear verification of frozen PPO finalists."""
import argparse
from types import SimpleNamespace
import numpy as np
import run_pso as files
from verify_hip_vofa_ppo import compile_replay
from run_stability_ppo import OUT, scenarios
from stability_metrics import summarize


def generate(fidelity=False):
    helper=compile_replay('stability_1ms' if fidelity else 'stability',OUT)
    source=(helper/'hip_vofa_replay.m').read_text(encoding='utf-8')
    tail=source[source.index('function m = simulate('):]
    def replace(old,new):
        nonlocal tail
        if tail.count(old)!=1:raise ValueError(f'Expected unique source anchor: {old}')
        tail=tail.replace(old,new)
    replace('3.2-dt','5.0-dt')
    if fidelity:
        replace('dt = p.Ts;', 'dt = .001;')
        replace('rt = tinympc14_runtime_init(p);','rt = tinympc14_runtime_init(p,1);')
        replace('for sample = 1:2','for sample = 1:1')
        replace('pplant.M_V = p.M_V+s(6); pplant.M_LOAD = p.M_LOAD+s(6);',
                'pplant.M_V = p.M_V+s(6); pplant.M_LOAD = p.M_LOAD+s(6); pplant.m_b=p.m_b+s(6); pplant.M_SUP=p.M_SUP+s(6);')
        replace('accepted(5:6)-p.DF_RATE),accepted(5:6)+p.DF_RATE)',
                'accepted(5:6)-p.DF_RATE*dt/p.Ts),accepted(5:6)+p.DF_RATE*dt/p.Ts)')
        replace('>=p.DF_RATE-1e-6','>=p.DF_RATE*dt/p.Ts-1e-6')
    replace('imp = round(.60/dt)+1;',"imp = round(.65/dt)+1; if p.COMMAND_KIND==3, imp=round(2.8/dt)+1; end")
    replace('if k==imp, x(12)=x(12)+s(7); x(10)=x(10)+s(13); end',
            'if k==imp, x(12)=x(12)+s(7); x(10)=x(10)+s(13); if p.COMMAND_KIND==3, x(2)=x(2)+.15; end; end')
    replace('ref(1)=0; ref(2)=s(8)*ramp; ref(3)=0; ref(4)=s(9)*ramp;',"""ref(1)=0; ref(2)=0; ref(3)=0; ref(4)=0;
    if t(k)>=.2 && t(k)<2.2
        ref(2)=s(8);
        if p.COMMAND_KIND==1 && t(k)>=1.9 && t(k)<2.04, ref(2)=-.35*s(8); end
        if p.COMMAND_KIND==1 && t(k)>=2.04, ref(2)=0; end
        if p.COMMAND_KIND==2 && t(k)>=1.2, ref(2)=-s(8); end
    end""")
    replace('if ~locked && t(k)>=1.52 && abs(x(2))<=.10 && legs_ok',
            'if ref(2)~=0, locked=false; weight=0; end\n    if ~locked && ref(2)==0 && abs(x(2))<=.10 && legs_ok')
    replace('origin=x(1);','origin=x(1); weight=.5;')
    replace('fallback = false(1,n); box = fallback; slew = fallback;',
            'fallback = false(1,n); box = fallback; slew = fallback; commands=zeros(1,n); weights=zeros(1,n); torques=zeros(6,n);')
    replace('trace(:,k) = x;', 'trace(:,k) = x; commands(k)=ref(2); weights(k)=weight; torques(:,k)=accepted;')
    replace('m.locked=locked;', '''m.locked=locked;
m.raw=struct('t',t(j)+dt,'x',trace(:,j)', 'reference',commands(j), ...
    'torque',torques(:,j)', 'fallback',fallback(j),'weight',weights(j));''')
    head='''function stability_verify(out)
p=tinympc14_config();
helper=fileparts(mfilename('fullpath')); addpath(helper,'-begin');
assert(strcmpi(which('tinympc14_model_at'),fullfile(helper,'tinympc14_model_at.m')));
p.PSO_EXACT=true; p.ONLINE_TERMINAL=[];
plan=jsondecode(fileread(fullfile(out,'plan.json')));
for i=1:numel(plan.rows)
    row=plan.rows(i); config=p;
    config.Q_DIAG=row.Q_DIAG(:)'; config.R_DIAG=row.R_DIAG(:)';
    for k=1:numel(plan.scenes)
        scene=plan.scenes(k);
        path=fullfile(out,[char(row.id) '__' char(scene.id) '.json']);
        if isfile(path), continue; end
        config.REPLAY_DAMP=scene.damping;
        config.REPLAY_DELAY=max(1,round(scene.delay_s/config.Ts));
        config.COMMAND_KIND=scene.kind;
        conditions=[scene.height 0 4 9 .578 scene.load .04 scene.velocity 0 4 0 0 .12 1];
        result=struct('candidate',row,'case',scene);
        try
            timer=tic; result.metrics=simulate(config,char(scene.id),conditions);
            result.seconds=toc(timer);
            fprintf('%s %s finite=%d %.1fs\\n',char(row.id),char(scene.id),result.metrics.finite,result.seconds);
        catch err
            result.error=getReport(err,'extended','hyperlinks','off');
            fprintf(2,'%s %s %s\\n',char(row.id),char(scene.id),result.error);
        end
        write_json(path,result);
    end
end
end
'''
    if fidelity:head=head.replace('scene.delay_s/config.Ts','scene.delay_s/.001')
    (helper/'stability_verify.m').write_text(head+'\n'+tail,encoding='utf-8')
    return helper


def collect(folder):
    rows=[]
    for path in folder.glob('*__*.json'):
        raw=files.read_json(path)
        if 'error' in raw:
            metric=dict(passed=False,failures=['simulation_error'],error=raw['error'])
        else:
            m=raw['metrics'];trace=m['raw']
            metric=summarize(trace['t'],trace['x'],trace['reference'],trace['torque'],trace['fallback'],trace['weight'],m['finite'])
            metric['hip_saturation']=float(np.mean(np.any(abs(np.array(trace['torque'])[:,2:4])>=40.-.01,axis=1)))
            metric['old_metrics']={k:v for k,v in m.items() if k!='raw'}
            if not metric['stops'] and raw['case']['velocity']!=0 and not m['finite']:
                metric['stops']=[dict(zero_s=None,stop_s=None,truncated=True,reason='terminated_before_reference_zero',rebound=None)]
        rows.append(dict(candidate=raw['candidate']['id'],case=raw['case'],**metric))
    files.write_json(folder/'summary.json',rows)
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-only',action='store_true')
    parser.add_argument('--collect-only',action='store_true')
    parser.add_argument('--fidelity',action='store_true',help='1ms execution and full added body mass; retain 2ms predictor and 4ms solve')
    parser.add_argument('--split',choices=['train','selection','independent','all'],default='all')
    parser.add_argument('--matlab',default='matlab')
    args=parser.parse_args()
    folder=OUT/('nonlinear_baseline' if args.baseline_only else 'nonlinear_final')
    if args.fidelity:folder=folder.with_name(folder.name+'_1ms')
    folder.mkdir(exist_ok=True)
    if args.collect_only:
        print('collected',len(collect(folder)));return
    baseline=files.read_json(OUT/'baseline.json')
    baseline={k:baseline[k] for k in ('id','Q_DIAG','R_DIAG')}
    rows=[baseline]
    if not args.baseline_only:
        rows += [{k:r[k] for k in ('id','Q_DIAG','R_DIAG')} for r in files.read_json(OUT/'finalists.json')]
    cases=sum([scenarios(s) for s in ('train','selection','independent')],[]) if args.split=='all' else scenarios(args.split)
    plan=dict(rows=rows,scenes=cases)
    old=folder/'plan.json'
    if old.exists() and files.read_json(old)!=plan:raise ValueError('Existing plan differs; use a distinct stage')
    files.write_json(old,plan)
    helper=generate(args.fidelity)
    command=f'mixmpc_paths();addpath({files.mq(helper)});stability_verify({files.mq(folder)});'
    code=files.matlab(SimpleNamespace(matlab=args.matlab),OUT/'src/Matlab/mixmpc_pre',command,folder/'matlab.log',timeout=86400)
    print('MATLAB return',code,'cases',len(collect(folder)))
    if code:raise SystemExit(code)


if __name__=='__main__':main()
