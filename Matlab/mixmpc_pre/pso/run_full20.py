"""Search all twenty Q/R entries with the installed controller as baseline."""
import argparse
from pathlib import Path
import shutil


import run_pso as r
from select_rechecked import publish_config
from verify_full20 import improves, rank, validate

OUT = r.HERE / "build/full20"


def compile_experiment(out):
    m = r.HERE/"matlab"
    build = out/"matlab"
    build.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(m/"tinympc14_model_at.m.txt",build/"tinympc14_model_at.m")
    text = (m/"pso_experiment.m.txt").read_text(encoding="utf-8-sig")
    sim = text[text.index("function m = simulate("):]
    changes = [
        ("x = pl.xb; applied = pl.uss;",
         "x=pl.xb; x(1)=s(11); x(3)=s(12); applied=pl.uss; parked=~any(s(8:9)); yawref=0;"),
        ("if k==imp, x(12)=x(12)+s(7); end",
         "if k==imp, x(12)=x(12)+s(7); x(10)=x(10)+s(13); end"),
        ("ramp = min(1,max(0,(t(k)-.16)/.38));",
         "ramp = min([1,max(0,(t(k)-.16)/.32),max(0,(1.52-t(k))/.32)]);"),
        ("ref(1) = x(1); ref(2) = s(8)*ramp;\n    ref(3) = x(3); ref(4) = s(9)*ramp;",
         "ref(1)=0; ref(2)=s(8)*ramp; ref(3)=0; ref(4)=s(9)*ramp;"),
        ("obs = x; obs(12) = roll_rate;",
         "obs=x; obs(1)=parked*x(1); obs(3)=x(3)-yawref; obs(12)=roll_rate;"),
        ("    x = tinympc14_plant_step(pplant,pl,x,applied+[zeros(4,1);turn;-turn]);", """
    u=applied+[zeros(4,1);turn;-turn];
    if s(14)
        k1=audit_dyn(pplant,x,u);
        k2=audit_dyn(pplant,x+dt*k1/2,u);
        k3=audit_dyn(pplant,x+dt*k2/2,u);
        k4=audit_dyn(pplant,x+dt*k3,u);
        x=x+dt*(k1+2*k2+2*k3+k4)/6;
    else
        x=tinympc14_plant_step(pplant,pl,x,u);
    end
    yawref=yawref+dt*ref(4);"""),
        ("    e(:,k) = x-ref;","    e(:,k)=x-ref; e(1,k)=parked*x(1); e(3,k)=x(3)-yawref;"),
        ("    if any(~isfinite(x))", "    if abs(e(1,k))>.8 || abs(e(3,k))>1.3 || any(~isfinite(x))"),
        ("m.score = 4*m.attitude_peak", """
m.position_rms=rms(e(1,j));
m.yaw_rms=rms(e(3,j));
m.position_settle_s=settle(t(j),abs(e(1,j))<.015 & abs(trace(2,j))<.06,0)*parked;
m.speed_stop_s=settle(t(j),abs(trace(2,j))<.06,1.52)*~parked;
m.speed_rise_lag_s=rise_lag(t(j),trace(2,j),s(8));
m.yaw_rise_lag_s=rise_lag(t(j),trace(4,j),s(9));
m.speed_overshoot=max(0,max(abs(trace(2,j)))-abs(s(8)));
m.score = 1.2*m.position_rms/.10+1.1*m.position_settle_s/2.4 + ...
    1.5*m.speed_rms/.20+.65*m.speed_stop_s/.88 + ...
    .8*m.speed_rise_lag_s/.40+.55*m.yaw_rise_lag_s/.40 + ...
    .4*m.speed_overshoot/.20+.5*m.yaw_rms/.20 + 4*m.attitude_peak"""),
    ]
    for a,b in changes:
        if sim.count(a)!=1:
            raise ValueError(f"Original simulation changed near {a}")
        sim=sim.replace(a,b,1)
    nonlin=(m/"pso_recheck.m.txt").read_text(encoding="utf-8-sig")
    nonlin=nonlin[nonlin.index("function xd = audit_dyn"):].strip()
    helper="""
function lag=rise_lag(t,v,target)
if target==0, lag=0; return; end
k=find(t>=.16 & sign(target)*v>=.5*abs(target),1);
if isempty(k), lag=1.52-.32; else, lag=max(0,t(k)-.32); end
end
"""
    head=(m/"pso_full.m.txt").read_text(encoding="utf-8-sig")
    (build/"pso_full.m").write_text(head+"\n"+sim+"\n"+nonlin+"\n"+helper,encoding="utf-8")


def run_search(args,out):
    out.mkdir(parents=True,exist_ok=True)
    mix=out/"src/Matlab/mixmpc_pre"
    if not mix.exists():
        r.prepare(out)
    (mix/"tinympc14_config.m").write_bytes((out/"baseline.m.txt").read_bytes())
    compile_experiment(out)
    cmd=f"mixmpc_paths();addpath({r.mq(out/'matlab')});pso_full({r.mq(out)},'search','');"
    return r.matlab(args,mix,cmd,out/"logs/search.log",timeout=21600)


def run_refine(args,out):
    settings=out/"refine_seeds.json"
    if not settings.is_file():
        raw=[r.read_json(p) for p in (out/"candidates").glob("*.json")]
        base=next(p for p in raw if p["id"]=="baseline")
        points=sorted((p for p in raw if p["feasible"]),key=lambda p:(not improves(p,base),rank(p,base),p["id"]))
        unique={}
        for p in points:
            unique.setdefault(tuple(p["Q_DIAG"]+p["R_DIAG"]),p)
        r.write_json(settings,list(unique.values())[:6])
    mix=out/"src/Matlab/mixmpc_pre"
    (mix/"tinympc14_config.m").write_bytes((out/"baseline.m.txt").read_bytes())
    cmd=f"mixmpc_paths();addpath({r.mq(out/'matlab')});pso_full({r.mq(out)},'refine',{r.mq(settings)});"
    code=r.matlab(args,mix,cmd,out/"logs/refine.log")
    if code:
        raise RuntimeError(f"Response refinement returned {code}")

def apply_verified(args,out,winner):
    publish_config(args,out,winner)
    proof=r.read_json(out/"applied.json")
    firmware=r.ROOT/"Wheel_Leg_Code"
    try:
        if r.validate_arm(args,firmware,out,"production"):
            raise RuntimeError("Production AC5 link failed")
        release=out/"packages/production/arm"
        manifest=r.read_json(release/"build_manifest.json")
        for f in r.MODEL_FILES:
            if manifest["original_code_sha256"]["Code/Model/"+f] != proof["sources_sha256"][f]:
                raise ValueError(f"Published source differs from AC5 input: {f}")
        proof["production_hex_sha256"]=r.digest(release/"Wheel_Leg_Code.hex")
        proof["production_hex_matches_isolated"]=(
            proof["production_hex_sha256"]==r.digest(out/"packages"/winner["id"]/"arm/Wheel_Leg_Code.hex"))
        r.write_json(out/"applied.json",proof)
    except (Exception,KeyboardInterrupt):
        baseline=out/"pre_publish"
        config=r.ROOT/"Matlab/mixmpc_pre/tinympc14_config.m"
        shutil.copyfile(baseline/"tinympc14_config.m.txt",config)
        for f in proof["sources_sha256"]:
            shutil.copyfile(baseline/f,firmware/"Code/Model"/f)
        (out/"applied.json").replace(out/"rolled_back.json")
        raise

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,default=OUT)
    parser.add_argument("--matlab",default=shutil.which("matlab") or "matlab")
    parser.add_argument("--armcc",type=Path,default=r.ARMCC)
    parser.add_argument("--phase",choices=("all","search","validate"),default="all")
    parser.add_argument("--apply",action="store_true")
    args=parser.parse_args()
    out=args.run.resolve()
    out.relative_to(r.HERE/"build")
    if args.phase in ("all","search"):
        code=run_search(args,out)
        if code:
            raise SystemExit(code)
    if args.phase in ("all","validate"):
        compile_experiment(out)
        run_refine(args,out)
        winner=validate(args,out)
        if winner and args.apply:
            apply_verified(args,out,winner)
        from render_full20 import render
        render(out)
