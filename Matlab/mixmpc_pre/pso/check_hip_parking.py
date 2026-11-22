"""Inspect sagittal hip and wheel allocation against the installed Q/R."""
import re
from sys import argv
from types import SimpleNamespace

import run_pso as r
from run_full20 import compile_experiment


def main():
    archive=r.HERE/"build/full20"
    root=r.HERE/"build/hip_parking_old"
    ppo="--ppo" in argv[1:]
    damping=ppo or "--damping" in argv[1:]
    out=r.HERE/"build/hip_ppo/mpc_verify" if ppo else root/("damping" if damping else "firmware_weight")
    out.mkdir(parents=True,exist_ok=True)
    mix=root/"src/Matlab/mixmpc_pre"
    if not mix.exists():
        src=archive/"src"
        r.copy_tree(src/"Matlab/mixmpc_pre",mix,("build",))
        r.copy_tree(src/"Matlab/Tool",root/"src/Matlab/Tool",("build",))
        header=root/"src/Wheel_Leg_Code/Code/Config/system_key.h"
        header.parent.mkdir(parents=True,exist_ok=True)
        header.write_bytes((src/"Wheel_Leg_Code/Code/Config/system_key.h").read_bytes())
        point=r.read_json(archive/"candidates/response_r1_p07.json")
        config=mix/"tinympc14_config.m"
        r.write_config((archive/"baseline.m.txt").read_bytes().decode("utf-8-sig"),config,point)
        (root/"baseline.m.txt").write_bytes(config.read_bytes())
    compile_experiment(out)
    helper=out/"matlab"
    search=(helper/"pso_full.m").read_text(encoding="utf-8")
    tail=search[search.index("function s = full_scenarios"):].replace(
        "fallback = false(1,n);", "hip=zeros(4,n); longitudinal=zeros(2,n); fallback = false(1,n);"
    ).replace(
        "    fallback(k) = y.fallback;",
        """    hip(:,k)=[requested(3:4); y.Tp];
    longitudinal(:,k)=pl.Bdp(2,1:2)*requested(1:2)/dt;
    longitudinal(2,k)=pl.Bdp(2,3:4)*requested(3:4)/dt;
    motors(:,k)=requested(1:2);
    fallback(k) = y.fallback;"""
    ).replace(
        "m.position_rms=rms(e(1,j));",
        """m.position_rms=rms(e(1,j));
m.hip_net_rms=sqrt(mean(hip(1:2,j).^2,'all'));
m.hip_request_rms=sqrt(mean(hip(3:4,j).^2,'all'));
m.wheel_rms=sqrt(mean(motors(:,j).^2,'all'));
m.accel_rms=rms(longitudinal(:,j),2)';
m.park_overshoot_mm=1000*parked*max([0,-sign(s(11))*trace(1,j)]);
m.late_span_mm=1000*parked*(max(trace(1,late))-min(trace(1,late)));
m.late_speed_rms=rms(trace(2,late));"""
    ).replace("longitudinal(:,k)=pl.Bdp", "longitudinal(1,k)=pl.Bdp").replace(
        "hip=zeros(4,n);","hip=zeros(4,n); motors=zeros(2,n);")
    if damping:
        # Both plant branches see the same passive net torque, while the
        # prediction model can be tested with or without the damping term.
        old="    u=applied+[zeros(4,1);turn;-turn];"
        assert tail.count(old)==1
        tail=tail.replace(old,old+"\n    u(3:4)=u(3:4)-p.PSO_PLANT_DAMP*(x(10)-x([6 8]));")
        model_path=helper/"tinympc14_model_at.m"
        model=model_path.read_text(encoding="utf-8-sig")
        section="[A,B,f] = tinympc14_zoh_ab(A,B,f,p.Ts);"
        assert model.count(section)==1
        model_path.write_text(model.replace(section, """A(:,[6 8])=A(:,[6 8])+p.PSO_PRED_DAMP*B(:,3:4);
A(:,10)=A(:,10)-p.PSO_PRED_DAMP*sum(B(:,3:4),2);
"""+section),encoding="utf-8")
    header=root/"src/Wheel_Leg_Code/Code/Config/system_key.h"
    weight=float(re.search(r"#define\s+FSM_RUNNING_POS_WEIGHT\s+([\d.]+)f",
                           header.read_text(encoding="utf-8-sig"))[1])
    old="obs(1)=parked*x(1);"
    assert tail.count(old)==1
    tail=tail.replace(old,f"obs(1)=parked*{weight:.17g}*x(1);")
    r.write_json(out/"parking_input.json",{"position_feedback_gain":weight,"config_sha256":r.digest(mix/"tinympc14_config.m")})
    name="hip_ppo_verify" if ppo else ("hip_damping" if damping else "hip_parking")
    prefix=(r.HERE/"matlab"/(name+".m.txt")).read_text(encoding="utf-8-sig")
    (helper/(name+".m")).write_text(prefix+"\n"+tail,encoding="utf-8")
    cmd=f"mixmpc_paths();addpath({r.mq(helper)});{name}({r.mq(out)});"
    args=SimpleNamespace(matlab="matlab")
    return r.matlab(args,mix,cmd,out/"run.log")


if __name__=="__main__":
    raise SystemExit(main())





