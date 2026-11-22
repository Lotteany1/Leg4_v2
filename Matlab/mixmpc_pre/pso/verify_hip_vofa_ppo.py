"""Challenge baseline and screened weights in a nonlinear constrained MPC replay."""
import argparse
from types import SimpleNamespace

from fit_hip_vofa import OUT
from run_full20 import compile_experiment
import run_pso as files


def compile_replay(tag, out=OUT):
    generated_dir = out / ("verify_" + tag)
    compile_experiment(generated_dir)
    helper = generated_dir / "matlab"
    generated = (helper / "pso_full.m").read_text(encoding="utf-8")
    tail = generated[generated.index("function m = simulate("):]
    edits = [
        ("2.4-dt", "3.2-dt"),
        ("parked=~any(s(8:9)); yawref=0;", "parked=~any(s(8:9)); locked=parked; origin=0; weight=.5*parked; yawref=0;"),
        ("obs=x; obs(1)=parked*x(1); obs(3)=x(3)-yawref; obs(12)=roll_rate;", """
    legs_ok=max(abs(x([5 7])))<=.15;
    if ~locked && t(k)>=1.52 && abs(x(2))<=.10 && legs_ok
        locked=true;
        origin=x(1);
    end
    if ~legs_ok
        weight=0;
    elseif locked
        weight=min(.5,weight+dt*.5/.20);
    end
    obs=x; obs(1)=weight*(x(1)-origin); obs(3)=x(3)-yawref; obs(12)=roll_rate;"""),
        ("queue = repmat(applied(5:6),1,delay);", "queue = repmat(applied(5:6),1,delay);\nmotor_queue=repmat(applied(1:4),1,p.REPLAY_DELAY);"),
        ("    turn = pplant.M_SUP", """
    motor_write=mod(k-1,p.REPLAY_DELAY)+1;
    motor_delayed=motor_queue(:,motor_write);
    motor_queue(:,motor_write)=applied(1:4);
    turn = pplant.M_SUP"""),
        ("    u=applied+[zeros(4,1);turn;-turn];", "    u=[motor_delayed;applied(5:6)]+[zeros(4,1);turn;-turn];"),
        ("u(5:6) = u(5:6)+tinympc14_roll_force", "u(3:4) = u(3:4)-p.REPLAY_DAMP*(x([6 8])-x(10));\nu(5:6) = u(5:6)+tinympc14_roll_force"),
        ("late = j(t(j)>=1.8)", "late = j(t(j)>=2.65)"),
        ("m.speed_rise_lag_s=rise_lag", """
m.locked=locked;
m.late_speed_rms=rms(trace(2,late));
m.late_pos_span_m=range(trace(1,late));
m.speed_rise_lag_s=rise_lag"""),
    ]
    for old, new in edits:
        if tail.count(old) != 1:
            raise ValueError(f"Unexpected simulation source around {old}")
        tail = tail.replace(old, new, 1)
    template = (files.HERE / "matlab/hip_vofa_replay.m.txt").read_text(encoding="utf-8")
    (helper / "hip_vofa_replay.m").write_text(template + "\n" + tail, encoding="utf-8")
    return helper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab", default="matlab")
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--export-model", action="store_true")
    parser.add_argument("--tag", choices=("wide", "local"), default="wide")
    args = parser.parse_args()
    mix = OUT / "src/Matlab/mixmpc_pre"
    if not mix.is_dir():
        files.prepare(OUT)
    matlab = SimpleNamespace(matlab=args.matlab)
    if args.export_model:
        export = (files.HERE / "matlab/export_ppo.m.txt").read_text(encoding="utf-8")
        export = export.replace("p.Ts=.004;", "").replace("heights=[.21 .27 .31];", "heights=[.17 .18 .19 .24 .25 .26];")
        (OUT / "export_ppo.m").write_text(export, encoding="utf-8")
        command = f"mixmpc_paths();addpath({files.mq(OUT)});export_ppo({files.mq(OUT)});"
        return files.matlab(matlab, mix, command, OUT / "export_model.log")
    helper = compile_replay(args.tag)
    report = OUT / "mpc"
    report.mkdir(exist_ok=True)
    baseline = "true" if args.baseline_only else "false"
    command = f"mixmpc_paths();addpath({files.mq(helper)});hip_vofa_replay({files.mq(report)},{baseline},'{args.tag}');"
    log = "matlab.log" if args.tag == "wide" else "matlab_local.log"
    return files.matlab(matlab, mix, command, report / log)


if __name__ == "__main__":
    raise SystemExit(main())
