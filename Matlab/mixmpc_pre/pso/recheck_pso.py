"""Re-run local Q/R probes and an independent, nonlinear scenario suite."""
from pathlib import Path
import argparse
import shutil
from types import SimpleNamespace

import numpy as np

import run_pso as r


def run_matlab_checks(out, mix, original, matlab):
    helper = out / "matlab"
    helper.mkdir(exist_ok=True)
    m = r.HERE / "matlab"
    shutil.copyfile(m / "tinympc14_model_at.m.txt", helper / "tinympc14_model_at.m")
    base = (m / "pso_experiment.m.txt").read_text(encoding="utf-8-sig")
    local = base[base.index("function s = scenarios"):].replace(
        "    x = tinympc14_plant_step(pplant,pl,x,applied+[zeros(4,1);turn;-turn]);",
        """    u = applied+[zeros(4,1);turn;-turn];
    if length(s)>10 && s(11)
        k1=audit_dyn(pplant,x,u);
        k2=audit_dyn(pplant,x+dt*k1/2,u);
        k3=audit_dyn(pplant,x+dt*k2/2,u);
        k4=audit_dyn(pplant,x+dt*k3,u);
        x=x+dt*(k1+2*k2+2*k3+k4)/6;
    else
        x=tinympc14_plant_step(pplant,pl,x,u);
    end""", 1)
    head = (m / "pso_recheck.m.txt").read_text(encoding="utf-8-sig")
    (helper / "pso_recheck.m").write_text(head + "\n" + local, encoding="utf-8")
    ids = ("baseline", "g04_p02", "g04_p04", "g54_p01", "g63_p05")
    entries = [{"id": k, "z": r.read_json(original / f"candidates/{k}.json")["log_multipliers"]} for k in ids]
    center = np.array(entries[1]["z"])
    for i, key in enumerate(("pitch", "dpitch", "roll", "droll", "height", "dheight", "Rforce")):
        for factor, suffix in ((.7, "low"), (1.4, "high")):
            z = center.copy()
            z[i] += np.log(factor)
            entries.append({"id": f"{key}_{suffix}", "z": z.tolist()})
    for key, factors in (
        ("cross_boundary", [.7, .7, 1.2, 2., 1., 1., .9]),
        ("more_droll", [1., 1., 1., 4., 1., 1., 1.]),
        ("slow_vertical", [1., 1., 1., 1., .7, 1.4, 1.]),
    ):
        entries.append({"id": key, "z": (center+np.log(factors)).tolist()})
    r.write_json(out/"probes.json", entries)
    cmd = f"mixmpc_paths();addpath({r.mq(helper)});pso_recheck({r.mq(out)},{r.mq(out/'probes.json')});"
    code = r.matlab(SimpleNamespace(matlab=matlab), mix, cmd, out/"logs/matlab_recheck.log")
    if code:
        raise SystemExit(code)


if __name__ == "__main__":
    opts = argparse.ArgumentParser(description=__doc__)
    opts.add_argument("--run", type=Path, default=r.HERE/"build/recheck")
    opts.add_argument("--matlab", default=shutil.which("matlab") or "matlab")
    args = opts.parse_args()
    out = args.run.resolve()
    # Use the original isolated baseline config. Cold builds change Q/R in a separate copy.
    mix = r.HERE/"build/experiment/src/Matlab/mixmpc_pre"
    run_matlab_checks(out, mix, r.HERE/"build/experiment", args.matlab)
