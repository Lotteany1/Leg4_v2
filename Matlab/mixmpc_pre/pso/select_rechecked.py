"""Rank the recheck results, build finalists and optionally publish verified Q/R."""
from pathlib import Path
import argparse
import re
import shutil


import run_pso as r
from recheck_firmware import sweep

OUT = r.HERE / "build/recheck"
SUITES = ("probes", "nonlinear", "holdout")


def load_suites(name):
    return [
        r.read_json(OUT / "probes" / f"{name}.json"),
        r.read_json(OUT / "nonlinear" / f"{name}.json"),
        r.read_json(OUT / "nonlinear" / f"{name}_original_holdout.json"),
    ]


def build_finalist(args, mix, firmware, point):
    name = point["id"]
    if (OUT / f"{name}_build.json").is_file():
        result = r.read_json(OUT / f"{name}_build.json")
    else:
        r.write_config((OUT/"baseline.m.txt").read_text(encoding="utf-8-sig"),
                       mix/"tinympc14_config.m", point)
        pkg = OUT/"packages"/name
        result = {}
        for entry, kind in (("build_mixmpc_pre_fast", "fast"), ("build_mixmpc_pre", "full")):
            result[kind] = r.matlab(args, mix, f"{entry}({r.mq(pkg/kind)});", OUT/f"logs/{name}_{kind}.log")
            if result[kind]:
                break
        if result.get("full") == 0:
            r.install_package(pkg/"full", firmware)
            result["closed"] = r.validate_c(firmware, OUT, name)
            if result["closed"] == 0:
                result["arm"] = r.validate_arm(args, firmware, OUT, name)
        r.write_json(OUT / f"{name}_build.json", result)
    if all(result.get(k) == 0 for k in ("fast", "full", "closed", "arm")):
        sweep_log = OUT/"logs"/f"{name}_sweep.log"
        if not sweep_log.is_file():
            result["sweep_code"] = sweep(OUT, OUT/"c_grid", OUT/"packages"/name/"full", name)
        raw = sweep_log.read_bytes()
        result["sweep_pass"] = raw.count(b"LIMITS PASS")
        result["sweep_fail"] = raw.count(b"LIMITS FAIL")
    return result


def choose(args):
    reference = load_suites("baseline")
    results = []
    for path in sorted((OUT/"nonlinear").glob("*_original_holdout.json")):
        name = path.stem.removesuffix("_original_holdout")
        if name == "baseline":
            continue
        case = load_suites(name)
        if not all(x["feasible"] for x in case):
            continue
        ratios = [x["score"]/b["score"] for x,b in zip(case,reference,strict=True)]
        results.append({"id":name, "eligible": all(r.better(x,b) for x,b in zip(case,reference,strict=True)),
                        "scores":{k:x["score"] for k,x in zip(SUITES,case,strict=True)},
                        "ratio":sum(ratios)/len(ratios)})
    results.sort(key=lambda x:x["ratio"])
    mix = OUT/"src/Matlab/mixmpc_pre"
    firmware = OUT/"src/Wheel_Leg_Code"
    state = {"ranked":results,"selected":None}
    r.write_json(OUT/"decision.json",state)
    for row in results:
        if not row["eligible"]:
            continue
        name = row["id"]
        row["build"] = result = build_finalist(args,mix,firmware,load_suites(name)[0])
        if all(result.get(k) == 0 for k in ("fast","full","closed","arm")) and result.get("sweep_pass") == 54 and result.get("sweep_fail") == 0:
            state["selected"] = name
            r.write_json(OUT/"decision.json",state)
            return name
        r.write_json(OUT/"decision.json",state)
    return None


def comparable_source(path):
    raw = path.read_bytes()
    return re.sub(rb"(?m)^([ \t]*\* C/C\+\+ source code generated on  : )[^\r\n]+", rb"\1<timestamp>", raw)

def publish(args, name):
    if name is None:
        raise ValueError("No verified finalist")
    publish_config(args, OUT, load_suites(name)[0])


def publish_config(args, out, point):
    name = point["id"]
    mix = r.ROOT/"Matlab/mixmpc_pre"
    firmware = r.ROOT/"Wheel_Leg_Code"
    config = mix/"tinympc14_config.m"
    before = config.read_bytes()
    backup = out/"pre_publish"
    backup.mkdir(exist_ok=True)
    if before != (out/"baseline.m.txt").read_bytes():
        raise ValueError("Production config changed since the independent replay")
    (backup/"tinympc14_config.m.txt").write_bytes(before)
    filenames = (*r.MODEL_FILES,"tinympc14_deployment.sha256")
    for f in filenames:
        shutil.copy2(firmware/"Code/Model"/f, backup/f)
    new = before
    for key in ("Q_DIAG", "R_DIAG"):
        values = ", ".join(format(x, ".17g") for x in point[key]).encode("ascii")
        new, count = re.subn(rb"(cfg\."+key.encode("ascii")+rb"\s*=\s*)\[[^\]]+\]",
                             lambda m: m[1]+b"["+values+b"]",new)
        assert count == 1
    pub = out/"packages"/"production_full"
    try:
        config.write_bytes(new)
        for phase, cmd in (
            ("full",f"build_mixmpc_pre({r.mq(pub)});"),
            ("fast","build_mixmpc_pre_fast;"),
        ):
            code = r.matlab(args,mix,cmd,out/f"logs/applied_{phase}.log")
            if code:
                raise RuntimeError(f"Production {phase} returned {code}")
        installed = {}
        for f in filenames:
            installed[f] = r.digest(firmware/"Code/Model"/f)
            live = firmware/"Code/Model"/f
            isolated = out/"packages"/name/"full"/f
            if installed[f] != r.digest(pub/f) or (f in r.MODEL_FILES and comparable_source(live) != comparable_source(isolated)):
                raise ValueError(f"Build outputs differ for {f}")
        r.write_json(out/"applied.json",{"id":name,"Q_DIAG":point["Q_DIAG"],"R_DIAG":point["R_DIAG"],
                                         "config_sha256":r.digest(config),"sources_sha256":installed})
    except (Exception,KeyboardInterrupt):
        config.write_bytes(before)
        for f in filenames:
            shutil.copy2(backup/f,firmware/"Code/Model"/f)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply",action="store_true")
    parser.add_argument("--matlab",default=shutil.which("matlab") or "matlab")
    parser.add_argument("--armcc",type=Path,default=r.ARMCC)
    args = parser.parse_args()
    winner = choose(args)
    print(f"SELECTED {winner}",flush=True)
    if args.apply:
        publish(args,winner)


