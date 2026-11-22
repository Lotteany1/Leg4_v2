"""Run TinyMPC14 Q/R experiments in an isolated copy of the current worktree."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
from datetime import datetime
from zipfile import ZipFile, ZIP_DEFLATED

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ARMCC = Path("E:/DevelopLotteany1/Keil5/ARM/arm_cc5/bin/armcc.exe")
FIRMWARE_PARTS = ("Code", "Core", "Drivers", "Middlewares", "MDK-ARM", ".eide", ".cmsis", "tests")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_tree(source, target, excludes=()):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns(*excludes, "__pycache__"))


def prepare(run):
    source = run / "src"
    mix = source / "Matlab/mixmpc_pre"
    live = ROOT / "Matlab/mixmpc_pre"
    copy_tree(live, mix, ("build", "pso", "archive"))
    copy_tree(ROOT / "Matlab/Tool", source / "Matlab/Tool", ("build", "archive"))
    firmware = source / "Wheel_Leg_Code"
    for name in FIRMWARE_PARTS:
        copy_tree(ROOT / "Wheel_Leg_Code" / name, firmware / name)
    cache = mix / "build"
    cache.mkdir()
    for name in ("qr_model_cache.mat", "qr_model_cache.mat.sha256"):
        if (live / "build" / name).is_file():
            shutil.copy2(live / "build" / name, cache / name)
    shutil.copy2(mix / "tinympc14_config.m", run / "baseline.m.txt")
    return mix, firmware


def run_command(command, cwd, log, timeout):
    log.parent.mkdir(parents=True, exist_ok=True)
    if log.is_file():
        previous = log.with_name(f"{log.stem}.{datetime.now():%Y%m%d_%H%M%S_%f}{log.suffix}")
        log.replace(previous)
    print(f"{datetime.now().isoformat(timespec='seconds')} {log.name}", flush=True)
    with log.open("wb") as output:
        try:
            return subprocess.run(command, cwd=cwd, stdout=output, stderr=subprocess.STDOUT,
                                  timeout=timeout).returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            output.write(f"\n{exc}\n".encode())
            return 124


def matlab(args, mix, expression, log, timeout=5400):
    return run_command([args.matlab, "-batch", expression], mix, log, timeout)


def mq(path):
    return "'" + Path(path).as_posix().replace("'", "''") + "'"


def checked_baseline(args, mix, run):
    logs = run / "logs"
    base = run / "packages/baseline"
    probe = matlab(args, mix, "D=mixmpc_paths(); qr_build_cache('read',D.ROOT);",
                   logs / "baseline_cache.log")
    if probe:
        full = matlab(args, mix, f"build_mixmpc_pre({mq(base/'full')});",
                      logs / "baseline_full.log")
        fast = matlab(args, mix, f"build_mixmpc_pre_fast({mq(base/'fast')});",
                      logs / "baseline_fast.log")
    else:
        fast = matlab(args, mix, f"build_mixmpc_pre_fast({mq(base/'fast')});",
                      logs / "baseline_fast.log")
        full = matlab(args, mix, f"build_mixmpc_pre({mq(base/'full')});",
                      logs / "baseline_full.log")
    return {"cache_probe": probe, "fast": fast, "full": full}


def write_json(path, data):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def matlab_search(args, mix, run, suffix="", candidates=(), mode=0):
    helper_dir = HERE / "build/matlab"
    helper_dir.mkdir(parents=True, exist_ok=True)
    for template in (HERE / "matlab").glob("*.m.txt"):
        target = helper_dir / template.stem
        if not target.is_file() or target.read_bytes() != template.read_bytes():
            shutil.copyfile(template, target)
    helper = mq(helper_dir)
    prefix = f"mixmpc_paths();addpath({helper});"
    rounds = args.archive_rounds if mode == 2 else args.rounds
    base = f"pso_experiment({mq(run)},{args.particles},{rounds}"
    expression = "".join(f"{base},{mq(path)});" for path in candidates) if candidates else base + (f",{mode});" if mode else ");")
    return matlab(args, mix, prefix + expression, run / f"logs/search{suffix}.log", timeout=86400)


def search(args, mix, run):
    cfg = mix / "tinympc14_config.m"
    cfg.write_bytes((run / "baseline.m.txt").read_bytes())
    for mode, suffix in enumerate(("", "_restart", "_archive")):
        code = matlab_search(args, mix, run, suffix, mode=mode)
        if code:
            raise RuntimeError(f"MATLAB search returned {code}, see logs/search{suffix}.log")


def install_package(package, firmware):
    manifest = package / "tinympc14_deployment.sha256"
    verified = set()
    for line in manifest.read_text(encoding="ascii").splitlines():
        expected, name = line.split()
        name = name.removeprefix("*")
        if name not in MODEL_FILES or digest(package / name) != expected.lower():
            raise ValueError(f"Invalid deployment manifest entry {name}")
        verified.add(name)
    if verified != set(MODEL_FILES):
        raise ValueError("Deployment manifest does not cover all six sources")
    for name in (*MODEL_FILES, manifest.name):
        shutil.copy2(package / name, firmware / "Code/Model" / name)


MODEL_FILES = ("tinympc14.c", "tinympc14.h", "tinympc14_data.c", "tinympc14_data.h",
               "tinympc14_cost.c", "tinympc14_schedule.c")


def validate_c(firmware, run, name):
    # Invoke the existing production-entry height/roll plant fixture directly.
    command = [sys.executable, "-B", str(HERE / "verify_firmware.py"), str(firmware)]
    code = run_command(command, firmware / "tests", run / f"logs/{name}_closed.log", 600)
    artifact = firmware / "build/distributor_check/height_roll_closed_loop.evidence.json"
    if artifact.is_file():
        shutil.copy2(artifact, run / f"logs/{name}_closed.evidence.json")
    return code


def validate_arm(args, firmware, run, name):
    output = firmware / "build" / "pso_arm" / name
    command = [sys.executable, str(firmware / "tests/build_project.py"), "--armcc",
               str(args.armcc.resolve()), "--output", str(output.relative_to(firmware))]
    code = run_command(command, firmware, run / f"logs/{name}_arm.log", 1200)
    if code == 0:
        code = run_command([str(args.armcc.resolve().with_name("fromelf.exe")), "--i32combined",
                            "--output", str(output / "Wheel_Leg_Code.hex"),
                            str(output / "Wheel_Leg_Code.axf")], firmware,
                           run / f"logs/{name}_hex.log", 120)
    if code == 0:
        delivery = run / "packages" / name / "arm"
        delivery.mkdir(parents=True, exist_ok=True)
        for filename in ("Wheel_Leg_Code.hex", "Wheel_Leg_Code.bin", "Wheel_Leg_Code.axf",
                         "project.map", "compile.txt", "link.txt", "build_manifest.json"):
            shutil.copy2(output / filename, delivery / filename)
    return code


def baseline(args, mix, firmware, run):
    config = run / "baseline.m.txt"
    (mix / "tinympc14_config.m").write_bytes(config.read_bytes())
    result_path = run / "baseline_build.json"
    result = read_json(result_path) if result_path.is_file() else checked_baseline(args, mix, run)
    if ("arm" not in result or "closed" not in result) and result["full"] == 0:
        install_package(run / "packages/baseline/full", firmware)
        result["closed"] = validate_c(firmware, run, "baseline")
        result["arm"] = validate_arm(args, firmware, run, "baseline")
    write_json(result_path, result)


def write_config(baseline_text, config, candidate):
    for key in ("Q_DIAG", "R_DIAG"):
        values = ", ".join(format(x, ".17g") for x in candidate[key])
        baseline_text, count = re.subn(rf"(cfg\.{key}\s*=\s*)\[[^\]]+\]",
                                      lambda m: m[1] + "[" + values + "]", baseline_text)
        if count != 1:
            raise ValueError(f"Expected a unique {key} declaration")
    config.write_bytes(baseline_text.encode("utf-8"))


def better(candidate, base):
    return (
        candidate["feasible"] and candidate["score"] < base["score"]
        and all(m["attitude_peak"] <= 1.05 * b["attitude_peak"] + .003
                and m["roll_peak"] <= 1.05 * b["roll_peak"] + .003
                and m["fallback"] <= b["fallback"] + .05
                for m, b in zip(candidate["scenarios"], base["scenarios"], strict=True))
    )


def validate(args, mix, firmware, run):
    folder = run / "candidates"
    base = read_json(folder / "baseline.json")
    holdout_base = read_json(folder / "baseline_holdout.json")
    ranked = sorted((r for p in folder.glob("g??_p??.json") if (r := read_json(p))["feasible"]),
                    key=lambda r: (r["score"], r["id"]))
    unique = {}
    for r in ranked:
        key = tuple(r["Q_DIAG"] + r["R_DIAG"])
        if key != tuple(base["Q_DIAG"] + base["R_DIAG"]) and r["feasible"]:
            unique.setdefault(key, r)
    ranked = list(unique.values())
    write_json(run / "ranking.json", ranked)
    prioritized = [r for r in ranked if better(r, base)]
    ordered = prioritized + [r for r in ranked if r not in prioritized]
    review = ordered[:args.finalists]
    archived = [r for r in ordered if r["id"][1:3] >= "61"]
    if args.finalists >= 4 and archived and not any(r in archived[:2] for r in review):
        # Retain an independently sampled high-roll-rate region in build testing.
        review[-1] = archived[0]
    results = []
    selected = None
    config = mix / "tinympc14_config.m"
    original = run / "baseline.m.txt"
    if review:
        config.write_bytes(original.read_bytes())
        code = matlab_search(args, mix, run, "_holdout",
                             [folder / f"{r['id']}.json" for r in review])
        if code:
            raise RuntimeError(f"MATLAB holdout returned {code}, see logs/search_holdout.log")
    try:
        for r in review:
            name = r["id"]
            holdout = read_json(folder / f"{name}_holdout.json")
            package = run / "packages" / name
            package.mkdir(parents=True, exist_ok=True)
            stage = {"id": name, "search_improved": better(r, base),
                     "holdout_improved": better(holdout, holdout_base)}
            results.append(stage)
            write_config(original.read_text(encoding="utf-8-sig"), config, r)
            shutil.copy2(config, package / "tinympc14_config.m.txt")
            stage["fast"] = matlab(args, mix, f"build_mixmpc_pre_fast({mq(package/'fast')});",
                                   run / f"logs/{name}_fast.log")
            if stage["fast"] == 0:
                install_package(package / "fast", firmware)
                stage["closed"] = validate_c(firmware, run, name)
                if stage["closed"] == 0:
                    stage["full"] = matlab(args, mix, f"build_mixmpc_pre({mq(package/'full')});",
                                           run / f"logs/{name}_full.log")
                    if stage["full"] == 0:
                        install_package(package / "full", firmware)
                        stage["fast_full_match"] = all(
                            digest(package / "fast" / f) == digest(package / "full" / f)
                            for f in MODEL_FILES)
                        if not stage["fast_full_match"]:
                            stage["closed_full"] = validate_c(firmware, run, name + "_full")
                        if stage.get("closed_full", 0) == 0:
                            stage["arm"] = validate_arm(args, firmware, run, name)
                        if stage.get("arm") == 0 and stage["holdout_improved"] and stage["search_improved"]:
                            selected = name
            write_json(run / "validation.json", {"selected": selected, "results": results})
            if selected:
                break
    finally:
        config.write_bytes(original.read_bytes())
        base_package = run / "packages/baseline/full"
        if (base_package / "tinympc14_deployment.sha256").is_file():
            install_package(base_package, firmware)
    write_json(run / "validation.json", {"selected": selected, "results": results})

def export_ab(run):
    selected = read_json(run / "validation.json")["selected"]
    if selected is None:
        return
    archive = run / "TinyMPC14_AB.zip"
    temp = archive.with_suffix(".tmp")
    with ZipFile(temp, "w", compression=ZIP_DEFLATED) as out:
        out.write(HERE / "VOFA.md", "VOFA.md")
        for label, name in (("A_baseline", "baseline"), ("B_" + selected, selected)):
            package = run / "packages" / name
            config = run / "baseline.m.txt" if name == "baseline" else package / "tinympc14_config.m.txt"
            out.write(config, f"{label}/tinympc14_config.m.txt")
            out.write(run / f"candidates/{name}.json", f"{label}/weights_and_metrics.json")
            manifest = read_json(package / "arm/build_manifest.json")
            if manifest["build_status"] != "linked":
                raise ValueError(f"{name}: cross-link status did not pass")
            for filename in MODEL_FILES:
                source = package / "full" / filename
                if manifest["original_code_sha256"][f"Code/Model/{filename}"] != digest(source):
                    raise ValueError(f"{name}: AC5 manifest does not match {filename}")
                out.write(source, f"{label}/Code/Model/{filename}")
            out.write(package / "full/tinympc14_deployment.sha256",
                      f"{label}/Code/Model/tinympc14_deployment.sha256")
            for filename in ("Wheel_Leg_Code.hex", "Wheel_Leg_Code.bin", "build_manifest.json"):
                out.write(package / "arm" / filename, f"{label}/firmware/{filename}")
        out.write(run / "validation.json", "validation.json")
    with ZipFile(temp) as out:
        if (bad := out.testzip()) is not None:
            raise ValueError(f"Corrupt A/B archive entry {bad}")
    temp.replace(archive)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab", default=shutil.which("matlab") or "matlab")
    parser.add_argument("--armcc", type=Path, default=ARMCC)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--phase", choices=("all", "baseline", "search", "validate"), default="all")
    parser.add_argument("--particles", type=int, default=8)
    parser.add_argument("--rounds", type=int, default=4)
    parser.add_argument("--archive-rounds", type=int, default=3)
    parser.add_argument("--finalists", type=int, default=8)
    parser.add_argument("--capture-dir", type=Path, default=ROOT / "Wheel_Leg_Code/diagnostics/height_roll_capture_20261119")
    args = parser.parse_args()
    if args.particles < 4 or args.rounds < 1 or args.archive_rounds < 1 or args.finalists < 1:
        parser.error("Require at least four particles, one round and one finalist")
    run = (args.run or HERE / "build" / datetime.now().strftime("run_%Y%m%d_%H%M%S")).resolve()
    run.relative_to(HERE / "build")
    run.mkdir(parents=True, exist_ok=True)
    mix = run / "src/Matlab/mixmpc_pre"
    firmware = run / "src/Wheel_Leg_Code"
    if not mix.exists():
        mix, firmware = prepare(run)
    if args.phase in ("all", "baseline"):
        baseline(args, mix, firmware, run)
        from analyze_vofa import analyze
        write_json(run / "vofa.json", analyze(args.capture_dir))
    if args.phase in ("all", "search"):
        search(args, mix, run)
    if args.phase in ("all", "validate"):
        validate(args, mix, firmware, run)
        export_ab(run)
        from render_report import render
        render(run, HERE / "REPORT.md")


if __name__ == "__main__":
    main()







