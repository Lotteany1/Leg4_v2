"""Strict host checks for the production distributor, encoding and execution ledger."""
import sys
sys.dont_write_bytecode = True
import argparse
import os
from pathlib import Path
import subprocess
import json
import re
from distributor_evidence import begin_evidence, finish_evidence
from run_step import step_profile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/distributor_check"
CC = "E:/DevelopLotteany1/Matlab/app/toolbox/sldrt/clang/win64/clang.exe"
CONTROLLER = ["Code/Control/balance.c", "Code/Control/turn_limit.c",
              "Code/Model/tinympc14.c", "Code/Model/tinympc14_data.c",
              "Code/Model/tinympc14_cost.c", "Code/Model/tinympc14_schedule.c"]

def environment():
    OUT.mkdir(parents=True, exist_ok=True)
    helper = OUT / "environment.cmd"
    helper.write_text('@echo off\ncall "E:/DevelopLotteany1/VisualStudio2022/Community/VC/Auxiliary/Build/vcvars64.bat" >nul 2>&1\nset\n', encoding="ascii")
    result = subprocess.check_output(["cmd.exe", "/d", "/c", str(helper)], text=True)
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in result.splitlines() if "=" in line and not line.startswith("="))
    env["TEMP"] = env["TMP"] = str(OUT)
    return env

def build(name, sources, extra=(), cases=((),), extra_inputs=(), step=False):
    exe = OUT / (name + ".exe")
    inc = ["-I" + str(p) for p in (ROOT/"Code").iterdir() if p.is_dir()]
    config = (ROOT / "Code/Config/system_key.h").read_text(encoding="utf-8")
    # Profiles override the firmware header in isolation, including unconditional switches.
    command_flags = []
    for flag in extra:
        define = re.fullmatch(r"-D(\w+)=(\w+)", flag)
        if define:
            config, count = re.subn(r"^(#define " + define[1] + r"\s+)\w+",
                                    lambda match: match[1] + define[2], config, flags=re.M)
            if count:
                continue
        command_flags.append(flag)
    profile = OUT / name
    profile.mkdir(parents=True, exist_ok=True)
    config_inputs = []
    for source in (ROOT / "Code/Config").glob("*.h"):
        if source.name == "system_key.h":
            continue
        copy = profile / source.name
        copy.write_bytes(source.read_bytes())
        config_inputs.append(copy)
    (profile / "system_key.h").write_text(step_profile(config, step), encoding="utf-8")
    extra_inputs = [*extra_inputs, ROOT / "tests/run_step.py", *config_inputs]
    command = [CC, "-std=c99", "-O3", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-Wno-tautological-constant-out-of-range-compare", "-include", "tests/stubs/ac5_host.h",
               *command_flags, "-I" + str(profile), "-Itests/distributor_stubs", "-Itests/stubs",
               *inc, *sources, "-o", str(exe)]
    evidence = begin_evidence(command, extra_inputs=[*extra_inputs, profile / "system_key.h"])
    evidence['profile'] = name
    evidence['compile_overrides'] = list(extra)
    result = subprocess.run(command, cwd=ROOT, env=environment(), capture_output=True, text=True, errors="replace")
    (OUT / (name + ".build.log")).write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        evidence['build_status'] = 'compile_failed'
        (OUT / (name + '.evidence.json')).write_text(
            json.dumps(finish_evidence(evidence, [OUT / (name + '.build.log')]), indent=2), encoding='utf-8')
        print(result.stdout + result.stderr)
        raise RuntimeError(name + " build failed")
    failure = None
    evidence['runs'] = []
    with (OUT / (name + ".run.log")).open("w", encoding="utf-8") as log:
        for args in cases:
            result = subprocess.run([str(exe), *args], cwd=ROOT, env=environment(), capture_output=True, text=True)
            log.write(result.stdout + result.stderr)
            print(result.stdout + result.stderr, end="")
            evidence['runs'].append(dict(arguments=list(args), returncode=result.returncode))
            if result.returncode:
                failure = result
                break
    evidence['build_status'] = 'run_failed' if failure else 'passed'
    evidence = finish_evidence(evidence, [exe, OUT / (name + '.build.log'), OUT / (name + '.run.log')])
    (OUT / (name + '.evidence.json')).write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    if failure:
        failure.check_returncode()

def build_spring_mount(spring_source, landing=False, step=False):
    test = "step_integration" if step else ("landing" if landing else "tinympc14_integration")
    sources = ["tests/distributor_runtime.c", f"tests/{test}_test.c",
               *CONTROLLER, "Code/Control/distributor.c",
               *["Code/Driver/" + n + ".c" for n in ("dm_motor", "dji_motor")],
               *["Code/Math/Leg_" + n + ".c" for n in ("Trq", "Comp", "RollComp", "CoriolisComp")], str(spring_source.resolve()),
               "Code/Tool/notch.c", "Code/Tool/lowpass.c", "Code/Eatimator/air.c"]
    if landing:
        sources.append("Code/Math/Leg_InvTrq.c")
    if landing or step:
        sources += ["Code/Fsm/fsm_running.c", "Code/Control/vel_limit.c"]
    if step:
        sources += ["Code/App/app_step.c", "Code/App/app_gyro.c", "Code/Eatimator/slip.c",
                    "Code/Eatimator/slip_side.c", "Code/Tool/kalman.c"]
    build(test if landing or step else "distributor_spring_mount", sources,
          ["-DTEST_SPRING_AIR", *(["-DTEST_LANDING", "-DBALANCE_USE_AIR_CHECK=1"] if landing or step else []),
           *(["-DBALANCE_USE_SLIP=1", "-DBALANCE_USE_SLIP_AIR_INHIBIT=1"] if step else [])],
          extra_inputs=[spring_source, spring_source.with_suffix(".h")], step=step)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spring-mount", type=Path, metavar="LEG_SPRINGF_C", help="Verify horizontal spring compensation with the generated production Leg_SpringF.c")
    parser.add_argument("--landing", action="store_true", help="Run LAND with production AIR, FSM, TinyMPC14 and distribution")
    parser.add_argument("--step", action="store_true", help="Run single-side integration with production AIR, KF, FSM, TinyMPC14 and distribution")
    args = parser.parse_args()
    if args.step:
        build_spring_mount(ROOT / "Code/Math/Leg_SpringF.c", step=True)
        raise SystemExit(0)
    if args.landing:
        build_spring_mount(ROOT / "Code/Math/Leg_SpringF.c", landing=True)
        raise SystemExit(0)
    if args.spring_mount:
        build_spring_mount(args.spring_mount)
        raise SystemExit(0)
    build("vofa_sender", ["tests/vofa_sender_test.c", "Code/Tool/vofa.c"], ["-Itests/vofa_stubs"])
    build("vofa_landing", ["tests/vofa_landing_test.c", "Code/Task/vofa_task.c"])
    execution_sources = ["tests/distributor_runtime.c", "tests/execution_boundary_test.c",
        "Code/Control/distributor.c", "Code/Driver/dm_motor.c", "Code/Driver/dji_motor.c",
        "Code/Math/Leg_Trq.c"]
    build("execution_boundary", execution_sources, cases=[(str(i),) for i in range(4)])
    can_source = (ROOT / "Code/Driver/can_bus.c").read_text(encoding="utf-8")
    recovery = can_source[can_source.index("static void Can_Bus_BusOff_Recover("):can_source.index("// CAN 总线原始计数快照")]
    (OUT / "can_recovery_under_test.h").write_text(recovery, encoding="utf-8")
    build("can_recovery", ["tests/can_recovery_test.c"], ["-I" + str(OUT)])
    sources = ["tests/distributor_runtime.c", "tests/distributor_test.c", *CONTROLLER,
        "Code/Control/distributor.c", "Code/Driver/dm_motor.c", "Code/Driver/dji_motor.c",
        "Code/Tool/notch.c", *["Code/Math/"+name+".c" for name in ("Leg_Trq", "Leg_Comp", "Leg_SpringF", "Leg_RollComp", "Leg_CoriolisComp")]]
    build("distributor", sources)
    build("distributor_project", sources, ["-DBALANCE_USE_DISTRIBUTOR=1"])
    build("distributor_clip", sources, ["-DBALANCE_USE_DISTRIBUTOR=0"])
    build("distributor_compare", sources, ["-DBALANCE_USE_DISTRIBUTOR=1", "-DBALANCE_DISTRIBUTOR_COMPARE=1"])
    build("distributor_clip_compare", sources, ["-DBALANCE_USE_DISTRIBUTOR=0", "-DBALANCE_DISTRIBUTOR_COMPARE=1"])
    controller_sources = ["tests/tinympc14_integration_test.c" if source == "tests/distributor_test.c" else source
                          for source in sources]
    build("tinympc14_integration_project", controller_sources, ["-DBALANCE_USE_DISTRIBUTOR=1"])
    build("tinympc14_integration_clip", controller_sources, ["-DBALANCE_USE_DISTRIBUTOR=0"])
    build("tinympc14_integration_turn", controller_sources, ["-DBALANCE_USE_ROLL_COMP=1"])
    plant_sources = ["tests/height_roll_closed_loop_test.c" if source == "tests/tinympc14_integration_test.c" else source
                     for source in controller_sources]
    build("height_roll_closed_loop", [*plant_sources, "Code/Tool/kalman.c", "Code/Math/Leg_Pos.c"])
