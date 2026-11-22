"""Run the right-leg bench control task against simulated motor replies and CAN faults."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang"),
                        help="Host GCC or Clang (run MSVC-target Clang in a developer prompt)")
    args = parser.parse_args()
    if not args.cc:
        parser.error("Supply a host compiler with --cc")
    root = Path(__file__).resolve().parents[1]
    output = root / "build" / "spring_bench_host"
    output.mkdir(parents=True, exist_ok=True)
    compiler = str(Path(args.cc).resolve()) if Path(args.cc).exists() else args.cc
    target = subprocess.check_output([compiler, "-dumpmachine"], text=True).strip()
    includes = ["-Itests/stubs", *["-ICode/" + name for name in
                ("App", "Config", "Control", "Driver", "Eatimator", "Fsm", "Math", "Model", "Task", "Tool")]]
    executable = output / ("spring_bench_test.exe" if os.name == "nt" else "spring_bench_test")
    link_flags = [] if "msvc" in target else ["-lm"]
    subprocess.run([compiler, "-std=c99", "-O2",
                    "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-include", "tests/stubs/ac5_host.h",
                    *includes, "tests/spring_bench_test.c", "Code/App/app_spring_cal.c",
                    "Code/App/app_spring_bench.c", "Code/Task/control_task.c", "Code/Fsm/fsm.c",
                    *link_flags, "-o", str(executable)], cwd=root, check=True)
    subprocess.run([str(executable)], cwd=root, check=True)


if __name__ == "__main__":
    main()
