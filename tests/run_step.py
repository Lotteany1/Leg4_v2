"""Exercise the single-side detector and production control phase logic with synthetic feedback."""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


def step_profile(config, enabled):
    """Select one firmware switch in an isolated build."""
    config, count = re.subn(r"(^#define STEP_ENABLE[ \t]+)\d+u", rf"\g<1>{int(enabled)}u", config, flags=re.M)
    assert count == 1, "STEP_ENABLE"
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("gcc"), required=not shutil.which("gcc"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    compiler = Path(args.cc).resolve()
    env = os.environ.copy()
    target = subprocess.check_output([str(compiler), "-dumpmachine"], text=True).strip()
    math_lib = [] if "msvc" in target else ["-lm"]
    env["PATH"] = str(compiler.parent) + os.pathsep + env["PATH"]
    includes = ["-Itests/stubs", *["-ICode/" + name for name in
                ("App", "Config", "Control", "Driver", "Eatimator", "Model", "Task", "Tool")]]
    system_key = (root / "Code/Config/system_key.h").read_text(encoding="utf-8")
    for enabled in (False, True):
        directory = root / "build" / "step_test" / ("enabled" if enabled else "disabled")
        directory.mkdir(parents=True, exist_ok=True)
        profile = step_profile(system_key, enabled)
        if enabled:
            # The step action accepts a request only with slip estimation and AIR enabled.
            profile, count = re.subn(r"(#define BALANCE_USE_AIR_CHECK\s+)\d+u", r"\g<1>1u", profile)
            assert count == 1
        (directory / "system_key.h").write_text(profile, encoding="utf-8")
        for header in (root / "Code/Config").glob("*.h"):
            if header.name != "system_key.h":
                (directory / header.name).write_bytes(header.read_bytes())
        executable = directory / ("step.exe" if os.name == "nt" else "step")
        subprocess.run([str(compiler), "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(directory), *includes, "tests/step_test.c", "Code/App/app_step.c",
                        "Code/Eatimator/slip_side.c", *math_lib, "-o", str(executable)],
                       cwd=root, env=env, check=True)
        subprocess.run([str(executable)], cwd=root, env=env, check=True)


if __name__ == "__main__":
    main()
