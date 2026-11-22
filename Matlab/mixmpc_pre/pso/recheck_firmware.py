"""Run the production C feedback/allocator against a grid of heave/roll plants."""
from itertools import product
from pathlib import Path
import sys

import run_pso as r

NAME = "height_roll_sweep"
ROOT = r.ROOT / "Wheel_Leg_Code"


def make_fixture(target):
    source = (ROOT / "tests/height_roll_closed_loop_test.c").read_text(encoding="utf-8")
    # The original assertion is recorded per case in the diagnostic sweep.
    # Constraints and the separate unmodified regression remain in force.
    assertion = "    assert(fabs(late_height - 0.17) < 0.01 && peak < 0.12 && late_rate < 0.15);"
    assert source.count(assertion) == 1
    source = source.replace(assertion, """    printf("LIMITS %s\\n", (fabs(late_height - 0.17) < 0.01
           && peak < 0.12 && late_rate < 0.15) ? "PASS" : "FAIL");""")
    main = ["int main(void)", "{"]
    axes = product((.232, .578, 1.371), ((2, .008), (4, .009), (6, .012)), (0, 4, 16), (-2, 2))
    for inertia, (delay, tau), period, pulse in axes:
        main.append(f"    Run_Closed_Loop({inertia}, {delay}u, {tau}, {period}, {pulse});")
    target.write_text(source.split("int main(void)", 1)[0] + "\n".join(main+["    return 0;", "}", ""]),
                      encoding="utf-8")


def sweep(out, firmware, package, name):
    r.install_package(package, firmware)
    make_fixture(firmware / f"tests/{NAME}.c")
    code = r.run_command([sys.executable, "-B", str(Path(__file__).resolve()), "--fixture", str(firmware)],
                         firmware / "tests", out / f"logs/{name}_sweep.log", 900)
    evidence = firmware / f"build/distributor_check/{NAME}.evidence.json"
    if evidence.is_file():
        (out/f"logs/{name}_sweep.evidence.json").write_bytes(evidence.read_bytes())
    return code


def compile_fixture(firmware):
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(firmware / "tests"))
    import run_distributor as fixture
    sources = [
        "tests/distributor_runtime.c", f"tests/{NAME}.c", *fixture.CONTROLLER,
        "Code/Control/distributor.c", "Code/Driver/dm_motor.c", "Code/Driver/dji_motor.c",
        *[f"Code/Math/Leg_{n}.c" for n in ("Trq", "Comp", "SpringF", "RollComp", "CoriolisComp", "Pos")],
        "Code/Tool/notch.c", "Code/Tool/kalman.c",
    ]
    fixture.build(NAME, sources)


if __name__ == "__main__":
    if sys.argv[1] == "--fixture":
        compile_fixture(Path(sys.argv[2]))
    else:
        out = r.HERE / "build/recheck"
        firmware = out / "c_grid"
        if not firmware.exists():
            for directory in ("Code", "tests"):
                r.copy_tree(ROOT / directory, firmware / directory)
        for name in sys.argv[1:]:
            package = out / "packages" / name / "full"
            if not package.is_dir():
                package = r.HERE / "build/experiment/packages" / name / "full"
            sweep(out, firmware, package, name)
