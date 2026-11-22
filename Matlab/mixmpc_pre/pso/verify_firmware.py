"""Run the existing grounded height and roll regression against an isolated package."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(sys.argv[1]).resolve() / "tests"))
import run_distributor as fixture

sources = [
    "tests/distributor_runtime.c",
    "tests/height_roll_closed_loop_test.c",
    *fixture.CONTROLLER,
    "Code/Control/distributor.c",
    "Code/Driver/dm_motor.c",
    "Code/Driver/dji_motor.c",
    *[f"Code/Math/Leg_{name}.c" for name in ("Trq", "Comp", "SpringF", "RollComp", "CoriolisComp", "Pos")],
    "Code/Tool/notch.c",
    "Code/Tool/kalman.c",
]
fixture.build("height_roll_closed_loop", sources)
