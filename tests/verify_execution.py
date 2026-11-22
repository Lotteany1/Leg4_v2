"""Independent float64 event ledger oracle for the real C execution history.

Uses physical geometry from the research FK/Jacobian reference, raw integer
protocol codes and explicit time segments; it does not call Distributor_Recover.
"""
import ctypes as ct
import json
import subprocess
import sys
import numpy as np
from run_distributor import ROOT, OUT, CC, environment
from distributor_evidence import begin_evidence, finish_evidence

sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT.parent / "research/actuator_coordination_20261101_01"))
from mapping_model import ik, sent_map


class Sample(ct.Structure):
    _fields_ = [("seq", ct.c_uint32), ("epoch", ct.c_uint32), ("tick", ct.c_uint32), ("cycle", ct.c_uint32),
                ("offset", ct.c_uint32 * 5), ("map", ct.c_float * 8), ("spring", ct.c_float * 2),
                ("comp", ct.c_float * 2), ("turn", ct.c_float * 2), ("before", ct.c_int32 * 6),
                ("code", ct.c_int32 * 6), ("u", ct.c_float * 4), ("s", ct.c_float * 2),
                ("valid", ct.c_uint8), ("valid_before", ct.c_uint8), ("accepted", ct.c_uint8)]


class Interval(ct.Structure):
    _fields_ = [("epoch", ct.c_uint32), ("seq", ct.c_uint32), ("u", ct.c_float * 4),
                ("s", ct.c_float * 2), ("turn", ct.c_float * 2), ("end_s", ct.c_float * 2),
                ("min_u", ct.c_float * 4), ("max_u", ct.c_float * 4),
                ("min_s", ct.c_float * 2), ("max_s", ct.c_float * 2), ("complete", ct.c_bool)]


def main():
    dll = OUT / "execution_reference.dll"
    exports = ["Execution_Init", "Execution_Mode", "Execution_Begin", "Execution_Publish", "Execution_Read", "Execution_Epoch"]
    command = [CC, "-std=c99", "-O3", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-shared",
                    "-include", "tests/stubs/ac5_host.h", "-Itests/distributor_stubs", "-Itests/stubs",
                    *["-I" + str(p) for p in (ROOT / "Code").iterdir() if p.is_dir()],
                    "-Wno-tautological-constant-out-of-range-compare",
                    "tests/distributor_runtime.c", "Code/Control/distributor.c", "Code/Driver/dm_motor.c",
                    "Code/Driver/dji_motor.c", "Code/Math/Leg_Trq.c",
                    "-o", str(dll), *["-Wl,/EXPORT:" + name for name in exports]]
    evidence = begin_evidence(command, extra_inputs=[ROOT.parent / 'research/actuator_coordination_20261101_01/mapping_model.py'])
    subprocess.run(command, cwd=ROOT, env=environment(), check=True)
    lib = ct.CDLL(str(dll))
    lib.Execution_Epoch.restype = ct.c_uint32
    lib.Execution_Begin.argtypes = lib.Execution_Publish.argtypes = [ct.POINTER(Sample)]
    lib.Execution_Read.argtypes = [ct.c_uint32] * 5 + [ct.POINTER(Interval)]
    assert ct.sizeof(Sample) == 168
    rng = np.random.default_rng(20261102)
    lib.Execution_Init()
    lib.Execution_Mode(0)
    epoch = lib.Execution_Epoch()
    step, origin, tick_origin = 550000, 0xFFFF8000, 0xFFFFFFF0
    segments = []
    held = np.zeros(6, dtype=np.int32)
    valid = 0
    for frame in range(80):
        sample = Sample()
        lib.Execution_Begin(ct.byref(sample))
        sample.tick = (tick_origin + frame) & 0xFFFFFFFF
        sample.cycle = (origin + frame * step) & 0xFFFFFFFF
        matrices = np.array([sent_map(ik(rng.uniform(.16, .34), rng.uniform(1.2, 1.9)), side == 1) for side in range(2)], dtype=np.float32)
        sample.map[:] = matrices.ravel()
        sample.spring[:] = rng.uniform(20, 160, 2)
        sample.comp[:] = np.array([8., -5.]) * (-1 if frame % 2 else 1)
        sample.turn[:] = rng.uniform(-30, 30, 2)
        sample.code[:4] = rng.integers(1000, 3000, 4)
        sample.code[4:] = rng.integers(-10000, 10000, 2)
        sample.offset[:] = [1000, 2200, 3300, 4100, 5200]
        sample.accepted = (rng.integers(0, 16).item() | (48 if rng.random() > .25 else 0)) if frame else 63
        points = [frame * step, *[frame * step + t for t in sample.offset], (frame + 1) * step]
        for event in range(6):
            if event:
                mask = (1 << (event - 1)) if event < 5 else 48
                for ch in range(6):
                    if mask & sample.accepted & (1 << ch):
                        held[ch] = sample.code[ch]
                        valid |= 1 << ch
            # Decode exact protocol grid in double precision, then solve A x=tau.
            tau = held[:4] * (108. / 4095.) - 54.
            physical = np.array([np.linalg.solve(a.astype(float), t) for a, t in zip(matrices, tau.reshape(2, 2))])
            s = physical[:, 0] + np.array(sample.spring)
            u = np.r_[-held[4] / 3326.29, held[5] / 3326.29, physical[:, 1] - np.array(sample.comp)]
            segments.append((points[event], points[event + 1], valid, u.copy(), s.copy(), np.array(sample.turn)))
        lib.Execution_Publish(ct.byref(sample))

    max_error = 0.
    for case in range(400):
        first = int(rng.integers(50, 75))
        begin = first * step + int(rng.integers(0, 6000))
        end = begin + int(rng.integers(1, 4 * step))
        end_tick = (tick_origin + end // step) & 0xFFFFFFFF
        result, another = Interval(), Interval()
        for output in (result, another):
            lib.Execution_Read(epoch, (origin + begin) & 0xFFFFFFFF, (origin + end) & 0xFFFFFFFF,
                               end_tick, step, ct.byref(output))
        assert result.complete and another.complete, case
        assert bytes(result) == bytes(another), "independent readers must not consume history"
        weighted, values = np.zeros(8), []
        end_s = None
        for lo, hi, known, u, s, turn in segments:
            duration = max(0, min(hi, end) - max(lo, begin))
            if duration:
                assert known == 63
                values.append(np.r_[u, s + turn])
                weighted += duration / (end - begin) * np.r_[u, s, turn]
                end_s = s
        expected = np.r_[weighted, end_s, np.min(values, axis=0), np.max(values, axis=0)]
        actual = np.r_[result.u, result.s, result.turn, result.end_s,
                       result.min_u, result.min_s, result.max_u, result.max_s]
        error = np.max(np.abs(expected - actual))
        max_error = max(max_error, error)
        assert np.allclose(expected, actual, rtol=3e-5, atol=8e-4), (case, error)
    result = Interval()
    lib.Execution_Read(epoch, origin & 0xFFFFFFFF, (origin + step) & 0xFFFFFFFF,
                       tick_origin, step, ct.byref(result))
    assert not result.complete, "overwritten history must remain incomplete"
    report = dict(cases=400, frames=80, seed=20261102, max_absolute_error=max_error,
                  sample_bytes=ct.sizeof(Sample), oracle="float64 raw CAN grid + independent Jacobian + explicit event segments")
    report['evidence'] = finish_evidence(evidence, [dll])
    (OUT / "execution_reference.json").write_text(json.dumps(report, indent=2))
    print("PASS:", {key: value for key, value in report.items() if key != 'evidence'})


if __name__ == "__main__":
    main()
