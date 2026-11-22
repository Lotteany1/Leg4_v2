"""Compare fixed raw CAN/history traces with a separately captured pre-refactor baseline."""
import argparse
from pathlib import Path
import subprocess
import json
from run_distributor import ROOT, OUT, CC, environment
from distributor_evidence import begin_evidence, finish_evidence

parser = argparse.ArgumentParser(description=__doc__)
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument('--record', type=Path, help='Capture a baseline before changing production code')
mode.add_argument('--baseline', type=Path, help='Compare with the captured pre-refactor traces')
args = parser.parse_args()
directory = args.record or OUT / 'equivalence'
directory.mkdir(parents=True, exist_ok=True)
sources = ['tests/distributor_runtime.c', 'tests/distributor_equivalence_test.c',
           'Code/Control/balance.c', 'Code/Control/distributor.c',
           'Code/Driver/dm_motor.c', 'Code/Driver/dji_motor.c', 'Code/Tool/notch.c',
           *['Code/Math/' + n + '.c' for n in ('Leg_Trq', 'Leg_Comp', 'Leg_SpringF', 'Leg_RollComp', 'Leg_CoriolisComp')]]
inc = ['-I' + str(p) for p in (ROOT / 'Code').iterdir() if p.is_dir()]
traces = {}
for name, flags in (('default', []), ('project', ['-DBALANCE_USE_DISTRIBUTOR=1']),
                    ('clip', ['-DBALANCE_USE_DISTRIBUTOR=0']),
                    ('compare', ['-DBALANCE_USE_DISTRIBUTOR=1', '-DBALANCE_DISTRIBUTOR_COMPARE=1']),
                    ('clip_compare', ['-DBALANCE_USE_DISTRIBUTOR=0', '-DBALANCE_DISTRIBUTOR_COMPARE=1'])):
    exe = directory / (name + '.exe')
    command = [CC, '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                    '-Wno-tautological-constant-out-of-range-compare', '-include', 'tests/stubs/ac5_host.h',
                    '-Itests/distributor_stubs', '-Itests/stubs', *inc, *flags, *sources, '-o', str(exe)]
    evidence = begin_evidence(command)
    evidence['profile'] = name
    evidence['compile_overrides'] = flags
    subprocess.run(command, cwd=ROOT, env=environment(), check=True)
    result = subprocess.run([str(exe)], cwd=ROOT, env=environment(), capture_output=True, check=True)
    trace = directory / (name + '.trace')
    trace.write_bytes(result.stdout)
    traces[name] = result.stdout
    assert len(result.stdout.splitlines()) == 320
    (directory / (name + '.evidence.json')).write_text(
        json.dumps(finish_evidence(evidence, [exe, trace]), indent=2), encoding='utf-8')
    if args.baseline:
        expected = (args.baseline / (name + '.trace')).read_bytes()
        assert result.stdout == expected, f'{name}: raw CAN / history / physical output differs'
        print(f'PASS: {name}, 320 frames byte-identical to pre-refactor trace')
    else:
        print(f'RECORDED: {name}, 320 frames (comparison not run)')

assert traces['project'] == traces['compare'], 'PROJECT comparison diagnostics changed motor commands'
assert traces['clip'] == traces['clip_compare'], 'CLIP comparison diagnostics changed motor commands'
assert traces['project'] != traces['clip'], 'Saturated fixture did not exercise different allocation policies'
print('PASS: explicit PROJECT/CLIP profiles differ on saturated inputs; comparison preserves both command traces')
