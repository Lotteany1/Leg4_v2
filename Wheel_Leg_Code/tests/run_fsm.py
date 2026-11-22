"""Run real FSM, running and safety behavior with AIR detection enabled and disabled."""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc'), help='Host GCC executable')
    parser.add_argument('--output', default='build/fsm_test', help='Build directory relative to the project root')
    args = parser.parse_args()
    if not args.cc:
        parser.error('Supply --cc or put gcc on PATH')
    root = Path(__file__).resolve().parents[1]
    output = root / args.output
    output.mkdir(parents=True, exist_ok=True)
    compiler = Path(args.cc).resolve()
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    config = (root / 'Code/Config/system_key.h').read_text(encoding='utf-8')
    running = (root / 'Code/Fsm/fsm_running.c').read_text(encoding='utf-8')
    running = '\n'.join(line for line in running.splitlines() if not line.startswith('#include '))
    includes = ['-Itests/stubs'] + ['-ICode/' + name for name in
                                  ('Fsm', 'App', 'Task', 'Control', 'Config', 'Driver',
                                   'Eatimator', 'Tool', 'Math', 'Model')]
    sources = ['tests/fsm_behavior_test.c', 'Code/Fsm/fsm.c',
               'Code/Fsm/fsm_running.c', 'Code/App/app_safety.c', 'Code/Control/vel_limit.c']
    cases = [(case,) for case in ('handshake', 'handoff', 'disable', 'safety', 'failed', 'fall')]
    cases += [('mapping', str(left), str(right)) for left in (1, 3, 2, 0) for right in (1, 3, 2, 0)]
    failures = 0
    for air, step in ((0, 0), (1, 0), (1, 1)):
        directory = output / (f'air_{air}_step' if step else f'air_{air}')
        directory.mkdir(exist_ok=True)
        variant = re.sub(r'(#define BALANCE_USE_AIR_CHECK\s+)\d+', rf'\g<1>{air}', config)
        variant = re.sub(r'(#define STEP_ENABLE\s+)\d+', rf'\g<1>{step}', variant)
        (directory / 'system_key.h').write_text(variant, encoding='utf-8')
        (directory / 'code_config.h').write_bytes((root / 'Code/Config/code_config.h').read_bytes())
        (directory / 'fsm_running_under_test.h').write_text(running, encoding='utf-8')
        suffix = '.exe' if os.name == 'nt' else ''
        executable = directory / ('fsm_behavior' + suffix)
        flags = [str(compiler), '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                 '-include', 'tests/stubs/ac5_host.h', '-I' + str(directory), *includes]
        subprocess.run([*flags, *sources, *([] if os.name == 'nt' else ['-lm']), '-o', str(executable)],
                       cwd=root, env=env, check=True)
        if step:
            for case in ('step_fall', 'step_fault'):
                subprocess.run([str(executable), case], cwd=root, env=env, check=True)
            print('FSM step ownership, fall and fault PASS', flush=True)
            continue
        lines = []
        profile_failures = 0
        for case in cases:
            result = subprocess.run([str(executable), *case], cwd=root, env=env,
                                    capture_output=True, text=True)
            lines.append(f'case={case} returncode={result.returncode}\n' + result.stdout + result.stderr)
            (directory / 'behavior.log').write_text(''.join(lines), encoding='utf-8')
            if result.returncode:
                print(result.stdout + result.stderr, flush=True)
                profile_failures += 1
        targets = directory / ('running_targets' + suffix)
        subprocess.run([*flags, 'tests/fsm_rescue_targets_test.c', 'Code/Control/vel_limit.c',
                        *([] if os.name == 'nt' else ['-lm']), '-o', str(targets)],
                       cwd=root, env=env, check=True)
        result = subprocess.run([str(targets)], cwd=root, env=env, capture_output=True, text=True)
        (directory / 'running.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        if result.returncode:
            print(result.stdout + result.stderr, flush=True)
            profile_failures += 1
        failures += profile_failures
        print(f'FSM + running {"FAIL" if profile_failures else "PASS"}: AIR_CHECK={air} '
              f'({len(cases)} scenarios + targets, {profile_failures} failures)',
              flush=True)
    raise SystemExit(bool(failures))


if __name__ == '__main__':
    main()
