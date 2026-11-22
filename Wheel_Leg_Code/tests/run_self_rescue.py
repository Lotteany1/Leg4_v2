"""Run the real self-rescue FSM on the host with measured-state fixtures."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc'), help='Host GCC executable')
    args = parser.parse_args()
    if not args.cc:
        parser.error('Supply --cc or put gcc on PATH')
    root = Path(__file__).resolve().parents[1]
    output = root / 'build/self_rescue_test'
    output.mkdir(parents=True, exist_ok=True)
    compiler = Path(args.cc).resolve()
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    includes = ['-ICode/' + name for name in
                ('App', 'Control', 'Tool', 'Config', 'Eatimator', 'Model', 'Math')]
    executable = output / ('self_rescue.exe' if os.name == 'nt' else 'self_rescue')
    subprocess.run([str(compiler), '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    *includes, 'tests/self_rescue_test.c', 'Code/Tool/pid.c', '-lm',
                    '-o', str(executable)], cwd=root, env=env, check=True)
    subprocess.run([str(executable)], cwd=root, env=env, check=True)

    running = (root / 'Code/Fsm/fsm_running.c').read_text(encoding='utf-8')
    running = '\n'.join(line for line in running.splitlines() if not line.startswith('#include '))
    (output / 'fsm_running_under_test.h').write_text(running, encoding='utf-8')
    target_test = output / ('fsm_targets.exe' if os.name == 'nt' else 'fsm_targets')
    subprocess.run([str(compiler), '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-include', 'tests/stubs/ac5_host.h', '-Itests/stubs', '-ICode/Driver', *includes, '-ICode/Fsm', '-I' + str(output), 'tests/fsm_rescue_targets_test.c',
                    '-lm', '-o', str(target_test)], cwd=root, env=env, check=True)
    subprocess.run([str(target_test)], cwd=root, env=env, check=True)


if __name__ == '__main__':
    main()
