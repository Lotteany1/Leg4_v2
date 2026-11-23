"""Verify velocity S-curve and real running-target integration with optional AC5."""
import argparse
import os
from itertools import product
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc'), help='Host GCC executable')
    parser.add_argument('--armcc', help='Optional AC5 armcc executable')
    args = parser.parse_args()
    if not args.cc:
        parser.error('Supply --cc or put gcc on PATH')

    root = Path(__file__).resolve().parents[1]
    output = root / 'build/vel_limit_test'
    output.mkdir(parents=True, exist_ok=True)
    compiler = Path(args.cc).resolve()
    env = os.environ.copy()
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    exe = output / ('vel_limit_test.exe' if os.name == 'nt' else 'vel_limit_test')
    subprocess.run([str(compiler), '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                    '-ICode/Config', '-ICode/Model', '-ICode/Control', 'Code/Control/vel_limit.c', 'tests/vel_limit_test.c',
                    '-lm', '-o', str(exe)], cwd=root, env=env, check=True)
    subprocess.run([str(exe)], cwd=root, env=env, check=True)

    config = (root / 'Code/Config/system_key.h').read_text(encoding='utf-8')
    includes = ['-Itests/stubs'] + ['-ICode/' + name for name in
                                  ('Fsm', 'App', 'Task', 'Control', 'Config', 'Driver',
                                   'Eatimator', 'Tool', 'Math', 'Model')]
    sources = ['tests/vel_limit_running_test.c', 'Code/Fsm/fsm_running.c',
               'Code/App/app_rc.c', 'Code/App/app_gyro.c', 'Code/Control/turn_limit.c']
    for air, vel_limit, slip in product((0, 1), repeat=3):
        directory = output / f'air_{air}_vel_{vel_limit}_slip_{slip}'
        directory.mkdir(exist_ok=True)
        variant = config
        # This integration checks turning speed scaling regardless of the deployed switch.
        for feature, value in (('TURN_LIMIT', 1), ('AIR_CHECK', air), ('VEL_LIMIT', vel_limit), ('SLIP', slip)):
            variant = re.sub(r'(#define BALANCE_USE_' + feature + r'\s+)\d+',
                             rf'\g<1>{value}', variant)
        (directory / 'system_key.h').write_text(variant, encoding='utf-8')
        (directory / 'code_config.h').write_bytes((root / 'Code/Config/code_config.h').read_bytes())
        integration_exe = directory / ('running.exe' if os.name == 'nt' else 'running')
        # Disabled builds deliberately omit the S-curve object to verify the bypass has no dependency.
        profile_sources = sources + (['Code/Control/vel_limit.c'] if vel_limit else [])
        subprocess.run([str(compiler), '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                        '-include', 'tests/stubs/ac5_host.h', '-I' + str(directory),
                        *includes, *profile_sources, '-lm', '-o', str(integration_exe)],
                       cwd=root, env=env, check=True)
        subprocess.run([str(integration_exe)], cwd=root, env=env, check=True)

    if args.armcc:
        subprocess.run([args.armcc, '-c', '--cpu', 'Cortex-M7.fp.dp', '--li', '--c99', '--gnu',
                        '-O3', '--split_sections', '-ICode/Config', '-ICode/Model', '-ICode/Control', 'Code/Control/vel_limit.c',
                        '-o', str(output / 'vel_limit.o')], cwd=root, check=True)
        print('AC5 module compile PASS', flush=True)


if __name__ == '__main__':
    main()
