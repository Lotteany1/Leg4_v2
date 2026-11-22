"""Run radar sector and DMA regression tests on the host (C99)."""
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
    output = root / 'build/radar_test'
    output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env['PATH'] = str(Path(args.cc).parent) + os.pathsep + env['PATH']
    for profile, defines in [('wrap', []), ('regular', ['-DRADAR_CENTER_DEG=95.0f', '-DRADAR_ANGLE_DEG=30.0f']), ('near_zero', ['-DRADAR_CENTER_DEG=355.0f']), ('narrow', ['-DRADAR_ANGLE_DEG=10.0f'])]:
        exe = output / (profile + '.exe')
        subprocess.run([args.cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-Itests/radar_stubs', '-ICode/Driver', '-ICode/Config', '-ICode/Math', '-ICode/Model', *defines,
                        'Code/Driver/radar.c', 'tests/radar_test.c', '-lm', '-o', str(exe)],
                       cwd=root, env=env, check=True)
        subprocess.run([str(exe)], cwd=root, env=env, check=True)


if __name__ == '__main__':
    main()
