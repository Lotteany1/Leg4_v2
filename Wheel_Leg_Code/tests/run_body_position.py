"""Check actual body position integration with fused and wheel velocity profiles."""
import sys
sys.dont_write_bytecode = True
import re
import subprocess
from run_distributor import ROOT, CC, environment


def main():
    output = ROOT / 'build/body_position'
    output.mkdir(parents=True, exist_ok=True)
    env = environment()
    includes = ['-ICode/' + name for name in ('Config', 'Driver', 'Eatimator', 'Tool', 'Math', 'Model')]
    config = (ROOT / 'Code/Config/system_key.h').read_text(encoding='utf-8')
    for enabled in (0, 1):
        variant = output / f'slip_{enabled}'
        variant.mkdir(exist_ok=True)
        switches, count = re.subn(r'(#define BALANCE_USE_SLIP\s+)\d+u', rf'\g<1>{enabled}u', config)
        assert count == 1, 'BALANCE_USE_SLIP'
        (variant / 'system_key.h').write_text(switches, encoding='utf-8')
        (variant / 'code_config.h').write_bytes((ROOT / 'Code/Config/code_config.h').read_bytes())
        exe = variant / 'body_position.exe'
        subprocess.run([CC, '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                        '-include', 'tests/stubs/ac5_host.h', '-Itests/stubs', f'-I{variant}', *includes,
                        'tests/body_position_test.c',
                        'Code/Eatimator/body.c', 'Code/Tool/lowpass.c', 'Code/Tool/kalman.c',
                        'Code/Math/Leg_InvTrq.c', 'Code/Math/Leg_Trq.c',
                        '-o', str(exe)], cwd=ROOT, env=env, check=True)
        subprocess.run([str(exe)], cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
