"""Exercise real App/FSM and restored global state with simulated hardware."""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--height-handoff', action='store_true', help='Run height handoff and gyro re-entry regression only')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root / 'build/migration_test'
    output.mkdir(parents=True, exist_ok=True)
    sources = ['tests/migration_test.c', 'Code/Fsm/fsm.c', 
               'Code/App/app_safety.c', 'Code/App/app_self_rescue.c', 'Code/Fsm/fsm_running.c',
               'Code/App/app_rc.c', 'Code/App/app_gyro.c', 'Code/Eatimator/body.c', 'Code/Eatimator/leg.c',
               'Code/Eatimator/imu_to_world.c', 'Code/Eatimator/air.c', 'Code/Math/Leg_SpringF.c',
               'Code/Tool/pid.c', 'Code/Tool/kalman.c', 'Code/Tool/lowpass.c', 'Code/Eatimator/slip.c',
               'Code/Math/Leg_Pos.c', 'Code/Math/Leg_Spd.c', 'Code/Math/Leg_InvTrq.c', 'Code/Math/Leg_Trq.c',
               'Code/Control/turn_limit.c']
    includes = ['-Itests/stubs'] + [f'-ICode/{d}' for d in
                                  ('App', 'Fsm', 'Task', 'Config', 'Control', 'Driver', 'Eatimator', 'Math', 'Tool', 'Model')]
    env = os.environ.copy()
    env['PATH'] = str(Path(args.cc).parent) + os.pathsep + env.get('PATH', '')
    exe = output / 'migration.exe'
    subprocess.run([args.cc, '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
                    '-fdata-sections', '-include', 'tests/stubs/ac5_host.h',
                    *includes, *sources, '-Wl,--gc-sections', '-lm', '-o', str(exe)],
                   cwd=root, env=env, check=True)
    subprocess.run([str(exe), *(['--height-handoff'] if args.height_handoff else [])], cwd=root, env=env, check=True)
    print('App/FSM PASS: TinyMPC14 height references', flush=True)

    if args.height_handoff:
        return

    exe = output / 'task_sync.exe'
    subprocess.run([args.cc, '-std=c99', '-O3', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-include', 'tests/stubs/ac5_host.h',
                    *includes, '-Itests/distributor_stubs', 'tests/task_sync_test.c', 'Code/Task/estimator_task.c',
                    'Code/Task/control_task.c', '-o', str(exe)], cwd=root, env=env, check=True)
    subprocess.run([str(exe)], cwd=root, env=env, check=True)


if __name__ == '__main__':
    main()
