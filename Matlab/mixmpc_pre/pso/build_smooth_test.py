"""User-authorized C-only test package; never write the production MATLAB config."""
import shutil
from types import SimpleNamespace
import run_pso as r
from search_leg_swing import active_cost
import numpy as np

OUT=r.HERE/'build/smooth_s19_001_c_test'


def main():
    OUT.mkdir(exist_ok=True)
    point=r.read_json(r.HERE/'build/ppo_smooth_accel_20261119/closest_candidate.json')
    backup=OUT/'before';backup.mkdir(exist_ok=True)
    live=r.ROOT/'Wheel_Leg_Code'
    config=r.ROOT/'Matlab/mixmpc_pre/tinympc14_config.m'
    if not (backup/'tinympc14_config.m').exists():
        shutil.copy2(config,backup/'tinympc14_config.m')
        for name in (*r.MODEL_FILES,'tinympc14_deployment.sha256'):
            shutil.copy2(live/'Code/Model'/name,backup/name)
        r.write_json(OUT/'before_sha256.json',{p.name:r.digest(p) for p in backup.iterdir() if p.is_file()})
    assert config.read_bytes()==(backup/'tinympc14_config.m').read_bytes()
    mix=OUT/'src/Matlab/mixmpc_pre';firmware=OUT/'src/Wheel_Leg_Code'
    if not mix.exists():r.prepare(OUT)
    r.write_config((OUT/'baseline.m.txt').read_text(encoding='utf-8-sig'),mix/'tinympc14_config.m',point)
    args=SimpleNamespace(matlab='matlab',armcc=r.ARMCC)
    result=dict(candidate=point['id'],user_authorized_unqualified_test=True,simulation_eligible=False)
    for entry,kind in (('build_mixmpc_pre_fast','fast'),('build_mixmpc_pre','full')):
        code=r.matlab(args,mix,f'{entry}({r.mq(OUT/"packages"/kind)});',OUT/f'logs/{kind}.log')
        result[kind]=code;r.write_json(OUT/'verification.json',result)
        if code:
            if kind=='fast' and 'CacheStale' in (OUT/'logs/fast.log').read_text(encoding='utf-8',errors='replace'):
                code=r.matlab(args,mix,f'build_mixmpc_pre({r.mq(OUT/"packages/full")});',OUT/'logs/cache_rebuild.log')
                result['cache_rebuild']=code
                if not code:
                    code=r.matlab(args,mix,f'build_mixmpc_pre_fast({r.mq(OUT/"packages/fast")});',OUT/'logs/fast_retry.log')
                    result['fast']=code
            if code:
                r.write_json(OUT/'verification.json',result);raise RuntimeError(f'{kind} failed; production untouched')
    r.install_package(OUT/'packages/full',firmware)
    result['host_closed']=r.validate_c(firmware,OUT,point['id'])
    result['arm']=r.validate_arm(args,firmware,OUT,point['id'])
    r.write_json(OUT/'verification.json',result)
    if result['host_closed'] or result['arm']:raise RuntimeError('Host/ARM validation failed; production untouched')
    q,cost=active_cost(firmware/'Code/Model/tinympc14_data.c')
    assert np.array_equal(np.array(q,dtype='f'),np.array(point['Q_DIAG'],dtype='f'))
    assert np.array_equal(np.array(cost,dtype='f'),np.array(point['R_DIAG'],dtype='f'))
    assert config.read_bytes()==(backup/'tinympc14_config.m').read_bytes()
    for name in r.MODEL_FILES:assert r.digest(live/'Code/Model'/name)==r.digest(backup/name),'Live firmware changed during build'
    r.install_package(OUT/'packages/full',live)
    result.update(c_installed=True,matlab_unchanged=r.digest(config)==r.digest(backup/'tinympc14_config.m'),
                  hex_sha256=r.digest(OUT/'packages'/point['id']/'arm/Wheel_Leg_Code.hex'),board_flashed=False)
    r.write_json(OUT/'verification.json',result)
    print(result,flush=True)


if __name__=='__main__':main()

