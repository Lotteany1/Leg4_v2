"""Create or reuse an isolated current/frozen baseline, then export its model."""
import argparse
import re
import shutil
from types import SimpleNamespace
from pathlib import Path
import run_pso as files
from run_stability_ppo import OUT
from search_leg_swing import active_cost


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--from-snapshot',type=Path)
    parser.add_argument('--matlab',default='matlab')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    if not (OUT/'src').exists():
        if args.from_snapshot:
            prior=args.from_snapshot.resolve()
            shutil.copytree(prior/'src',OUT/'src')
            shutil.copytree(prior/'captures',OUT/'captures')
            shutil.copy2(prior/'baseline.m.txt',OUT/'baseline.m.txt')
        else:files.prepare(OUT)
    config=(OUT/'baseline.m.txt').read_text(encoding='utf-8')
    weights={key:[float(x) for x in re.search(r'cfg\.'+key+r'\s*=\s*\[([^]]+)\]',config)[1].replace(',',' ').split()]
             for key in ('Q_DIAG','R_DIAG')}
    c=active_cost(OUT/'src/Wheel_Leg_Code/Code/Model/tinympc14_data.c')
    assert weights['Q_DIAG']==c[0] and weights['R_DIAG']==c[1], 'MATLAB/C mismatch; preserve evidence and resolve before searching'
    path=OUT/'baseline.json'
    if not path.exists():files.write_json(path,dict(id='baseline',**weights,matlab_c_equal=True))
    manifest=OUT/'source_sha256.json'
    if not manifest.exists():
        files.write_json(manifest,{str(p.relative_to(OUT/'src')):files.digest(p) for p in (OUT/'src').rglob('*') if p.is_file()})
    if not (OUT/'captures').exists():
        (OUT/'captures').mkdir()
        for name in ('brake01_capture.csv','swing14_capture.csv'):
            shutil.copy2(files.HERE/'build/leg_swing'/name,OUT/'captures'/name)
    files.write_json(OUT/'capture_sha256.json',{p.name:files.digest(p) for p in (OUT/'captures').glob('*.csv')})
    if not (OUT/'model.json').exists():
        export=(files.HERE/'matlab/export_ppo.m.txt').read_text().replace('p.Ts=.004;','').replace(
            'heights=[.21 .27 .31];','heights=[.17 .175 .18 .185 .19 .195 .24 .25 .255 .26 .265];')
        (OUT/'export_ppo.m').write_text(export)
        code=files.matlab(SimpleNamespace(matlab=args.matlab),OUT/'src/Matlab/mixmpc_pre',
                          f'mixmpc_paths();addpath({files.mq(OUT)});export_ppo({files.mq(OUT)});',OUT/'logs/export_model.log')
        if code or not (OUT/'model.json').exists():raise RuntimeError('Model export failed; see export_model.log')
    print(OUT)


if __name__=='__main__':main()
