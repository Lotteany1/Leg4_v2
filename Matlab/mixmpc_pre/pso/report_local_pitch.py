"""Deliver the local nonlinear PPO experiment with unchanged production evidence."""
import csv
import os
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from local_pitch_ppo import OUT, PRIOR, files, OLD, fresh_cases, read_metrics, rank_group


def main():
    rows=read_metrics(OUT/'final')
    final=files.read_json(OUT/'finalists.json')
    names=['baseline','s117_0185']+[r['id'] for r in final]
    expected={c['id'] for c in OLD+fresh_cases()}
    by={name:{r['case']['id']:r for r in rows if r['candidate']==name} for name in names}
    assert len(rows)==130 and all(set(v)==expected for v in by.values())
    best=min(names[2:],key=lambda name:rank_group(list(by[name].values())))
    eligible=[name for name in names[2:] if all(r['passed'] for r in by[name].values())]
    weights=next(r for r in final if r['id']==best)
    files.write_json(OUT/'closest_candidate.json',dict(**weights,eligible=best in eligible,
        failures={key:r['failures'] for key,r in by[best].items() if r['failures']}))
    protected=['Matlab/mixmpc_pre/tinympc14_config.m']+['Wheel_Leg_Code/Code/Model/'+name for name in files.MODEL_FILES]
    manifest=files.read_json(OUT/'source_sha256.json')
    project=Path(os.environ.get('TINY_LOCAL_PROJECT_ROOT',str(files.ROOT))).resolve()
    proof={name:dict(before=manifest[name.replace('/','\\')],after=files.digest(project/name)) for name in protected}
    assert all(r['before']==r['after'] for r in proof.values()),'Production drift requires reconciliation'
    files.write_json(OUT/'production_protection.json',dict(unchanged=True,files=proof))
    count=sum(len(files.read_json(OUT/f'ppo_seed{seed}.json')) for seed in (19,59,117))
    reused=len(files.read_json(OUT/'final/reused_selection_cases.json'))
    files.write_json(OUT/'verification.json',dict(ppo_trials=count,training_closed_loops=2*count,final_closed_loops=130,
        final_reused_identical_selection_cases=reused,final_new_simulations=130-reused,
        old_cases_now_development=19,new_independent_cases=7,closest=best,eligible=eligible,
        production_unchanged=True,original_gates_unchanged=True,added_height_guard_m=.06,height_guard_tolerance_m=.002,
        build_status='REQUIRED_BEFORE_FIRMWARE' if eligible else 'NOT_RUN_NO_QUALIFIED_CANDIDATE',
        terminal_P_offgrid_gate=.03))
    with (OUT/'case_comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
        columns=['candidate','case','passed','failures','pitch_deg','left_deg','right_deg','height_min_m','height_max_m',
                 'height_excursion_m','wheel_saturation','hip_saturation','fallback','speed_rms','speed_rms_delta_production','speed_rms_delta_center',
                 'stop_s','truncated','rebound','park_weight','drift_m','lpv_clamped_fraction']
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader()
        for name in names:
            for key,r in sorted(by[name].items()):
                stop=r['stops'][-1] if r['stops'] else {}
                writer.writerow(dict(candidate=name,case=key,passed=r['passed'],failures=';'.join(r['failures']),
                    pitch_deg=np.rad2deg(r['angles']['pitch']['peak_rad']),left_deg=np.rad2deg(r['angles']['left_leg']['peak_rad']),
                    right_deg=np.rad2deg(r['angles']['right_leg']['peak_rad']),
                    **{k:r[k] for k in ('height_min_m','height_max_m','height_excursion_m','wheel_saturation','hip_saturation','fallback','speed_rms','lpv_clamped_fraction')},
                    speed_rms_delta_production=r['speed_rms']-by['baseline'][key]['speed_rms'],stop_s=stop.get('stop_s'),
                    speed_rms_delta_center=r['speed_rms']-by['s117_0185'][key]['speed_rms'],
                    truncated=stop.get('truncated'),rebound=stop.get('rebound'),park_weight=stop.get('final_park_weight'),drift_m=stop.get('drift_m')))
    def aggregate(name):
        group=list(by[name].values());stops=[s for r in group for s in r['stops']]
        return [f"{sum(r['passed'] for r in group)}/26"]+[
            f"{max(np.rad2deg(r['angles'][a]['peak_rad']) for r in group):.2f}°" for a in ('pitch','left_leg','right_leg')]+[
            f"{max(r['height_excursion_m'] for r in group)*1000:.1f} mm",
            f"{max(r['height_max_m'] for r in group):.4f} m",
            f"{max(r['wheel_saturation'] for r in group)*100:.2f}% / {max(r['hip_saturation'] for r in group)*100:.2f}%",
            f"{max(r['fallback'] for r in group)*100:.3f}%",
            f"{max((s['stop_s'] for s in stops if s['stop_s'] is not None),default=0):.3f} s / {sum(s['stop_s'] is None for s in stops)} 次未观测",
            f"{max(r['speed_rms'] for r in group if r['case']['height']>=.24):.4f} m/s"]
    labels=['全部门槛通过','pitch 全程峰值','左腿全程峰值','右腿全程峰值','最坏高度跟踪误差','最高绝对高度','最差轮/髋饱和占比','最差回退占比','最慢已观测停稳 / 未观测数','中腿最差速度 RMS']
    a,b,c=(aggregate(name) for name in ('baseline','s117_0185',best))
    text=['# s117_0185 局部 PPO 与失败机理分析','',
          f"结论：{'存在闭环合格候选，尚需构建验收' if eligible else '本轮无合格候选，保留当前生产基线，不生成测试固件'}。最接近局部候选 `{best}`。",'',
          '| 指标 | 生产基线 | s117_0185 | 局部候选 |','|---|---:|---:|---:|']
    text += [f'| {label} | {x} | {y} | {z} |' for label,x,y,z in zip(labels,a,b,c)]
    text += ['', f'均为相同 26 工况的仿真结果，包含提前终止前记录。最终 130 条记录中，{reused} 条复用已完成的相同候选/工况，已核对 MATLAB 验证器和配置 SHA256；其余 {130-reused} 条新运行。较短轨迹的较小峰值不是稳定证明；提前终止硬淘汰。','',
             '## 四个旧失败工况的原因','',
             '详细单变量证据在 `MECHANISM.md`。`independent_01` 的超限发生在起步阶段，不是反向归零阶段；另外三个主要峰值发生在 1.2 s 反向切换后。',
             '最坏切换工况将延迟从 24 ms 降到 4 ms，pitch 从 23.60° 降至 17.79°，高度峰值降至约 0.388 m；去掉 0.30 kg 载荷不能消除失败。降低髋阻尼可降低 pitch，但仍可能触及高度边界，说明 pitch 和升降存在取舍。',
             '模型的高度广义力同时包含轴力投影和轮矩 × sin(腿角)/轮半径的投影。反向后两者共同抬升，随后轴力虽降到 20 N 下限，已积累的向上速度仍造成高度超调。`F_pitch=0`，不存在本次由该已关闭轴力直耦项直接造成 pitch 的证据。力分解为 1 ms 样本重建近似，不能当作独立实车物理辨识。','',
             '## 搜索与门槛','',
             f'- 三个固定种子 19/59/117，共 {count} 次 PPO 参数采样，每次直接运行两个约束 TinyMPC14 非线性训练工况；不再使用看不见高度耦合的线性 LQR 作为奖励。',
             '- 以 s117_0185 为中心，十组成组对数倍率，范围 1/1.75–1.75：速度、双腿角/角速度、pitch/角速度、双轮 R、双髋 R、高度、高度速度、双轴力 R。左右同类项保持对称。附有 20 个单轴倍率点和 12 个机理单变量对照。',
             '- pitch 10°、双腿各 30°及 0.002 rad 公差不变；持续振荡判据、±0.06 m/s 连续 0.20 s 停稳口径不变。固件倒地和仿真提前终止边界不变。',
             '- PPO 开跑前新增更严格的恒高误差门槛：60 mm + 2 mm 数值公差，避免仅压到 0.42 m 故障边界以内就宣称解决腾起。它是本轮工程验收目标，不是已证明的实车安全边界。',
             '- 原来的 19 个工况全部成为本轮开发/回归数据，包括旧 independent_*；只在冻结三名候选后才运行另七个 new_holdout_*。不能把旧留出继续称为独立验证。',
             '- 保持 1 ms 执行、2 ms 预测节点、4 ms 后台求解；逐点 Riccati、原约束/预算/回退/LPV 调度、延迟和载荷不确定性；生产 MATLAB/C 未改动。所有样本完整 Q/R、动作、种子、奖励、原始轨迹和失败项保存于 ppo_trials/。','',
             '## 四个重点工况的前后比较','',
             '| 工况 | s117 pitch / 新 pitch | s117 高度误差 / 新误差 | 局部候选失败项 |','|---|---:|---:|---|']
    for key in ('train_02','selection_02','independent_01','independent_02'):
        old,new=by['s117_0185'][key],by[best][key]
        text.append(f"| {key} | {np.rad2deg(old['angles']['pitch']['peak_rad']):.2f}° / {np.rad2deg(new['angles']['pitch']['peak_rad']):.2f}° | {1000*old['height_excursion_m']:.1f} / {1000*new['height_excursion_m']:.1f} mm | {', '.join(new['failures']) or '无'} |")
    original_failures={name:[key for key,r in by[name].items() if any(f!='height_tracking_over_60mm_plus_2mm_tolerance' for f in r['failures'])] for name in names[2:]}
    files.write_json(OUT/'original_gate_failures.json',original_failures)
    if all(original_failures.values()):
        text += ['', '三组局部候选在原有角度/振荡/故障门槛下也均有失败。因此“无合格候选”的结论不依赖新增的 60 mm 高度误差门槛。']
    text += ['', '## 中腿同工况回归','', '| 工况 | 生产 RMS | 中心 RMS | 局部 RMS | 相对生产 / 中心增量 m/s | 生产/局部首次停稳 s |','|---|---:|---:|---:|---:|---|']
    def stopping(r):
        if not r['stops']:return '驻车扰动'
        s=r['stops'][-1]
        return '未观测' if s['stop_s'] is None else f"{s['stop_s']:.3f}"+('，反弹' if s['rebound'] else '')
    for key,r in sorted(by[best].items()):
        if r['case']['height']<.24:continue
        old=by['baseline'][key]
        center=by['s117_0185'][key]
        text.append(f"| {key} | {old['speed_rms']:.5f} | {center['speed_rms']:.5f} | {r['speed_rms']:.5f} | {r['speed_rms']-old['speed_rms']:+.5f} / {r['speed_rms']-center['speed_rms']:+.5f} | {stopping(old)} / {stopping(r)} |")
    text += ['', '## 构建、限制与复跑','',
             '若全部闭环门槛失败，fast、主机 C 对拍、完整构建、ARM 和测试 HEX 均不运行。终端 P 离网格 3% 等原构建门槛不变。生产配置及六份模型 C/H 的 SHA256 对照在 `production_protection.json`。',
             '这是模型内参数搜索，模型不覆盖全部轮地接触、估计误差、实物执行器动态和自救动作；中腿仍缺少匹配实车 A/B。没有实车效果承诺。',
             '从工程根目录执行；已有阶段会复用同名已完成轨迹，完整重跑请使用另一个隔离实验目录并保留原始记录：','', '```powershell',
             'python -B Matlab/mixmpc_pre/pso/local_pitch_ppo.py diagnose',
             'python -B Matlab/mixmpc_pre/pso/analyze_local_pitch.py',
             'python -B Matlab/mixmpc_pre/pso/local_pitch_ppo.py search --steps 32',
             'python -B Matlab/mixmpc_pre/pso/local_pitch_ppo.py select',
             'python -B Matlab/mixmpc_pre/pso/local_pitch_ppo.py final',
             'python -B Matlab/mixmpc_pre/pso/report_local_pitch.py', '```','',
             '完整重新计算可先设置 `$env:TINY_LOCAL_PITCH_OUT` 为新的绝对目录；冻结源码重跑时另设 `$env:TINY_LOCAL_PITCH_SOURCE` 为本实验 `src` 的绝对路径，`$env:TINY_LOCAL_PITCH_PRIOR` 指向上一轮完整实验目录，然后运行本实验 `harness` 内的同名脚本。归档报告脚本需用 `$env:TINY_LOCAL_PROJECT_ROOT` 指定实际工程根目录，以核对生产保护哈希。不要覆盖或删除已有证据。',
             '逐工况全程/松杆后峰值、时刻、超限时长、恢复时间、角速度、停稳、反弹、漂移和权重在 `final/enriched_summary.json`；精简对照在 `case_comparison.csv`。']
    (OUT/'REPORT.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    fig,axes=plt.subplots(4,2,figsize=(13,12))
    for index,key in enumerate(('train_02','selection_02','independent_01','independent_02')):
        for name,color in (('baseline','gray'),('s117_0185','tab:blue'),(best,'tab:orange')):
            trace=files.read_json(OUT/'final'/f'{name}__{key}.json')['metrics']['raw']
            x=np.array(trace['x']);t=trace['t']
            axes[index,0].plot(t,np.rad2deg(x[:,8]),color=color,label=name)
            axes[index,1].plot(t,.27+x[:,12],color=color,label=name)
        for bound in (-10,10):axes[index,0].axhline(bound,color='r',ls='--',alpha=.5)
        target=by[best][key]['case']['height']
        for bound in (target-.06,target+.06):axes[index,1].axhline(bound,color='r',ls='--',alpha=.5)
        axes[index,0].set_ylabel(key+'\nPitch (deg)');axes[index,1].set_ylabel('Height (m)')
        for ax in axes[index]:ax.grid(alpha=.25);ax.set_xlabel('Time (s)')
    axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8)
    fig.suptitle('Four original failures | traces stop at fault boundary')
    fig.tight_layout();(OUT/'plots').mkdir(exist_ok=True)
    fig.savefig(OUT/'plots/four_failures_comparison.png',dpi=140);plt.close(fig)
    print('best',best,'eligible',eligible)


if __name__=='__main__':main()
