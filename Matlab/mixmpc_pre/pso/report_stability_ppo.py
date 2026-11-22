"""Make auditable per-case comparisons; never publish a failed candidate."""
import csv
import importlib.metadata
import shutil
import sys
import numpy as np
from run_stability_ppo import OUT, files, scenarios
from verify_stability_ppo import collect


def main():
    folder=OUT/'nonlinear_final_1ms'
    rows=collect(folder)
    candidates=files.read_json(OUT/'finalists.json')
    names=['baseline']+[r['id'] for r in candidates]
    expected={c['id'] for s in ('train','selection','independent') for c in scenarios(s)}
    by={name:{r['case']['id']:r for r in rows if r['candidate']==name} for name in names}
    assert all(set(v)==expected for v in by.values()),'Verification incomplete'
    assert all('angles' in r for r in rows),'Simulation errors require resolution'
    def ranking(name):
        values=list(by[name].values());bad=sum(not r['passed'] for r in values)
        return (bad,-min(r['worst_margin_rad'] for r in values),max(r['fallback'] for r in values),
                max(r['wheel_saturation'] for r in values),max(r['speed_rms'] for r in values))
    best=min(names[1:],key=ranking)
    eligible=[name for name in names[1:] if ranking(name)[0]==0]
    best_weights=next(r for r in candidates if r['id']==best)
    files.write_json(OUT/'closest_candidate.json',dict(**best_weights,eligible=best in eligible,
                     failures={k:r['failures'] for k,r in by[best].items() if not r['passed']}))
    protected=['Matlab/mixmpc_pre/tinympc14_config.m']+['Wheel_Leg_Code/Code/Model/'+x for x in files.MODEL_FILES]
    source_manifest=files.read_json(OUT/'source_sha256.json')
    protection={name:dict(before=source_manifest[name.replace('/', '\\')],after=files.digest(files.ROOT/name)) for name in protected}
    unchanged=all(v['before']==v['after'] for v in protection.values())
    files.write_json(OUT/'production_protection.json',dict(unchanged=unchanged,files=protection))
    assert unchanged,'Production changed during run; do not claim baseline retained without reconciliation'
    proof=dict(complete_closed_loops=len(rows),expected_closed_loops=76,eligible=eligible,closest=best,
               production_unchanged=unchanged,gate_result='PASS' if eligible else 'NO_QUALIFIED_CANDIDATE',
               stages={name:('NOT_RUN: no candidate passes posture and independent closed-loop gates' if not eligible else 'REQUIRED')
                       for name in ('fast','host_C','full_build','ARM','test_HEX')},
               terminal_P_offgrid_limit=.03,threshold_relaxed=False,
               versions={name:importlib.metadata.version(name) for name in ('numpy','scipy','gymnasium','stable-baselines3','torch')},
               python=sys.version)
    files.write_json(OUT/'verification.json',proof)
    fields=['candidate','case','height_m','damping','delay_ms','load_kg','kind','passed','failures',
            'pitch_deg','left_deg','right_deg','pitch_postzero_deg','left_postzero_deg','right_postzero_deg',
            'wheel_saturation_pct','hip_saturation_pct','fallback_pct','reverse_sat_left','reverse_sat_right',
            'stop_s','stop_truncated','rebound','park_weight','drift_m','speed_rms','speed_rms_delta_baseline']
    with (OUT/'case_comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for name in names:
            for case,r in sorted(by[name].items()):
                s=r['case'];stop=r['stops'][-1] if r['stops'] else {}
                line=dict(candidate=name,case=case,height_m=s['height'],damping=s['damping'],delay_ms=1000*s['delay_s'],
                          load_kg=s['load'],kind=s['kind'],passed=r['passed'],failures=';'.join(r['failures']),
                          wheel_saturation_pct=100*r['wheel_saturation'],hip_saturation_pct=100*r['hip_saturation'],
                          fallback_pct=100*r['fallback'],reverse_sat_left=r['reverse_saturation_counts'][0],
                          reverse_sat_right=r['reverse_saturation_counts'][1],stop_s=stop.get('stop_s'),
                          stop_truncated=stop.get('truncated'),rebound=stop.get('rebound'),park_weight=stop.get('final_park_weight'),
                          drift_m=stop.get('drift_m'),speed_rms=r['speed_rms'],speed_rms_delta_baseline=r['speed_rms']-by['baseline'][case]['speed_rms'])
                for axis,col in [('pitch','pitch'),('left_leg','left'),('right_leg','right')]:
                    a=r['angles'][axis];line[col+'_deg']=np.rad2deg(a['peak_rad'])
                    line[col+'_postzero_deg']=max((np.rad2deg(z['peak_rad']) for z in a['releases']),default=None)
                writer.writerow(line)
    def aggregate(name):
        values=list(by[name].values());stops=[s for r in values for s in r['stops']]
        return [f"{sum(r['passed'] for r in values)}/19"]+[
            f"{max(np.rad2deg(r['angles'][a]['peak_rad']) for r in values):.2f}°" for a in ('pitch','left_leg','right_leg')]+[
            f"{max(r['wheel_saturation'] for r in values)*100:.2f}% / {max(r['hip_saturation'] for r in values)*100:.2f}%",
            f"{max(r['fallback'] for r in values)*100:.3f}%",
            f"{max((s['stop_s'] for s in stops if s['stop_s'] is not None),default=0):.3f} s；{sum(s['stop_s'] is None for s in stops)} 次未观测到",
            f"{max(r['speed_rms'] for r in values if r['case']['height']>=.24):.4f} m/s"]
    a,b=aggregate('baseline'),aggregate(best)
    labels=['通过姿态/振荡/有限性','pitch 全程峰值','左腿全程峰值','右腿全程峰值','最差轮/髋矩饱和占比','最差回退占比','已观测停稳的最慢值 / 未观测数','中腿最差速度 RMS']
    text=['# 当前 Q/R 基线 PPO 稳定性搜索 — 2026-11-19','',
          f"结论：{'存在通过闭环的候选，仍需构建门槛' if eligible else '本轮无合格候选，保留当前生产基线'}。最接近参数为 `{best}`。未通过参数不可作为可上车固件。",'',
          '| 指标 | 当前基线 | 最接近候选 |','|---|---:|---:|']
    text += [f'| {label} | {x} | {y} |' for label,x,y in zip(labels,a,b)]
    text += ['', '峰值按全程绝对角统计，包含提前终止前记录；提前终止会截短峰值和停稳观察，不能据较小峰值认定更安全。饱和与回退是每工况占比的最大值。',
             '', '## 硬门槛及失败原因','',
             'pitch 10°、左腿 30°、右腿 30°，数值公差均 0.002 rad。任何一轴超限、持续振荡或提前终止均硬淘汰；基线没有豁免。',
             '正常门槛依据较稳实车片段的 pitch 4.58°、双腿 25.21°及行驶腿角 28.07°确定，属于本轮工程验收目标，不是已证明的物理安全边界。',
             '固件倒地边界：pitch/roll 1 rad、任一腿 0.8 rad，连续 200 ms；仿真故障边界：任一姿态 0.6 rad、绝对高度小于 0.12 或大于 0.42 m，以及非有限状态。仿真在更早的故障边界终止，并未模拟自救动作本身。',
             '持续振荡：最后零指令段至少 1.2 s，最后 0.6 s 角度峰峰值超过 3°、不低于前 0.6 s 的 85%，且角速度至少两次换向；三轴分别检查。该定义是本轮预先固定的检测器。','']
    for name in names:
        text.append(f"- `{name}`：{sum(r['passed'] for r in by[name].values())}/19 通过；"+'；'.join(f"{k}: {','.join(r['failures'])}" for k,r in sorted(by[name].items()) if not r['passed']))
    text += ['', '最接近候选的具体淘汰记录：','', '| 工况 | pitch 峰值 | pitch 超限时长 | 左/右腿峰值 | 其他故障 |','|---|---:|---:|---:|---|']
    for key,r in sorted(by[best].items()):
        if r['passed']:continue
        angles=r['angles']
        text.append(f"| {key} | {np.rad2deg(angles['pitch']['peak_rad']):.2f}° | {angles['pitch']['over_duration_s']:.3f}s | {np.rad2deg(angles['left_leg']['peak_rad']):.2f}° / {np.rad2deg(angles['right_leg']['peak_rad']):.2f}° | {'提前终止，详见轨迹' if 'nonfinite_or_early_termination' in r['failures'] else '无提前终止'} |")
    text += ['', '在独立高阻尼反向切换 `independent_02` 中，候选于 1.569 s 触及 0.42 m 高度故障边界，尚未等到 2.2 s 参考归零；该次停稳观察被截断。基线在 1.380 s 先触及 0.6 rad 姿态边界。两者左右轮均有 2 次反向饱和切换。此处是模型边界失败，不是已观察到的实车倒地。',
             '对照轨迹见 `plots/train_01.png`、`plots/independent_01.png`、`plots/independent_02.png`；饱和与故障边界证据见 `saturation_failure_diagnostics.json`。完整参数见 `closest_candidate.json`（eligible=false）。']
    text += ['', '## 中腿逐工况回归','', '| 工况 | 基线速度 RMS | 候选速度 RMS | 增量 | 基线/候选首次停稳 |','|---|---:|---:|---:|---|']
    def stopstr(r):
        if not r['stops']:return '驻车扰动，无松杆事件'
        s=r['stops'][-1]
        return '未观测到' if s['stop_s'] is None else f"{s['stop_s']:.3f}s"+('（反弹）' if s['rebound'] else '')
    for key,r in sorted(by[best].items()):
        if r['case']['height']<.24:continue
        base=by['baseline'][key]
        text.append(f"| {key} | {base['speed_rms']:.5f} | {r['speed_rms']:.5f} | {r['speed_rms']-base['speed_rms']:+.5f} | {stopstr(base)} / {stopstr(r)} |")
    text += ['', '## 方法、证据与局限','',
             '- 首先运行当前基线。固定种子 19/59/117，每种子 192 次，共 576 次 PPO 采样；对数倍率范围 1/2.25–2.25，七组为速度、双腿角、双腿角速度、pitch、pitch 角速度、双轮 R、双髋 R。左右保持对称。每次完整 Q/R、动作、分工况指标和失败原因在 `ppo_seed*.json`。',
             '- 训练 6 工况、选优 6 工况，冻结前三名后再做 7 个独立工况；PPO 不使用独立工况奖励。额外 16 次未搜索轴灵敏度检查，最好约 5% 初筛代价改善且失败数不减，因此未扩维。',
             '- 快速模型为限幅线性 LQR，只作筛选，不包含 MPC 求解回退；最终使用约束 TinyMPC14、逐工作点 Riccati、非线性 RK4。19 工况包含低/中腿、阶跃起步、匀速、高速倒车归零、140 ms 短反向再归零、正反切换及停车后冲击。',
             '- 最终 `nonlinear_final_1ms` 为 1 ms 执行/积分、2 ms 预测节点、4 ms 后台求解。保留原 2 ms 近似的 76 次记录，不混入最终结果。1 ms 版本同步实际输出变化率，附加载荷进入 m_b/M_SUP/M_V/M_LOAD。',
             '- 固件关闭速度 S 曲线限制，试验采用参考阶跃；锁点速度 0.10 m/s、腿角 0.15 rad，首次锁点权重直接 0.5，腿角异常后以 200 ms 爬升恢复。轮矩 ±16384/3326.29 N·m，净髋矩 ±40 N·m；保留原求解迭代预算、回退和 LPV 调度更新条件，Riccati 替换用于精确终端验证，并非生成表近似。',
             '- 停稳从实际参考归零计时，速度连续 0.20 s 在 ±0.06 m/s 内；另记反弹、驻车权重及漂移。新指令和提前终止会截断观察；归零前即终止的记录明确标注。',
             '- 阻尼 0–5.5，独立压力 7.5 N·m·s/rad；依据历史直接/投影模型的不同拟合范围。4–24 ms 为执行延迟不确定性，不能当作录波的 0–1 ms 反馈年龄。0–0.30 kg 附加载荷是工程压力范围，非辨识结论。',
             '- 两份实车录波仅用于失败机理和参数范围证据；未拿其拟合后宣称独立实车通过。布局 5402 不是唯一固件版本号，沿用命名归档的历史关联；详见 `capture_analysis.json` 的缺帧、转向、松杆前腿长/车速和截断记录。中腿缺少匹配实车 A/B，所有中腿比较仅为仿真结论。',
             '- 模型仍不包含真实轮地接触/打滑、完整电机电流动态、传感估计误差、实车自救状态机和 MCU 时间开销。没有通过姿态硬门槛，不能进入构建或宣称实车可用。',
             '', '## 构建与基线保留','',
             '本轮未运行 fast、主机 C 对拍、完整构建、ARM 编译或生成测试 HEX，原因是全部候选已被前置闭环硬门槛淘汰；不是构建通过，也没有放宽终端 P 离网格 3% 门槛。',
             '生产 MATLAB 配置及六份 C/H 源 SHA256 与开跑前一致，见 `production_protection.json`。原基线及完整源快照在 `baseline.m.txt`、`baseline.json`、`src/`。本轮未切换、烧录，因此无需回刷；已有板端 HEX 与当前源码的匹配关系未在本轮证明，不能将旧 HEX 冒充当前基线镜像。',
             '', '## 复跑','', '在工程根目录执行（同目录已有结果会续跑；完整重新计算使用下面的新目录方式）：','', '```powershell',
             'python -B Matlab/mixmpc_pre/pso/prepare_stability_ppo.py',
             'python -B Matlab/mixmpc_pre/pso/run_stability_ppo.py --baseline-only',
             'python -B Matlab/mixmpc_pre/pso/run_stability_ppo.py --steps 192',
             'python -B Matlab/mixmpc_pre/pso/run_stability_ppo.py --sensitivity',
             'python -B Matlab/mixmpc_pre/pso/verify_stability_ppo.py --fidelity',
             'python -B Matlab/mixmpc_pre/pso/analyze_stability_captures.py',
             'python -B Matlab/mixmpc_pre/pso/report_stability_ppo.py', '```','',
             '冻结源码重跑到新目录，避免覆盖本轮证据：','', '```powershell',
             "$frozen = (Resolve-Path 'Matlab/mixmpc_pre/pso/build/ppo_stability_20261119').Path",
             "$env:TINY_STABILITY_OUT = Join-Path (Split-Path $frozen) 'ppo_stability_20261119_rerun'",
             'python -B "$frozen/harness/prepare_stability_ppo.py" --from-snapshot "$frozen"',
             'python -B "$frozen/harness/run_stability_ppo.py" --baseline-only',
             'python -B "$frozen/harness/run_stability_ppo.py" --steps 192',
             'python -B "$frozen/harness/verify_stability_ppo.py" --fidelity',
             '```','',
             '所有逐工况指标在 `case_comparison.csv`；原始逐点状态/参考/力矩/权重/回退在 `nonlinear_final_1ms/*__*.json`，三轴松杆后峰值、峰值时刻、超限时长、恢复时间在 `summary.json`。']
    (OUT/'REPORT.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    print('closest',best,'eligible',eligible,'production unchanged',unchanged)


if __name__=='__main__':main()
