"""Quantify old failures and one-variable nonlinear counterfactuals."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from local_pitch_ppo import OUT, files, read_metrics, score


def main():
    mechanism=read_metrics(OUT/'mechanism')
    sensitivity=read_metrics(OUT/'sensitivity')
    ranking=[]
    for name in sorted({r['candidate'] for r in sensitivity}):
        rows=[r for r in sensitivity if r['candidate']==name]
        ranking.append(dict(id=name,score=score(rows),failed=sum(not r['passed'] for r in rows),
                            pitch_deg=max(np.rad2deg(r['angles']['pitch']['peak_rad']) for r in rows),
                            height_max_m=max(r['height_max_m'] for r in rows)))
    ranking.sort(key=lambda r:r['score'])
    files.write_json(OUT/'sensitivity_rank.json',ranking)
    contributions=[]
    plot=OUT/'plots';plot.mkdir(exist_ok=True)
    for case in ('train_02','selection_02','independent_01','independent_02'):
        raw=files.read_json(OUT/'mechanism'/f's117_0185__{case}.json')['metrics']['raw']
        x=np.array(raw['x']);u=np.array(raw['applied']);t=np.array(raw['t'])
        arm=1/(np.cos(x[:,10,None])*np.cos(x[:,[4,6]]))
        wheel=np.sum(arm*u[:,:2]*np.sin(x[:,[4,6]])/.0525,axis=1)
        axial=np.sum(arm*u[:,4:6],axis=1)
        window=(t>=(.2 if case=='independent_01' else 1.2))
        k=int(np.argmax(abs(x[:,8])));h=int(np.argmax(x[:,12]))
        contributions.append(dict(case=case,pitch_peak_time_s=float(t[k]),height_peak_time_s=float(t[h]),
            vertical_rate_at_pitch_peak_m_s=float(x[k,13]),
            wheel_heave_peak_after_trigger_N=float(wheel[window].max()),
            wheel_heave_at_height_peak_N=float(wheel[h]),axial_heave_at_height_peak_N=float(axial[h]),
            height_peak_m=float(.27+x[h,12]),end_s=float(t[-1]),
            reconstruction='Delayed inputs and endpoint state, 1ms sampling; approximate decomposition, not exact RK substage integral'))
        fig,ax=plt.subplots(4,1,figsize=(9,9),sharex=True)
        ax[0].plot(t,np.rad2deg(x[:,8]));ax[0].axhline(10,color='r',ls='--');ax[0].axhline(-10,color='r',ls='--')
        ax[1].plot(t,.27+x[:,12]);ax[1].axhline(.42,color='r',ls='--')
        ax[2].plot(t,wheel,label='wheel heave');ax[2].plot(t,axial,label='axial heave');ax[2].legend()
        ax[3].plot(t,u[:,4],label='left axial input');ax[3].plot(t,u[:,5],ls=':',label='right axial input');ax[3].legend()
        for a,label in zip(ax,('Pitch (deg)','Height (m)','Generalized force (N)','Delayed axial input (N)')):a.set_ylabel(label);a.grid(alpha=.25)
        ax[-1].set_xlabel('Time (s)');fig.suptitle(case+' | s117_0185');fig.tight_layout()
        fig.savefig(plot/(case+'_mechanism.png'),dpi=140);plt.close(fig)
    files.write_json(OUT/'mechanism_contributions.json',contributions)
    text=['# s117_0185 失败机理与单变量对照','',
          '旧留出工况在本轮全部属于开发数据；不得再称为本轮独立验证。所有对照只改标注的一项，控制器和验收门槛保持一致。','',
          '| 工况 / 单变量 | pitch 峰值 ° | 高度峰值 m | 结束 s | 失败项 |','|---|---:|---:|---:|---|']
    for r in sorted(mechanism,key=lambda r:r['case']['id']):
        text.append(f"| {r['case']['id']} | {np.rad2deg(r['angles']['pitch']['peak_rad']):.3f} | {r['height_max_m']:.4f} | {r['end_s']:.3f} | {', '.join(r['failures']) or '无'} |")
    text += ['', '`damping` 对照改为 2.1 N·m·s/rad，`delay_s` 改为 4 ms，`load` 改为 0 kg。提前终止的较小峰值不可当成改善。',
             '', '动力学合同：高度广义力包含轴力的 sec(roll)sec(leg) 投影，以及轮矩 × sin(leg)/轮半径的同方向投影；`F_pitch=0`，不能将轴力直接作用于 pitch 的已关闭项归因为本次 pitch 超限。反向切换的轮矩/腿角同号区间会产生向上的轮矩投影。轴力降至下限后的高度继续增加还受此前积累的向上速度影响。',
             '力分解采用记录的延迟后输入与每毫秒末端状态近似重算，见 `mechanism_contributions.json` 和四张机理图；阻尼、延迟、载荷的因果影响以上表单变量对照为准，不能由单条轨迹相关性下定论。',
             '', '## 局部单轴灵敏度','', '| 倍率点 | 训练代价 | 失败数 | 最坏 pitch ° | 最高高度 m |','|---|---:|---:|---:|---:|']
    for r in ranking:text.append(f"| {r['id']} | {r['score']:.2f} | {r['failed']} | {r['pitch_deg']:.3f} | {r['height_max_m']:.4f} |")
    (OUT/'MECHANISM.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    print('mechanism and sensitivity report saved; top',ranking[:3])


if __name__=='__main__':main()
