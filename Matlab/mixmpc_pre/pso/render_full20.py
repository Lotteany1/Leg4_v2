"""Render the full-diagonal tuning evidence from the executed run."""
import argparse
from pathlib import Path
import math
import re

import run_pso as r
from verify_full20 import response_times, response_ratio

LABELS = ("s","ds","yaw","dyaw","ψL","dψL","ψR","dψR","pitch","dpitch","roll","droll","H−H_REF","dH",
          "Twl","Twr","TBl","TBr","FL","FR")


def render(out):
    decision=r.read_json(out/"validation.json")
    chosen=decision["selected"]
    installed=(out/"applied.json").is_file()
    evaluated=[r.read_json(path)["log_multipliers"] for path in (out/"candidates").glob("*.json")]
    baseline=r.read_json(out/"candidates/baseline.json")
    heldbase=r.read_json(out/"holdout/baseline.json")
    rows=decision["results"]
    compared=chosen or next((row["id"] for row in rows if row["holdout_improved"]),rows[0]["id"] if rows else None)
    point=r.read_json(out/"candidates"/f"{compared}.json") if compared else baseline
    held=r.read_json(out/"holdout"/f"{compared}.json") if compared else heldbase
    lines=["# TinyMPC14 全参数优化", ""]
    if chosen:
        lines += [f"**{chosen}** 是本次实际评估、通过稳定性与固件门限的最低综合排名候选。",
                  "当前配置已同步生成 C/H。" if installed else "独立构建验证已完成。",""]
    else:
        lines += ["**保留本轮开始时的配置。** 已评估候选中没有同时改善响应、保持稳定并通过部署验证的参数。",""]
        if compared:
            lines += [f"以下对比为 {compared}，该候选未达到发布门限。",""]
    lines += [
        f"已完成 {decision['search_positions']} 个候选位置，{decision['search_eligible']} 组通过训练集稳定性及响应筛选，"
        f"{sum(x['holdout_improved'] for x in decision['results'])} 组也通过留出筛选。",
        "搜索包含 40 个单坐标对照、两个独立分层初始化粒子群及一次响应导向粒子群。",
        "状态 14 项和输入 6 项独立可变。相对本轮基线，Q 范围 0.25–4 倍，轮髋 R 为 0.30–3 倍，"
        "左右轴力 R 各为 0.45–5 倍。左/右镜像工况共享原实车模型，评分对过度左右不对称另计小额代价。",
        "",
        "停车场景以初始位置偏移刺激 s，跟随模式对控制器提供零位移误差，"
        "与固件按模式选择位置反馈一致。转向使用积分参考产生的偏航误差。"
        "响应按停车回到 ±15 mm 且速度小于 0.06 m/s、半幅起速延迟、停速至 ±0.06 m/s 等计时。",
        "",
        "硬越界和非有限状态直接淘汰。软姿态、横滚和回退恶化量按基线和容差归一后，对闭环分乘以 "
        "`(1+恶化量)^2`，硬失稳另乘 1000。"
        "原稳定性容差沿用基线峰值 ×1.05 加 0.003 rad，回退率最多多 5 个百分点。"
        "每个工况集要求三类计时平均相对基线更快，任一类别可容许 5% 加 20 ms 的单项波动。"
        "综合分最多比基线多 5%。排名给工况综合分 35%、时效 65%，训练与留出等权。",
        "",
        "| 工况集 | 综合分 | 驻车回位 s | 起速滞后 s | 停稳 s |",
        "|---|---:|---:|---:|---:|",
    ]
    for label,b,c in (("训练",baseline,point),("非线性留出",heldbase,held)):
        times=[f"{x:.3f} → {y:.3f}" for x,y in zip(response_times(b),response_times(c),strict=True)]
        lines.append(f"| {label} | {b['score']:.3f} → {c['score']:.3f} | "+" | ".join(times)+" |")
    lines += ["","## 权重","",
              "| 项 | 本轮基线 | 选定权重 | 倍率 | 实际采样倍率范围 |",
              "|---|---:|---:|---:|---:|"]
    for k,(name,b,c) in enumerate(zip(LABELS,baseline["Q_DIAG"]+baseline["R_DIAG"],
                                  point["Q_DIAG"]+point["R_DIAG"],strict=True)):
        observed=[values[k] for values in evaluated]
        lines.append(f"| {'Q' if k<14 else 'R'} {name} | {b:.10g} | {c:.10g} | {c/b:.5f} | "
                     f"{math.exp(min(observed)):.3f}–{math.exp(max(observed)):.3f} |")
    lines += ["","## 位移、速度和稳定性逐项","",
              "| 工况 | 位移 RMS mm | 速度 RMS m/s | 起速 / 停稳 / 回位 s | 姿态峰值 ° | 回退 % |",
              "|---|---:|---:|---:|---:|---:|"]
    for b,c in ((baseline,point),(heldbase,held)):
        for x,y in zip(b["scenarios"],c["scenarios"],strict=True):
            def item(key,mult=1):
                return f"{mult*x[key]:.3f} → {mult*y[key]:.3f}"
            timings=" / ".join(item(k) for k in ("speed_rise_lag_s","speed_stop_s","position_settle_s"))
            lines.append("| "+" | ".join([x["name"],item("position_rms",1000),item("speed_rms"),
                                           timings,item("attitude_peak",180/math.pi),item("fallback",100)])+" |")
    lines += ["","## 构建验证","",
              "| 候选 | 留出通过 | fast | C 六工况 | 扩展 C 网格 | full | AC5 |",
              "|---|---|---|---|---:|---|---|"]
    def status(row,k):
        return "未运行" if k not in row else ("通过" if row[k]==0 else "失败")
    for row in decision["results"]:
        n=row.get("sweep_passes")
        grid="未运行" if n is None else f"{n}/54"
        lines.append("| "+" | ".join([row["id"],"是" if row["holdout_improved"] else "否",
            status(row,"fast"),status(row,"closed"),grid,status(row,"full"),status(row,"arm")])+" |")
    if chosen:
        sweep=out/"logs"/f"{chosen}_sweep.log"
        hits=re.findall(rb"peak=([\d.]+) deg",sweep.read_bytes())
        old=r.HERE/"build/recheck"
        previous=old/"applied.json"
        if hits and previous.is_file() and r.read_json(previous)["config_sha256"]==r.digest(out/"baseline.m.txt"):
            old_log=old/"logs/Rforce_low_sweep.log"
            old_hits=re.findall(rb"peak=([\d.]+) deg",old_log.read_bytes())
            if len(old_hits)==len(hits)==54:
                lines += ["",f"已核对前一轮 C 网格对应同一基线配置。最差横滚峰值由 "
                                f"{max(map(float,old_hits)):.3f}° 变为 {max(map(float,hits)):.3f}°。"]
        elif hits:
            lines += ["",f"候选固件 C 网格最大横滚峰值 {max(map(float,hits)):.3f}°。"]
        terminal=out/"logs"/f"{chosen}_full.log"
        match=re.findall(rb"Terminal rank \d+: P error ([\d.e+-]+), K error ([\d.e+-]+), min eig [\d.e+-]+, radius ([\d.e+-]+)", terminal.read_bytes())
        if match:
            a,b,c=map(float,match[-1])
            lines.append(f"完整构建的终端 P、K 最大相对误差分别为 {100*a:.4f}%、{100*b:.4f}%，最差闭环谱半径为 {c:.9f}。")
    lines += ["","结论以本轮测试边界、评分和已评估位置为准。实车响应及姿态表现需同版本 A/B 验证。",
              "",f"- [候选及验证记录]({out.name}/validation.json) · [搜索日志]({out.name}/logs/search.log)",
              "- 复跑命令在仓库根目录。", "", "```powershell",
              "python Matlab/mixmpc_pre/pso/run_full20.py --run Matlab/mixmpc_pre/pso/build/full20_repeat",
              "```", ""]
    prefix=out.relative_to(r.HERE).as_posix()
    lines=[x.replace(f"]({out.name}/",f"]({prefix}/") for x in lines]
    if installed:
        lines += [f"- [实际目录 HEX]({prefix}/packages/production/arm/Wheel_Leg_Code.hex) · [发布哈希]({prefix}/applied.json)",""]
    (r.HERE/"FULL20.md").write_text("\n".join(lines),encoding="utf-8")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    args=parser.parse_args()
    render(args.run.resolve())





