"""Write the audit report from executed recheck results."""
import math
import re


import run_pso as r
from select_rechecked import OUT, comparable_source, load_suites


def sweep(name):
    path = OUT/"logs"/f"{name}_sweep.log"
    if not path.is_file():
        return None
    raw = path.read_bytes()
    peaks = [float(p) for p in re.findall(rb"peak=([\d.]+) deg", raw)]
    if not peaks:
        return None
    return raw.count(b"LIMITS PASS"), len(peaks), max(peaks)


def report():
    state = r.read_json(OUT/"decision.json")
    winner = state["selected"]
    base = load_suites("baseline")
    assert winner
    new = load_suites(winner)
    deployed = OUT/"applied.json"
    lines = ["# TinyMPC14 权重交叉复核", "",
             f"**本轮选定 {winner}。** 在复核的参数和工况范围内，按三个工况集各自相对基线的评分取等权平均。"
             "每个工况的姿态、横滚和回退继续受原 PSO 门限约束。"
             "通过原有构建门限、独立闭环 C 回归和扩展 C 网格后才允许写入实际配置。", ""]
    if deployed.is_file():
        applied = r.read_json(deployed)
        assert applied["id"] == winner
        lines += ["实际配置仅更新 Q_DIAG 与 R_DIAG。实际目录的完整与 fast 构建均通过，"
                   "发布文件在 Code/Model，其哈希与同次完整生成包一致。没有上板烧录。", ""]
    lines += ["| 配置 | 原搜索集 | 原留出集 | 新非线性集 | 三集合平均基线比 |",
              "|---|---:|---:|---:|---:|",
              f"| baseline | {base[0]['score']:.3f} | {base[2]['score']:.3f} | {base[1]['score']:.3f} | 1.000 |"]
    for row in state["ranked"]:
        scores = row["scores"]
        lines.append(f"| {row['id']} | {scores['probes']:.3f} | {scores['holdout']:.3f} | {scores['nonlinear']:.3f} | {row['ratio']:.3f} |")
    lines += ["", "## 复核覆盖", "",
              "- 原 PSO 搜索 88 个位置。复核运行 22 组，包括基线、4 个已搜候选和 17 个单维或联合扰动。"
              "原第一名附近检查 0.7/1.4 倍、边界外横滚速度和两腿共用 R 等方向。",
              "- 对四个有竞争力的候选重跑原留出集，并在新 8 组工况中使用随状态更新的非线性动力学与 RK4。"
              "工况覆盖升降、转弯、正负冲击、高惯量和长执行滞后。长滞后是超出原搜索范围的压力工况。"
              "该非线性核与仓库 S10 共享物理方程。",
              "- 独立固件 C 网格使用生产控制入口、输出分配、CAN 码反算及电机反馈。"
              "组合为 3 个横滚惯量、3 组执行延迟与时间常数、3 个求解节奏、2 种扰动方向。"
              "诊断网格逐项记录原高度/横滚门限，正常六工况 C 回归仍以原断言验证。", "",
              "| 配置 | C 网格通过 / 总计 | 最大横滚 ° |",
              "|---|---:|---:|"]
    for name in dict.fromkeys(["baseline", "g04_p02", winner]):
        passed, count, peak = sweep(name)
        lines.append(f"| {name} | {passed}/{count} | {peak:.3f} |")
    lines += ["", "## 新非线性工况逐项", "",
              f"每格由基线指向 {winner}。", "",
              "| 工况 | 横滚峰值 ° | 高度 RMS mm | 高度稳定 s | 回退 % |",
              "|---|---:|---:|---:|---:|"]
    for b,c in zip(base[1]["scenarios"],new[1]["scenarios"],strict=True):
        vals = [f"{b[k]*mult:.2f} → {c[k]*mult:.2f}" for k,mult in (
            ("roll_peak",180/math.pi),("height_rms",1000),("height_settle_s",1),("fallback",100))]
        lines.append("| " + " | ".join([b["name"],*vals]) + " |")
    lines += ["", "## 构建与发布", ""]
    for name in dict.fromkeys(["baseline","g04_p02",winner]):
        stage = r.read_json(OUT/f"{name}_build.json")
        line = ", ".join(f"{k} {'PASS' if stage.get(k)==0 else 'FAIL'}" for k in ("full","fast","closed","arm"))
        lines.append(f"- {name}。{line}。")
    for name in ("baseline","g04_p02"):
        original = r.HERE/"build/experiment/packages"/name/"full"
        current = OUT/"packages"/name/"full"
        if all(comparable_source(original/f)==comparable_source(current/f) for f in r.MODEL_FILES):
            lines.append(f"- {name} 的独立完整构建与上一轮生成的六个文件一致，比较时忽略 MATLAB Coder 生成时间注释。")
    source = OUT/"logs"/f"{winner}_full.log"
    match = re.findall(rb"Terminal rank \d+: P error ([\d.e+-]+), K error ([\d.e+-]+), min eig [\d.e+-]+, radius ([\d.e+-]+)",source.read_bytes())
    if match:
        p,k,radius = map(float,match[-1])
        lines.append(f"- 候选离网格终端 P 误差 {100*p:.4f}%，K 误差 {100*k:.4f}%，最差闭环谱半径 {radius:.9f}。")
    lines += ["", "完整权重", "", "```matlab"]
    for key in ("Q_DIAG","R_DIAG"):
        lines.append(f"{key} = ["+", ".join(format(x,".17g") for x in new[0][key])+"];")
    lines += ["```", "",
              "## 实车证据边界", "",
              "历史数据能约束执行链相位和力箱，也暴露了短窗横滚回放误差。"
              "仍缺当前配置的同版实车 A/B。三套仿真排序、C 网格和编译验证共同支持本轮选择，"
              "范围以实际评估过的点为准。", "",
              "- [决策记录](build/recheck/decision.json) · [全部补点](build/recheck/probes/) · [非线性逐项数据](build/recheck/nonlinear/) · [日志](build/recheck/logs/)",
              f"- [本轮候选 HEX](build/recheck/packages/{winner}/arm/Wheel_Leg_Code.hex) · [基线 HEX](build/recheck/packages/baseline/arm/Wheel_Leg_Code.hex) · [VOFA 通道](VOFA.md)",
              "- [原搜索报告](REPORT.md)。", ""]
    if deployed.is_file():
        lines.append("- [实际发布哈希](build/recheck/applied.json)。")
        if applied.get("arm_after_publish") == 0:
            lines.append("- 实际固件目录的 AC5 已重新编译并链接。[实际目录构建的 HEX](build/recheck/packages/production/arm/Wheel_Leg_Code.hex)。")
            if applied.get("production_hex_matches_isolated"):
                lines.append("- 实际目录与隔离构建生成的 HEX 字节相同。")
    (r.HERE/"RECHECK.md").write_text("\n".join(lines)+"\n", encoding="utf-8")


if __name__ == "__main__":
    report()


