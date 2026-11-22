"""Render the completed Q/R experiment and case-by-case audit."""
from pathlib import Path
import math
import os
import re

from run_pso import HERE, digest, read_json

DEG = 180 / math.pi


def render(run, target):
    rel = Path(os.path.relpath(run, target.parent)).as_posix()

    def link(label, path):
        return f"[{label}]({rel}/{path})"

    result = read_json(run / "validation.json")
    build = read_json(run / "baseline_build.json")
    base = read_json(run / "candidates/baseline.json")
    base_test = read_json(run / "candidates/baseline_holdout.json")
    ranked = read_json(run / "ranking.json")
    selected = result["selected"]
    nearest = selected or (result["results"][0]["id"] if result["results"] else None)
    best = read_json(run / f"candidates/{nearest}.json") if nearest else base
    test = read_json(run / f"candidates/{nearest}_holdout.json") if nearest else base_test
    points = [read_json(p) for p in (run / "candidates").glob("g??_p??.json")]
    out = ["# TinyMPC14 Q/R 搜索结果", ""]
    if selected:
        out.append(f"**{selected} 通过仿真筛选、原有门限和 STM32H723 完整链接。** "
                   f"搜索集综合分由 {base['score']:.3f} 降至 {best['score']:.3f}，"
                   f"留出集由 {base_test['score']:.3f} 降至 {test['score']:.3f}。"
                   "实车证据来自早期固件，用于约束执行链模型。当前基线和候选的实车效果仍待 A/B。")
    else:
        out.append(f"**保留当前 Q/R。** 本次范围内没有候选同时改善搜索集、留出集并通过部署门限。"
                   f"报告对比 {nearest or '当前基线'}。")
    out += ["", "## 完整配置", "",
            "状态顺序为 `s ds yaw dyaw ψL dψL ψR dψR pitch dpitch roll droll H-H_REF dH`。"
            "输入顺序为 `Twl Twr TBl TBr FL FR`。", ""]
    for r in (base, best) if nearest else (base,):
        out += [f"`{r['id']}`", "", "```matlab"]
        for k in ("Q_DIAG", "R_DIAG"):
            out.append(f"{k} = [" + ", ".join(format(x, ".17g") for x in r[k]) + "];")
        out += ["```", ""]
    out += ["## 搜索覆盖", "",
            f"实际评估 {len(points)} 个粒子位置（含基线、重复位置及失败），"
            f"{sum(r['feasible'] for r in points)} 个闭环评估完毕，"
            f"{len(ranked)} 组不同的可仿真非基线参数进入排名。评分越低越好。", "",
            "| 粒子群 | 随机种子 | 粒子 × 轮数 | 本群最低分 |",
            "|---|---:|---:|---:|"]
    for name, file in [("基线附近", "progress.json"), ("分层重启", "progress_restart.json"),
                       ("历史权重扩展", "progress_archive.json")]:
        if (run / file).is_file():
            p = read_json(run / file)
            out.append(f"| {name} | {p['seed']:.0f} | {p['particles']} × {p['generation']} | {p['best_score']:.3f} |")
    if ranked:
        out += ["", f"全部粒子中最低仿真分为 {ranked[0]['id']} 的 {ranked[0]['score']:.3f}。"
                "有限采样无法证明全局最优。",
                "", " · ".join(link(label, path) for label, path in
                    [("排名", "ranking.json"), ("逐点记录", "candidates/"), ("运行日志", "logs/")])]
    out += ["", "## 逐工况闭环结果", "",
            f"每格依次给出基线 → {best['id']}。姿态为俯仰、横滚、左右矢状腿角误差中的最大值。"
            "横滚栏为绝对峰值。", "",
            "| 工况 | 综合分 | 姿态 / 俯仰 / 横滚 ° | 高度 RMS / 尾段误差 mm | 纵向 / 偏航速度 RMS (m/s, rad/s) | 回退 / 触箱 / 触速率 % |",
            "|---|---:|---:|---:|---:|---:|"]
    pairs = list(zip(base["scenarios"], best["scenarios"], strict=True))
    if test["feasible"]:
        pairs += list(zip(base_test["scenarios"], test["scenarios"], strict=True))

    def pair(a, b, fields, factors, places):
        return " → ".join("/".join(f"{m[name]*factor:.{p}f}" for name, factor, p in
                          zip(fields, factors, places, strict=True)) for m in (a, b))

    for a, b in pairs:
        cells = [a["name"], f"{a['score']:.2f} → {b['score']:.2f}",
                 pair(a, b, ("attitude_peak", "pitch_peak_error", "roll_peak"), (DEG,)*3, (2,)*3),
                 pair(a, b, ("height_rms", "height_late_error"), (1000,)*2, (1,)*2),
                 pair(a, b, ("speed_rms", "yaw_rate_rms"), (1,)*2, (3,)*2),
                 pair(a, b, ("fallback", "box", "slew"), (100,)*3, (1,)*3)]
        out.append("| " + " | ".join(cells) + " |")
    out += ["", "升降延迟按参考与响应达到 10% 行程的时间差计算。高度误差进入 ±8 mm 后须持续到仿真窗末。"
            "横滚恢复同时要求误差小于 0.015 rad、滚速小于 0.12 rad/s。"
            "时间接近剩余仿真窗表示窗内未恢复。定高工况的高度延迟和超调记零。"
            "腿力列依次为仿真请求、模型接纳和模型已建立力。", "",
            "| 工况 | 高度延迟 / 稳定 / 横滚恢复 s | 高度超调 mm | 合力均值 N | 差力 RMS N |",
            "|---|---:|---:|---:|---:|"]
    for a, b in pairs:
        times = pair(a, b, ("height_delay_s", "height_settle_s", "roll_recovery_s"), (1,)*3, (3,)*3)
        over = pair(a, b, ("height_overshoot_m",), (1000,), (1,))

        def force(key):
            return " → ".join("/".join(f"{x:.1f}" for x in m[key]) for m in (a, b))

        out.append(f"| {a['name']} | {times} | {over} | {force('sum_force_mean')} | {force('diff_force_rms')} |")
    out += ["", "## 部署验证", "",
            "| 配置 | 搜索集通过 | 留出集通过 | fast | 固件 C 独立闭环 | full | AC5 / HEX |",
            "|---|---|---|---|---|---|---|"]

    def gate(r, key):
        if key not in r:
            return "未进入"
        suffix = "PASS" if r[key] == 0 else "FAIL"
        return link(suffix, f"logs/{r['id']}_{key}.log")

    out.append("| baseline | 基准 | 基准 | " + " | ".join(
        gate(dict(build, id="baseline"), k) for k in ("fast", "closed", "full", "arm")) + " |")
    for r in result["results"]:
        out.append(f"| {r['id']} | {'是' if r['search_improved'] else '否'} | "
                   f"{'是' if r['holdout_improved'] else '否'} | " + " | ".join(
                       gate(r, k) for k in ("fast", "closed", "full", "arm")) + " |")
    closed = run / "logs/baseline_closed.log"
    if closed.is_file() and b"peak=8.640 deg" in closed.read_bytes():
        out += ["", "本次基线 C 回归在 I=1.371 kg·m²、2 ms 传输 / 12 ms 一阶执行、16 ms 求解节奏工况"
                "测得横滚峰值 8.640°，超过原门限 0.120 rad（6.875°）。"]
    if selected:
        selected_log = run / "logs" / f"{selected}_closed.log"
        peaks = re.findall(rb"peak=([0-9.]+) deg", selected_log.read_bytes())
        if peaks:
            out += ["", f"{selected} 的原固件独立闭环通过 {len(peaks)} 个工况，"
                    f"最差横滚峰值 {max(float(x) for x in peaks):.3f}°。"]
    out += ["", "| 配置 | fast 离网格 P 最大相对误差 | fast K 最大相对误差 | 最差闭环谱半径 |",
            "|---|---:|---:|---:|"]
    for r in (dict(build, id="baseline"), *result["results"]):
        name = r["id"]
        p = run / "logs" / f"{name}_fast.log"
        if not p.is_file():
            continue
        m = re.findall(rb"Terminal rank \d+: P error ([0-9.e+-]+), K error ([0-9.e+-]+), min eig [0-9.e+-]+, radius ([0-9.e+-]+)", p.read_bytes())
        if m:
            v = [float(x) for x in m[-1]]
            out.append(f"| {name} | {v[0]:.4%} | {v[1]:.4%} | {v[2]:.9f} |")
    out += ["", "终端 P、K 门限均为 3%，且要求正定、谱半径小于 1。"
            "fast 包含调度与求解器主机对拍，full 另包含 S8/S10 及仓库门限。"
            "这次每侧轴力 R 搜索到 1.0。历史每侧 R=5 触发约 9.69% 的 P 插值误差，超出本次范围。",
            "", "## 实车证据", ""]
    telemetry = read_json(run / "vofa.json")
    c, h = telemetry["calibration"], telemetry["heldout"]
    if c and h:
        out += [f"02:24 低目标窗 {c['board_window_s'][0]:.3f}–{c['board_window_s'][1]:.3f} s 的 "
                f"{c['frequency_hz']:.2f} Hz 振荡中，反馈估计差力相对预测拍 CAN 接纳差力 "
                f"{c['feedback_minus_can_deg']:.1f}°，相位等效延迟 {c['apparent_transport_plus_response_ms']:.2f} ms。"
                f"固定传输 {c['assumed_transport_ms']:.0f} ms 得一阶时间常数 {c['estimated_tau_ms']:.2f} ms。"
                f"高位留出窗预测相位 {h['predicted_deg']:.2f}°，实测 {h['feedback_minus_can_deg']:.2f}°。"
                f"幅值比预测 {h['predicted_gain']:.3f}，实测 {h['feedback_to_can_gain']:.3f}。"
                f"两段原始到控制滚速的相位等效延迟为 {c['roll_filter_effective_delay_ms']:.2f}、"
                f"{h['roll_filter_effective_delay_ms']:.2f} ms。高度按源码 19 Hz 陷波和 10 ms 速度低通半衰期建模。"
                "窄带相位无法分开辨识传输、响应、采样和力反算估计。", ""]
    for name, w in telemetry.get("plant_replay", {}).items():
        if w["windows"]:
            out += [f"{name} 用实机接纳力驱动升降/横滚模型，{w['windows']} 个 "
                    f"{w['horizon_s']:.2f} s 窗的末端高度、横滚 RMSE 为 "
                    f"{1000*w['height_rmse_m']:.1f} mm、{DEG*w['roll_rmse_rad']:.2f}°。", ""]
    out += ["回放未能复现原机的小窗横滚。版本差异、稀疏采样、未测轮地反力和回放省略的轮矩路径均会影响结果。"
            "搜索使用实测相位支持的延迟带、单位静态增益和力箱，并覆盖不同惯量、载荷与求解节奏。", "",
            "| 录波 | 有效帧 | 全程回退 % | 低目标高度均值范围 m | 低位横滚 RMS 范围 ° |",
            "|---|---:|---:|---:|---:|"]
    for name, data in telemetry["captures"].items():
        low = [w for w in data["plateaus"] if w["reference_m"] == .17]
        heights = [w["height_mean_m"] for w in low]
        rolls = [DEG*w["roll_rms_rad"] for w in low]
        hspan = f"{min(heights):.3f}–{max(heights):.3f}" if heights else "无"
        rspan = f"{min(rolls):.2f}–{max(rolls):.2f}" if rolls else "无"
        out.append(f"| {name} | {data['valid_rows']} | {100*data['fallback_fraction']:.2f} | {hspan} | {rspan} |")
    out += ["", "历史录波中低目标未降到 0.17 m。02:24 的 CH40/43 用各自的 CAN 时间对齐并去重。"
            "逐平台单侧触底比例、高度延迟、超调、未恢复窗口，以及请求、接纳、反馈估计的合力与差力见结构化结果。"
            "CSV 只有两位小数，且缺少独立滚转扰动输入。 "
            + link("VOFA 对齐结果与原始文件哈希", "vofa.json"), "",
            "50 通道录波为结构修复前的发布版。历史 AC5 快照的轮矩 R=6.5、横滚速度 Q=100、轴力 R=0.01，"
            "当前工作树基线对应值为 5.5、1、0.05。未找到同版本的 54 通道录波。"
            "实车证据支持延迟和约束的仿真取值，候选相对当前基线的改善须由同版本 A/B 确认。",
            "", "## 交付", ""]
    if selected:
        if (run / "TinyMPC14_AB.zip").is_file():
            out.append("- " + link("A/B 压缩包", "TinyMPC14_AB.zip") + "，包含两套 HEX、配置、生成包和构建清单。")
        for pkg, label in (("baseline", "A 基线"), (selected, "B 候选")):
            path = f"packages/{pkg}/arm/Wheel_Leg_Code.hex"
            out.append(f"- {link(label+' HEX', path)} · {link('六文件包', f'packages/{pkg}/full/')}。"
                       f" HEX SHA-256 `{digest(run/path)}`。")
    else:
        out.append("- 基线固件和候选已通过的阶段保存在 " + link("packages", "packages/") + "。")
    out += ["- [复跑说明](README.md) · [A/B VOFA 通道与对齐口径](VOFA.md)。", ""]
    target.write_text("\n".join(out), encoding="utf-8")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    opts = parser.parse_args()
    render(opts.run.resolve(), HERE / "REPORT.md")

