"""Generate tasks 1/2 data, figures and a measured validation report."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from model.basic_model import forward_kinematics
from model.kinematics import (LENGTHS, POINTS, carrier_k1, carrier_k2, evaluate,
                              joint_motion, rotation)

CARRIERS = {"K1": carrier_k1, "K2": carrier_k2}
STEPS = (0.04, 0.02, 0.01, 0.005)
FIELDS = [f"{name}_{axis}" for name in ("p", "v", "a", "ae", "ar", "aC")
          for axis in ("x", "y")]


def values(result):
    return np.concatenate((result.state.position, result.state.velocity,
                           result.state.acceleration, result.ae, result.ar, result.aC))


def reference_positions(t, carrier):
    """Independent position-only path through the pre-existing geometry code."""
    _, _, end, c1, c2, c3 = forward_kinematics(joint_motion(t)[0], LENGTHS)
    f = carrier(t)
    return np.array([f.origin + rotation(f.angle) @ p for p in (end, c1, c2, c3)])


def export_motion(out, carrier_name, carrier, times):
    with (out / f"{carrier_name}_motion.csv").open("w", newline="") as f, \
            (out / f"{carrier_name}_stages.csv").open("w", newline="") as g:
        writer, stage_writer = csv.writer(f), csv.writer(g)
        writer.writerow(["time_s", "point", "frame", *FIELDS])
        stage_writer.writerow(["time_s", "point", "mapping", *FIELDS])
        for t in times:
            _, absolute, stages = evaluate(t, carrier)
            for point in POINTS:
                writer.writerow([t, point, "S0", *values(absolute[point])])
                for mapping, result in stages[point]:
                    stage_writer.writerow([t, point, mapping, *values(result)])


def validate(out, name, carrier, duration):
    # Common sample locations, at least max(h) away from either endpoint.
    times = np.arange(1, int(np.floor(duration/STEPS[0]))) * STEPS[0]
    center = np.array([reference_positions(t, carrier) for t in times])
    results = [evaluate(t, carrier)[1] for t in times]
    exact = np.array([[[r[p].state.velocity, r[p].state.acceleration]
                       for p in POINTS] for r in results])
    recursive_position = np.array([[r[p].state.position for p in POINTS] for r in results])
    np.testing.assert_allclose(recursive_position, center, rtol=1e-12, atol=1e-12)
    for r in results:
        for p in POINTS:
            np.testing.assert_allclose(r[p].ae+r[p].ar+r[p].aC,
                                       r[p].state.acceleration, atol=1e-12)
    rows, errors = [], []
    for h in STEPS:
        plus = np.array([reference_positions(t+h, carrier) for t in times])
        minus = np.array([reference_positions(t-h, carrier) for t in times])
        fd = np.stack(((plus-minus)/(2*h), (plus-2*center+minus)/h**2), axis=2) # finite difference
        error = np.linalg.norm(fd-exact, axis=-1)
        errors.append(error)
        for i, p in enumerate(POINTS):
            for j, quantity in enumerate(("velocity", "acceleration")):
                maximum = error[:, i, j].max()
                rms = np.sqrt(np.mean(error[:, i, j]**2))
                scale = max(1., np.linalg.norm(exact[:, i, j], axis=-1).max())
                rows.append([name, h, p, quantity, maximum, rms, maximum/scale])
    with (out/f"{name}_errors.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["h_s", "time_s", "point", "velocity_error_m_s", "acceleration_error_m_s2"])
        for h, error in zip(STEPS, errors):
            for k, t in enumerate(times):
                for i, p in enumerate(POINTS):
                    w.writerow([h, t, p, *error[k, i]])
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for j, quantity in enumerate(("Velocity", "Acceleration")):
        for i, p in enumerate(POINTS):
            axes[j, 0].semilogy(times, errors[-1][:, i, j], label=p)
            axes[j, 1].loglog(STEPS, [e[:, i, j].max() for e in errors], "o-", label=p)
        axes[j, 0].set(xlabel="Time (s)", ylabel=f"{quantity} error ({'m/s' if j == 0 else 'm/s²'})",
                       title=f"{name}: central difference, h={STEPS[-1]} s")
        axes[j, 1].set(xlabel="Step h (s)", ylabel="Maximum error", title=f"{quantity}: convergence")
        for ax in axes[j]:
            ax.grid(True, which="both", alpha=.3)
            ax.legend()
    fig.tight_layout()
    fig.savefig(out/f"{name}_validation.png", dpi=160)
    plt.close(fig)
    checks = []
    for i, p in enumerate(POINTS):
        for j, quantity in enumerate(("velocity", "acceleration")):
            order = np.log2(errors[-2][:, i, j].max()/errors[-1][:, i, j].max())
            scale = max(1., np.linalg.norm(exact[:, i, j], axis=-1).max())
            normalized = errors[-1][:, i, j].max()/scale
            checks.append((name, p, quantity, errors[-1][:, i, j].max(), normalized,
                           order, bool(normalized < 1e-3 and 1.8 <= order <= 2.2)))
    return rows, checks


def write_report(out, checks, duration, dt):
    lines = ["# 运动学分析报告：任务一、二", "",
             "## 模型与方法", "",
             "公式推导、模型选择及接口约定见 [README](README.md#运动学任务一二)。"
             "沿用三连杆及杆中点质心，使用 K1 匀速转台、K2 垂荡与横摇船体。"
             "关节运动的解析导数作为输入，点的运动通过逐级复合计算。",
             "", f"时间范围为 0～{duration:g} s，运动数据输出步长为 {dt:g} s。"
             "所有最终向量均使用惯性系 S0 坐标。分解采用载体系 S1 作为动系；"
             "局部递推分解单独保存，不能直接跨坐标系相加。", "",
             "## 验证方法与判据", "",
             "独立调用原有正运动学函数生成位置，经载体变换后直接做一阶、二阶中心差分。"
             "比较两种载体下末端及三个质心的速度、加速度；另检查递推位置与独立位置一致。"
             "四个步长为 0.04、0.02、0.01、0.005 s，均在共同内部采样时刻比较，不纳入端点。",
             "", "误差采用二维误差向量的欧氏范数，输出最大值和均方根值。"
             "归一化尺度为解析结果的最大模长与 1 m/s 或 1 m/s² 中较大者。"
             "通过要求：最细步长归一化最大误差 < 10⁻³，最后两级观测阶在 [1.8, 2.2]。", "",
             "|载体|点|量|最大误差（SI）|归一化误差|收敛阶|结果|",
             "|---|---|---|---:|---:|---:|---|"]
    for name, p, quantity, maximum, normalized, order, passed in checks:
        lines.append(f"|{name}|{p}|{'速度' if quantity == 'velocity' else '加速度'}|"
                     f"{maximum:.3e}|{normalized:.3e}|{order:.4f}|{'通过' if passed else '未通过'}|")
    for name in CARRIERS:
        image_path = (out/f"{name}_validation.png").as_posix()
        lines += ["", f"## {name} 误差与收敛", "", f"![{name} 验证]({image_path})", "",
                  "左列展示最细步长下四个点的逐时刻误差，右列展示最大误差随步长的变化。"
                  "局部误差起伏来自轨迹高阶导数随时间变化；步长减半后误差约缩小四倍，"
                  "对应中心差分的二阶截断误差。"]
    lines += ["", "## 误差来源与结论", "",
              "中心差分的速度截断误差主项为 h²r'''/6，加速度主项为 h²r''''/12。"
              "步长过小时，位置相减导致浮点消减，尤其二阶差分再除以 h²，"
              "因此不能无限缩小步长。此处不差分包裹后的角度；三角函数接收连续角度，"
              "跨越 ±π 不会人为制造速度跳变。端点不采用单边差分，以保持验证阶数一致。", "",
              ("本次所有载体、点和运动量均满足上述判据，支持逐级合成实现的正确性。"
               if all(c[-1] for c in checks) else "存在未通过项，请检查数值结果，不能认定验证通过。"),
              "验证结论针对当前平滑工况；它不替代对新增非平滑运动规律的单独检查。", "",
              "## 数据复用", "",
              f"数据位于 `{out.as_posix()}/`。`*_motion.csv` 的 com1、com2、com3 行中，"
              "a_x、a_y 可作为后续动力学的质心绝对加速度输入。"
              "`*_stages.csv` 保留每级递推结果；`*_errors.csv` 保存逐时刻差分误差；"
              "`validation_summary.csv` 保存各步长最大误差、均方根误差及归一化误差。"]
    Path("运动学分析报告.md").write_text("\n".join(lines)+"\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--dt", type=float, default=.01)
    parser.add_argument("--output", type=Path, default=Path("assets/kinematics"))
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < .16:
        parser.error("duration must be finite and at least 0.16 s")
    if not np.isfinite(args.dt) or args.dt <= 0:
        parser.error("dt must be finite and positive")
    times = np.arange(int(np.floor(args.duration/args.dt))+1)*args.dt
    if times[-1] < args.duration:
        times = np.append(times, args.duration)
    args.output.mkdir(parents=True, exist_ok=True)
    rows, checks = [], []
    for name, carrier in CARRIERS.items():
        export_motion(args.output, name, carrier, times)
        r, c = validate(args.output, name, carrier, args.duration)
        rows.extend(r)
        checks.extend(c)
    with (args.output/"validation_summary.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["carrier", "h_s", "point", "quantity", "max_error", "rms_error", "normalized_max_error"])
        w.writerows(rows)
    write_report(args.output, checks, args.duration, args.dt)
    passed = all(c[-1] for c in checks)
    print(f"Validation: {'PASS' if passed else 'FAIL'} ({sum(c[-1] for c in checks)}/{len(checks)}); output: {args.output}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
