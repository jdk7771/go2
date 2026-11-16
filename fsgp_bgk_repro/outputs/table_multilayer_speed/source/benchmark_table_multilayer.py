"""Time the actual 2125-point, K=30, height-gated multilayer table pipeline.

No cached trees, neighbours or coefficients in the whole-frame timed calls.
Reading, Python import, comparison-method fitting, JSON/image generation and
browser rendering are outside these algorithm timings.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter_ns

import numpy as np

from .experiment import environment
from .table_multilayer import (SETTINGS, assign_layers, fit_layer,
                               fit_multilayer, grid_triangles, height_modes)

ROOT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(samples):
    return dict(median_ms=float(np.median(samples)),
                p95_ms=float(np.percentile(samples, 95)),
                minimum_ms=float(np.min(samples)), samples_ms=samples)


def time_tasks(tasks, repeats, warmups):
    for fn in tasks.values():
        for _ in range(warmups):
            fn()
    values = {name: [] for name in tasks}
    names = list(tasks)
    rng = np.random.default_rng(530)
    for _ in range(repeats):
        for i in rng.permutation(len(names)):
            name = names[i]
            start = perf_counter_ns()
            result = tasks[name]()
            values[name].append((perf_counter_ns()-start)/1e6)
            del result
    return {name: stats(samples) for name, samples in values.items()}


def queries(size):
    axis = np.linspace(-2.5, 2.5, size)
    x, y = np.meshgrid(axis, axis)
    return np.column_stack((x.ravel(), y.ravel()))


def frame_tasks(points, xy, size):
    def fitting():
        return fit_multilayer(points, xy, gate=0.04)

    def fitting_and_mesh():
        centres, labels, fits = fitting()
        triangles = [grid_triangles(f["height"], size, size,
                                    SETTINGS["max_triangle_height_jump_m"]) for f in fits]
        return centres, labels, fits, triangles

    return dict(fitting=fitting, fitting_and_mesh=fitting_and_mesh)


def report(result, output):
    lines = ["# 大桌子：2125 点、多层 30 近邻拟合耗时", "",
             f"实测硬件：{result['environment']['processor']}；Python {result['environment']['python'].split()[0]}；NumPy/SciPy CPU，BLAS 单线程，未使用 GPU。",
             f"预热 {result['warmups']} 次后，每项测 {result['repeats']} 次，按固定随机顺序交错运行。以下均为中位数 / P95，单位 ms。", "",
             "同一帧 2125 点，自动 XYZ 高度峰分层、±4 cm 门限、每层半径 0.75 m 内恰好 30 近邻，不足留空；最近 XY 支持 0.20 m 和局部凸包/退化/预测高度检查均计入。",
             "整帧计时每次重新分层、建树、搜索、构造六系数二次模型并求解，三个高度层全部处理，不复用上次的近邻或模型。", "",
             "| XY 查询网格 | 查询层对数（三层） | 有效拟合数 | 分层+拟合 | 再加独立建网 | 三角形数 |",
             "|---|---:|---:|---:|---:|---:|"]
    for row in result["frames"]:
        values = [f"{row['timing'][key]['median_ms']:.3f} / {row['timing'][key]['p95_ms']:.3f}"
                  for key in ("fitting", "fitting_and_mesh")]
        lines.append(f"| {row['size']}×{row['size']} = {row['query_points']} | {row['query_layer_pairs']} | {sum(row['valid_by_layer'])} | " +
                     " | ".join(values)+f" | {sum(row['triangles_by_layer'])} |")
    lines += ["", "两个网格均覆盖 [-2.5, 2.5]²；101×101 是当前页面实际输出，5 cm 间隔。25×25 是额外的低密度对照，间隔约 20.83 cm。", "",
              "## 当前页面的阶段开销", "",
              "各阶段独立重复计时：分层项仅检测峰及分配点；单层项从已知分层开始，含该层每次建树、近邻、所有有效性检查和拟合；建网项使用已完成拟合结果。各中位数不要求相加等于整帧中位数。", "",
              "| 阶段 | 中位数 / P95 ms |", "|---|---:|"]
    names = dict(height_segmentation="高度峰检测与分层", floor_layer="地面层全部查询", underside_layer="桌底层全部查询", top_layer="桌面层全部查询", mesh_only="三层独立建网")
    for key, value in result["stages"].items():
        lines.append(f"| {names[key]} | {value['median_ms']:.3f} / {value['p95_ms']:.3f} |")
    centre = result["single_xy"]
    lines += ["", "## 只查询桌子中心一个 XY", "",
              f"从原始 2125 点重新检测高度层、建树并拟合三个高度，耗时 {centre['timing']['median_ms']:.3f} / {centre['timing']['p95_ms']:.3f} ms。",
              "该项包含全帧高度分层和三层的树/近邻开销，不是单个 6×6 线性求解的耗时。", "",
              "## 计时边界与证据", "",
              "不含文件读取、Python 启动/导入、旧单层对照、绘图、评估、导出 JSON/PNG、HTTP 传输或浏览器渲染，也不含 FSGP 的 GP/BGK。当前网页读取预先拟合文件，交互加载时间不是这里的算法耗时。",
              "实测的是现有 Python/NumPy 代码在本机 CPU 上的延迟，不能直接当作完整机器人系统帧率。",
              "当前 101×101 输出已逐值对照原页面完整精度 NPZ：输入、分层、各层高度（含无效处）及三角形完全一致。metrics.json 保存每次样本、环境、线程设定、输入及源代码哈希；source/ 保存本次实现快照。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=31)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/table_multilayer_speed")
    args = parser.parse_args()
    assert args.repeats >= 5 and args.warmups >= 1
    output = args.output
    (output/"source").mkdir(parents=True, exist_ok=True)
    source = ROOT/"outputs/table_cpu/table_m125_evidence.npz"
    points = np.asarray(np.load(source)["points"], dtype=float)
    assert points.shape == (2125, 3)
    source_files = ["table_multilayer.py", "benchmark_table_multilayer.py"]
    for name in source_files:
        shutil.copy2(ROOT/name, output/"source"/name)
    result = dict(environment=environment(), repeats=args.repeats, warmups=args.warmups,
                  settings=SETTINGS, input_path=str(source.relative_to(ROOT)),
                  input_sha256=sha(source), input_points=len(points),
                  source_sha256={name: sha(ROOT/name) for name in source_files}, frames=[])
    for size in (25, 101):
        xy = queries(size)
        tasks = frame_tasks(points, xy, size)
        centres, labels, fits, faces = tasks["fitting_and_mesh"]()
        checks = {}
        if size == 101:
            reference = np.load(ROOT/"outputs/table_multilayer/arrays/scan_04cm.npz")
            for key, value in dict(points=points, xy=xy, centres=centres, labels=labels).items():
                np.testing.assert_array_equal(value, reference[key])
            for i, (fit, triangles) in enumerate(zip(fits, faces)):
                np.testing.assert_allclose(fit["height"], reference[f"layer{i}_height"], atol=0, rtol=0, equal_nan=True)
                np.testing.assert_array_equal(triangles, reference[f"layer{i}_triangles"])
            checks["identical_to_current_viewer_arrays"] = True
        row = dict(size=size, query_points=len(xy), query_layer_pairs=len(xy)*len(centres),
                   query_sha256=hashlib.sha256(xy.tobytes()).hexdigest(),
                   centres=centres.tolist(), assigned_points_by_layer=[int((labels == i).sum()) for i in range(len(centres))],
                   valid_by_layer=[int(np.isfinite(f["height"]).sum()) for f in fits],
                   triangles_by_layer=[len(f) for f in faces], checks=checks,
                   timing=time_tasks(tasks, args.repeats, args.warmups))
        result["frames"].append(row)
        print(size, {key: {s: v[s] for s in ('median_ms', 'p95_ms')} for key, v in row['timing'].items()}, flush=True)
    xy = queries(101)
    centres, labels, fits = fit_multilayer(points, xy, gate=.04)

    def segmentation():
        c = height_modes(points)
        return c, assign_layers(points, c, .04)

    stages = dict(height_segmentation=segmentation)
    for i, name in enumerate(("floor_layer", "underside_layer", "top_layer")):
        ids = np.flatnonzero(labels == i)
        stages[name] = lambda i=i, ids=ids: fit_layer(points, xy, ids, centres[i], .04)
    stages["mesh_only"] = lambda: [grid_triangles(f["height"], 101, 101, SETTINGS["max_triangle_height_jump_m"]) for f in fits]
    result["stages"] = time_tasks(stages, args.repeats, args.warmups)
    timing = time_tasks(dict(single=lambda: fit_multilayer(points, np.array([[0., 0.]]), gate=.04)), args.repeats, args.warmups)
    centre_fits = fit_multilayer(points, np.array([[0., 0.]]), gate=.04)[2]
    result["single_xy"] = dict(query_xy=[0., 0.], timing=timing["single"], heights_m=[float(f["height"][0]) for f in centre_fits])
    (output/"metrics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    report(result, output)
    destination = ROOT/"outputs/visualization/table-layers/speed"
    shutil.copytree(output, destination, dirs_exist_ok=True)
    print(json.dumps(dict(stages=result["stages"], single_xy=result["single_xy"]), ensure_ascii=False))


if __name__ == "__main__":
    main()
