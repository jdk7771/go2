"""Time the exact seed-local table pipeline and diagnose mesh omissions.

Every full timed call rebuilds neighbours, fits, compatibility graph and meshes
from the original XYZ. Hole analysis is a separate post-fit diagnostic.
"""

import argparse
from dataclasses import asdict
import hashlib
import inspect
import json
from pathlib import Path
import shutil
from time import perf_counter_ns

import numpy as np

from .experiment import environment
from .table_local_surfaces import (LocalConfig, assemble, fit_seeds,
                                    patch_mesh, reconstruct, select_neighbors)

ROOT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def time_tasks(tasks, repeats, warmups):
    for function in tasks.values():
        for _ in range(warmups):
            function()
    names, samples = list(tasks), {name: [] for name in tasks}
    rng = np.random.default_rng(716)
    for _ in range(repeats):
        for i in rng.permutation(len(names)):
            name = names[i]
            start = perf_counter_ns()
            value = tasks[name]()
            samples[name].append((perf_counter_ns()-start)/1e6)
            del value
    return {name: dict(median_ms=float(np.median(values)), p95_ms=float(np.percentile(values, 95)),
                       minimum_ms=float(np.min(values)), samples_ms=values)
            for name, values in samples.items()}


def diagnose_holes(points, result, config):
    """Exclusive first-failure reasons, not overlapping rejection counts."""
    fit = result["fits"]
    rows = []
    for i, component in enumerate(result["components"]):
        ids, tri = component["point_ids"], component["triangulation"]
        if tri is None:
            continue
        faces = ids[tri.simplices]
        n = len(faces)
        xy = points[faces, :2]
        area = .5*np.abs(np.cross(xy[:, 1]-xy[:, 0], xy[:, 2]-xy[:, 0]))
        passes = {name: np.ones(n, dtype=bool) for name in ("edge_length", "model_agreement", "normal_angle", "centroid_agreement")}
        for left, right in ((0, 1), (1, 2), (2, 0)):
            a, b = faces[:, left], faces[:, right]
            distance = np.linalg.norm(np.column_stack((points[a, :2]-points[b, :2], fit["height"][a]-fit["height"][b])), axis=1)
            passes["edge_length"] &= distance <= config.connect_radius_m
            passes["normal_angle"] &= np.einsum("ni,ni->n", fit["normal"][a], fit["normal"][b]) >= np.cos(np.deg2rad(config.max_normal_angle_deg))
            for source, target in ((a, b), (b, a)):
                u, v = (points[target, :2]-points[source, :2]).T
                design = np.column_stack((np.ones(n), u, v, u*u, u*v, v*v))
                pred = points[source, 2]+np.einsum("ni,ni->n", design, fit["coefficient"][source])
                passes["model_agreement"] &= np.abs(pred-fit["height"][target]) <= config.max_model_disagreement_m
        centre_xy = xy.mean(axis=1)
        centre_z = fit["height"][faces].mean(axis=1)
        for corner in range(3):
            seeds = faces[:, corner]
            u, v = (centre_xy-points[seeds, :2]).T
            design = np.column_stack((np.ones(n), u, v, u*u, u*v, v*v))
            prediction = points[seeds, 2]+np.einsum("ni,ni->n", design, fit["coefficient"][seeds])
            passes["centroid_agreement"] &= np.abs(prediction-centre_z) <= config.max_model_disagreement_m
        remaining, reasons = np.ones(n, dtype=bool), {}
        for name, keep in passes.items():
            failed = remaining & ~keep
            reasons[name] = dict(triangles=int(failed.sum()), area_m2=float(area[failed].sum()))
            remaining &= keep
        np.testing.assert_array_equal(remaining, component["kept_simplex"])
        rows.append(dict(component=i, seeds=len(ids), median_z_m=float(np.median(fit["height"][ids])),
                         candidate_triangles=n, kept_triangles=int(remaining.sum()),
                         observed_seed_xy_hull_area_m2=float(area.sum()), kept_area_m2=float(area[remaining].sum()),
                         kept_area_fraction_of_seed_hull=float(area[remaining].sum()/area.sum()),
                         first_failure_reasons=reasons))
    # Known synthetic ground is used only to examine failures, never as input
    # to fitting/grouping. This is a +/-6 cm observed-height band, not exact
    # ground-truth point identity (it can include low leg observations).
    near_ground = np.abs(points[:, 2]) < .06
    return dict(components=rows, near_ground_band_m=[-.06, .06],
                near_ground_seeds=int(near_ground.sum()),
                near_ground_under_30=int((near_ground & (fit["candidate_counts"] < config.neighbors)).sum()),
                near_ground_invalid_fit=int((near_ground & ~np.isfinite(fit["height"])).sum()),
                near_ground_median_candidates=float(np.median(fit["candidate_counts"][near_ground])),
                independent_patch_display_width_m=2*config.patch_half_width_m,
                independent_patch_display_convex_hull_clipped=True)


def report(data, output):
    t = data["timings"]
    lines = ["# 局部 30 点拟合：地面缺口、配色与完整耗时", "",
             "本报告对应 table-local 页面，无全局高度峰或预设面数。原始输入 2125 点；每点作为种子，半径 0.75 m、相对自身高差 ±4 cm 后取恰好 30 点（含自身）。", "",
             "## 当前算法的完整耗时", "",
             f"CPU：{data['environment']['processor']}；Python {data['environment']['python'].split()[0]}，NumPy/SciPy CPU，BLAS 单线程；预热 {data['warmups']} 次，每项重复 {data['repeats']} 次，固定随机顺序交错计时。", "",
             "| 计算范围 | 中位数 ms | P95 ms |", "|---|---:|---:|"]
    names = dict(neighbors_only="建树、相对高度过滤与 30 近邻选择", fitting_with_neighbors="上述邻域搜索 + 全部局部二次拟合",
                 assembly_on_ready_fits="已拟合后的兼容图、连通分组和三角网格",
                 complete_xyz_to_mesh="完整：原始 XYZ → 独立局部拟合 → 分组 → 组合网格",
                 complete_with_independent_patch_geometry="完整流程 + 中图独立小曲面的显示采样几何")
    for name in names:
        lines.append(f"| {names[name]} | {t[name]['median_ms']:.3f} | {t[name]['p95_ms']:.3f} |")
    lines += ["", "完整流程每次重新建树、选邻点、拟合、分组及建网，不复用邻域、模型或网格。阶段项是独立测量，不能要求中位数之和等于完整项。中图显示采样包含为 1955 个有效局部函数生成小面片及凸包裁剪，属于额外可视化几何。", "",
              "不含文件读取、Python 启动/导入、静态 PNG 绘制、JSON 导出、网络传输和浏览器渲染。页面当前加载离线结果，页面打开时间不等于算法延迟。", "",
              f"输出：{data['outputs']['valid_fits']} 个有效局部函数，{data['outputs']['major_components']} 个主要连通组件，{data['outputs']['triangles']} 个组合三角形。所有数值和三角形已与页面完整精度 NPZ 对照一致。旧 151 ms 对应另一种全局分层、101×101 查询实现，不能直接作为本实现的耗时。", "",
              "## 地面为什么有空洞", ""]
    holes = data["holes"]
    lines += [f"观察高度位于 ±6 cm 的近地面带共 {holes['near_ground_seeds']} 个种子，仅 {holes['near_ground_under_30']} 个在规定高度带和半径内不足 30 点；候选点数中位数为 {holes['near_ground_median_candidates']:.0f}。这是事后诊断高度带，可能包括少量低处桌腿点，不是算法使用的地面分层或精确语义真值。", "",
              "右图还会因连网条件产生缺口。逐个候选三角形按‘边长→双向局部预测→法向→重心预测’检查，下面列出首次失败原因；不同规则同时失败的三角形只计一次。", "",
              "|自动组件|有效种子|候选三角形|边长>0.35m|预测差>2cm|法向>15°|重心不一致|保留三角形|凸包内保留面积|", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in holes["components"]:
        r = row["first_failure_reasons"]
        lines.append(f"|{row['component']+1} (Z中位数{row['median_z_m']:.3f}m)|{row['seeds']}|{row['candidate_triangles']}|{r['edge_length']['triangles']}|{r['model_agreement']['triangles']}|{r['normal_angle']['triangles']}|{r['centroid_agreement']['triangles']}|{row['kept_triangles']}|{row['kept_area_fraction_of_seed_hull']:.1%}|")
    lines += ["", "组件1事后可解释为地面。面积比例的分母是该组件有效种子的 XY 凸包，不是全场景真实地面的面积。密度影响种子间距，严格的边长和预测一致性也会删掉已有观测之间的三角形，因此不能把这些缺口都解释为‘缺少 30 点’或真实场景有洞。", "",
              "中图另有显示范围限制：每个局部函数仅绘制种子周围 24×24 cm，并裁到这 30 点的 XY 凸包和相对高度带内；即使有有效局部模型，小面片之间也可能留白。", "",
              "## 颜色是怎么来的", "",
              "真实计算顺序是原始 XYZ → 邻域和独立局部函数 → 局部兼容图 → 自动连通组件。此前为了对照，把最终组件标签回填到左侧原始点和中间独立小面片着色；因此看起来好像拟合前已经完成分割。组别由算法计算，RGB 调色板由实现指定，颜色不参与拟合或分组。", "",
              "现已改为默认左图灰色、中图统一蓝色、右图按最终组件着色，并保留‘按最终组件回看前两图颜色’开关。组件开关仅作用于右图及剖面。该显示修正前后，所有近邻、系数、标签、图边和三角形逐数组一致。", "",
              "metrics.json 含所有逐次计时、环境、输入与源代码哈希，holes.json 含逐组件缺口诊断，source/ 为完整实现快照。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=31)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/table_local_speed")
    args = parser.parse_args()
    assert args.repeats >= 5 and args.warmups >= 1
    output = args.output
    (output/"source").mkdir(parents=True, exist_ok=True)
    input_path = ROOT/"outputs/table_cpu/table_m125_evidence.npz"
    points = np.asarray(np.load(input_path)["points"], dtype=float)
    config = LocalConfig()
    reference = np.load(ROOT/"outputs/table_local_surfaces/results.npz")
    result = reconstruct(points, config)
    np.testing.assert_array_equal(points, reference["points"])
    for key, value in result["fits"].items():
        np.testing.assert_allclose(value, reference[key], atol=0, rtol=0, equal_nan=True)
    for key in ("labels", "edges"):
        np.testing.assert_array_equal(result[key], reference[key])
    for i, component in enumerate(result["components"]):
        np.testing.assert_array_equal(component["point_ids"], reference[f"component{i}_point_ids"])
        np.testing.assert_array_equal(component["triangles"], reference[f"component{i}_triangles"])

    def complete():
        return reconstruct(points, config)

    def with_display_geometry():
        current = complete()
        return current, patch_mesh(points, current["fits"], config)

    tasks = dict(neighbors_only=lambda: select_neighbors(points, config),
                 fitting_with_neighbors=lambda: fit_seeds(points, config),
                 assembly_on_ready_fits=lambda: assemble(points, result["fits"], config),
                 complete_xyz_to_mesh=complete,
                 complete_with_independent_patch_geometry=with_display_geometry)
    source_files = ["table_local_surfaces.py", "benchmark_table_local_surfaces.py"]
    for name in source_files:
        shutil.copy2(ROOT/name, output/"source"/name)
    metrics = dict(environment=environment(), settings=asdict(config), repeats=args.repeats, warmups=args.warmups,
                   input_path=str(input_path.relative_to(ROOT)), input_sha256=sha(input_path),
                   source_sha256={name: sha(ROOT/name) for name in source_files},
                   function_sha256={function.__name__: hashlib.sha256(inspect.getsource(function).encode()).hexdigest()
                                    for function in (select_neighbors, fit_seeds, assemble, reconstruct, patch_mesh)},
                   input_points=len(points), viewer_array_equality=True,
                   outputs=dict(valid_fits=int(np.isfinite(result["fits"]["height"]).sum()), major_components=len(result["components"]),
                                triangles=sum(len(c["triangles"]) for c in result["components"])),
                   timings=time_tasks(tasks, args.repeats, args.warmups), holes=diagnose_holes(points, result, config))
    (output/"metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False))
    (output/"holes.json").write_text(json.dumps(metrics["holes"], ensure_ascii=False, indent=2, allow_nan=False))
    report(metrics, output)
    shutil.copytree(output, ROOT/"outputs/visualization/table-local/speed", dirs_exist_ok=True)
    print(json.dumps({key: {s: value[s] for s in ("median_ms", "p95_ms")}
                      for key, value in metrics["timings"].items()}), flush=True)


if __name__ == "__main__":
    main()
