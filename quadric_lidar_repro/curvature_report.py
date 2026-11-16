"""Standalone scientific figures and a local report for curvature experiments."""

from html import escape
from pathlib import Path
import shutil

import numpy as np

LABELS = {"plane": "Inclined plane", "sphere": "Sphere R=2m", "cylinder": "Cylinder R=2m",
          "saddle": "Saddle", "ellipsoid": "Ellipsoid", "shallow_sphere": "Sphere R=10m", "ripple": "Non-quadric ripple"}
NAMES = {"schur": "Schur (default)", "taubin": "Taubin (default)",
         "taubin_general": "Taubin (plane gate disabled)", "jet": "Local quadratic (64-NN)",
         "jet256": "Local quadratic (256-NN)", "pca": "PCA normal/slope"}
COLORS = {"schur": "#216e9c", "taubin": "#7d6aab", "taubin_general": "#3f9270",
          "jet": "#d38235", "jet256": "#a25426", "pca": "#767676"}


def average(result, shape, method, field, noise=.01):
    rows = [r for r in result["accuracy"] if r["shape"] == shape and r["method"] == method and np.isclose(r["noise_m"], noise)]
    values = [r[field] for r in rows if r[field] is not None]
    return float(np.mean(values)) if values else None


def fmt(value, digits=4):
    return "无有效值" if value is None else f"{value:.{digits}f}"


def create_report(output, result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output = Path(output)
    shapes = list(dict.fromkeys(r["shape"] for r in result["accuracy"]))
    methods = ["schur", "taubin", "taubin_general", "jet", "jet256"]
    noise = .01 if any(np.isclose(r["noise_m"], .01) for r in result["accuracy"]) else result["accuracy"][0]["noise_m"]
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.9), constrained_layout=True)
    x = np.arange(len(shapes))
    for j, method in enumerate(methods):
        for ax, field in zip(axes, ["curvature_rmse_per_m", "slope_mae_deg"]):
            values = [average(result, shape, method, field, noise) for shape in shapes]
            ax.bar(x+(j-2)*.16, [np.nan if v is None else v for v in values], width=.15,
                   color=COLORS[method], label=NAMES[method])
    for ax in axes:
        ax.set_xticks(x, [LABELS[s] for s in shapes], rotation=25, ha="right", fontsize=8)
        ax.grid(axis="y", alpha=.2)
    axes[0].set(ylabel="Principal curvature RMSE / m^-1", title=f"Curvature: {noise*1000:g}mm noise; mean over seeds")
    axes[0].set_yscale("symlog", linthresh=.01)
    axes[0].set_ylabel("Principal curvature RMSE / m^-1 (symlog)")
    axes[1].set(ylabel="Slope angle MAE / degree", title="Same input points and independent query points")
    axes[0].legend(fontsize=8)
    fig.savefig(output/"accuracy.png", dpi=170); plt.close(fig)

    data = np.load(output/"examples.npz")
    example_shapes = [s for s in ["sphere", "saddle", "shallow_sphere", "ripple"] if s+"_truth_principal" in data]
    image_names = ["accuracy.png"]
    if example_shapes:
        fig, axes = plt.subplots(len(example_shapes), 5, figsize=(15, 2.7*len(example_shapes)), constrained_layout=True)
        fig.suptitle("max |k|: white = invalid; colors clipped to each row's truth-based range", fontsize=12)
        for i, shape in enumerate(example_shapes):
            truth = np.max(np.abs(data[shape+"_truth_principal"]), axis=1)
            side = int(np.sqrt(len(truth)))
            vmax = max(float(truth.max()), .15)
            for j, method in enumerate(["truth", "schur", "taubin", "taubin_general", "jet256"]):
                values = truth if method == "truth" else np.max(np.abs(data[f"{shape}_{method}_principal"]), axis=1)
                image = axes[i, j].imshow(values.reshape(side, side), origin="lower", extent=(-.65,.65,-.65,.65),
                                           vmin=0, vmax=vmax, cmap="viridis")
                axes[i, j].set_title(("Truth" if method == "truth" else NAMES[method])+"\n"+LABELS[shape], fontsize=9)
                axes[i, j].set(xlabel="x / m", ylabel="y / m" if j == 0 else "")
            fig.colorbar(image, ax=list(axes[i, :]), shrink=.75, label="max |k| / m^-1")
        fig.savefig(output/"curvature_maps.png", dpi=150); plt.close(fig)
        image_names.append("curvature_maps.png")
    data.close()

    fig, axes = plt.subplots(1, 2, figsize=(12,4.3), constrained_layout=True)
    speed_names = [("taubin_existing_model", "Existing Taubin model + projection", "#3f9270"),
                   ("taubin_fit_and_geometry", "Taubin fit + analytic geometry", "#7d6aab"),
                   ("local_jet_full", "Local quadratic, full", "#d38235"),
                   ("pca_normal_slope_only", "PCA normal/slope only", "#767676")]
    for key, label, color in speed_names:
        axes[0].plot([r["queries"] for r in result["speed"]],
                     [r["timings"][key]["median_ms"] for r in result["speed"]], "o-", label=label, color=color)
    axes[0].set(xscale="log", yscale="log", xlabel="Query points on ONE patch", ylabel="Median time / ms",
                title=f"One pre-segmented patch, {result['protocol']['args']['points']} support points")
    axes[0].legend(fontsize=8)
    if result["real"]:
        x = np.arange(len(result["real"]))
        for i, (key, label, color) in enumerate(speed_names[1:]):
            axes[1].bar(x+(i-1)*.25, [r["timings"][key]["median_ms"] for r in result["real"]],
                        width=.24, color=color, label=label)
        axes[1].set_xticks(x, [f"KITTI04/{r['frame']:06}\n{r['query_points']} queries" for r in result["real"]], fontsize=8)
        axes[1].set(ylabel="Median time / ms", title="All retained points; shared segmentation")
        axes[1].legend(fontsize=8)
    else:
        axes[1].axis("off")
    for ax in axes:
        ax.grid(axis="y", alpha=.2)
    fig.savefig(output/"speed.png", dpi=170); plt.close(fig)
    image_names.append("speed.png")

    if (output/"kitti_geometry.npz").exists():
        with np.load(output/"kitti_geometry.npz") as scene:
            good = np.flatnonzero(scene["valid"])
            keep = good[np.linspace(0, len(good)-1, min(12000, len(good)), dtype=int)]
            p = scene["points"][keep]
            curvature = np.max(np.abs(scene["principal"][keep]), axis=1)
            vmax = float(np.percentile(curvature, 95))
            fig = plt.figure(figsize=(13,5), constrained_layout=True)
            for index, (values, title, limit) in enumerate([(scene["slope_deg"][keep], "Analytic slope / degree", 90),
                                                          (curvature, f"max |k| / m^-1 (clipped at P95={vmax:.2f})", vmax)]):
                ax = fig.add_subplot(1,2,index+1, projection="3d")
                image = ax.scatter(*p.T, c=values, s=1, cmap="viridis", vmin=0, vmax=max(limit, 1e-6))
                ax.set(xlabel="LiDAR x / m", ylabel="LiDAR y / m", zlabel="LiDAR z / m", title=title)
                ax.view_init(elev=25, azim=-125); ax.set_box_aspect((2,1.5,.6))
                fig.colorbar(image, ax=ax, shrink=.65)
            fig.savefig(output/"kitti_geometry.png", dpi=160); plt.close(fig)
        image_names.append("kitti_geometry.png")

    chosen = next((r for r in result["speed"] if r["queries"]==1000), result["speed"][-1])
    t = chosen["timings"]
    full, existing, baseline = [t[k]["median_ms"] for k in
                                ["taubin_fit_and_geometry", "taubin_existing_model", "local_jet_full"]]
    lines = ["# 二次曲面解析法向、坡度和曲率：验证与速度比较", "",
             f"CPU：{result['environment']['cpu']}；Python {result['environment']['python']} / NumPy {result['environment']['numpy']}，单线程BLAS。", "",
             f"主速度案例：{chosen['training_points']}个点构成一个已分割面片，{chosen['queries']}个查询点。",
             f"Taubin预存模型+投影+解析几何 {existing:.4f}ms；Taubin从点云拟合后计算 {full:.4f}ms；局部二次拟合64近邻 {baseline:.4f}ms。", "",
             "实测结论：解析求导成立，但默认Schur拟合受噪声影响明显。Taubin在符合二次曲面模型的场景更稳定；",
             "缓弯面的平面分类、非二次起伏面的模型偏差仍需处理。当前整帧密集查询仍超过100ms，不能把局部查询速度当作整帧实时性能。", "",
             "## 精度验证", "",
             f"七种场景，噪声{result['protocol']['args']['noise']}米，{result['protocol']['args']['seeds']}个随机种子，",
             f"每例{result['protocol']['args']['points']}个支持点、{result['protocol']['args']['query_side']**2}个独立查询点。",
             "噪声沿干净表面法向加入，查询位于面片内部。真值由显式图形的第一/第二基本形式独立计算。",
             "法向误差不计朝向；曲率仅在评估时按真值法向统一正负号，真值不参与拟合。",
             "误差为多个随机种子RMSE/MAE的平均；同时保留有效覆盖率，PCA不参与几何曲率误差排名。", "",
             f"以下为{noise*1000:g}mm噪声：", "",
             "|场景|默认Schur曲率RMSE(m⁻¹)|Taubin曲率RMSE(m⁻¹)|关闭平面门限的Taubin|局部二次64近邻RMSE(m⁻¹)|局部二次256近邻RMSE(m⁻¹)|Taubin坡度MAE(°)|局部二次256近邻坡度MAE(°)|Schur有效比例|",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for shape in shapes:
        values = [average(result, shape, method, "curvature_rmse_per_m", noise) for method in methods]
        slope = [average(result, shape, method, "slope_mae_deg", noise) for method in ["taubin","jet256"]]
        coverage = average(result, shape, "schur", "valid_fraction", noise)
        lines.append("|"+LABELS[shape]+"|"+"|".join(fmt(v) for v in values+slope)+f"|{coverage:.1%}|")
    lines += ["", "![误差比较](accuracy.png)", "",
              "曲率误差图采用对称对数纵轴，保留零误差并显示数量级差异；无有效值表示拟合或投影失败。",
              "默认平面门限可能把缓弯曲面归为平面；关闭门限是单独的消融对照，未修改原里程计默认配置。",
              "非二次起伏面检验模型偏差，不能因为解析求导快就把整个复杂场景拟合成一个二次曲面。",
              "局部二次方法比较64及256近邻、PCA坐标系和二阶多项式；大邻域降低噪声，同时扩大支持范围。",
              "这是本地NumPy实现，不是CGAL或作者原程序。", ""]
    ripple_coverage = average(result, "ripple", "taubin_general", "valid_fraction", noise)
    if ripple_coverage is not None:
        lines += [f"关闭平面门限的Taubin在非二次起伏面仅{ripple_coverage:.1%}查询点数值有效；表中其误差只在这些有效点上计算。", ""]
    if "curvature_maps.png" in image_names:
        lines += ["![真值与估计曲率空间分布](curvature_maps.png)", "",
                  "图中显示max(|k₁|,|k₂|)，同一行使用相同色标；色标按真值范围设置，超出值截断，白色表示无效。",
                  "10mm噪声，第一个种子。", ""]
    lines += ["## 速度", "",
              "全部预热后交错随机顺序计时，使用同一支持点云和查询点；包含结果构建，不含结果销毁。",
              "完整解析法包括点云矩统计、曲面拟合、4次Newton法向投影和解析法向/坡度/主曲率/平均曲率/高斯曲率。",
              "已有模型一列不含拟合。局部二次一列包含建KD树、近邻搜索、PCA和局部多项式拟合及几何计算。",
              "两者利用的支持范围不同：解析法假设已有单一面片，局部方法每个查询点使用64或256近邻。",
              "PCA只输出法向、坡度和无量纲表面变化指标，输出能力更少。", "",
              "|查询点数|Schur仅导数，不投影(ms)|Taubin已有模型+投影+几何(ms)|Schur拟合+几何(ms)|Taubin拟合+几何(ms)|局部二次64完整(ms)|局部二次256完整(ms)|64近邻复用KD树(ms)|PCA法向坡度(ms)|Taubin对64近邻加速比|",
              "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    speed_keys = ["analytic_derivatives_only", "taubin_existing_model", "quadric_fit_and_geometry", "taubin_fit_and_geometry",
                  "local_jet_full", "local_jet256_full", "local_jet_reuse_tree", "pca_normal_slope_only"]
    for row in result["speed"]:
        values = [row["timings"][k]["median_ms"] for k in speed_keys]
        lines.append(f"|{row['queries']}|"+"|".join(f"{v:.4f}" for v in values)+f"|{values[4]/values[3]:.2f}×|")
    lines += ["", "![速度比较](speed.png)", "", "## KITTI真实点云：只验证计算与耗时", "",
              "先按同一算法分割，两条方法都只使用该面片自己的支持点，不跨面片查找邻居。",
              "查询该帧所有保留点；计时不含分割、读盘、关联和位姿优化。真实场景没有曲率真值，不作精度排名。", "",
              "|帧|查询点数|面片|Schur单独拟合(ms)|Schur已有模型几何(ms)|Schur拟合+几何(ms)|Taubin已有模型几何(ms)|Taubin拟合+几何(ms)|局部二次64(ms)|PCA法向坡度(ms)|Schur数值有效点|Taubin数值有效点|",
              "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in result["real"]:
        values = [row["timings"][k]["median_ms"] for k in ["fit_only","analytic_existing_models","quadric_fit_and_geometry",
                  "taubin_existing_models","taubin_fit_and_geometry","local_jet_full","pca_normal_slope_only"]]
        lines.append(f"|{row['frame']}|{row['query_points']}|{row['patches']}|"+"|".join(f"{v:.2f}" for v in values)+
                     f"|{row['valid_fraction']:.2%}|{row['taubin_valid_fraction']:.2%}|")
    if "kitti_geometry.png" in image_names:
        lines += ["", "![真实点云坡度与曲率](kitti_geometry.png)", "",
                  "图为观测点按所属面片的解析量着色；曲率色标在95百分位截断，不能据此认定曲率准确。"]
    lines += ["", "## 证据与限制", "",
              "- 闭式曲率、投影、系数缩放、刚体变换、法向有限差分和局部鞍面测试见 tests/test_curvature.py。",
              "- 原始逐次耗时、全部种子误差、配置、输入和源码SHA256见 metrics.json；accuracy.csv / speed.csv可直接检查。",
              "- 合成场景假设分割正确，不含离群点、遮挡、桌腿与地面混合等问题；不能据此声称解决了这些场景。",
              "- 导数公式正确与拟合曲面可靠是两件事；当前里程计精度问题未因新增曲率计算而解决。", "",
              "解析几何公式参考：[Goldman 2005](https://www.cgeo.uliege.be/CAO/Goldman_Curvature_formulas_implicit.pdf)。",
              "局部多项式方法参考：[CGAL官方Jet fitting说明](https://doc.cgal.org/latest/Jet_fitting_3/index.html)。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))

    body = []
    table = []
    def flush_table():
        if not table:
            return
        head = table[0].strip("|").split("|")
        body.append('<div class="scroll"><table><thead><tr>'+"".join("<th>"+escape(c)+"</th>" for c in head)+"</tr></thead><tbody>")
        for line in table[2:]:
            body.append("<tr>"+"".join("<td>"+escape(c)+"</td>" for c in line.strip("|").split("|"))+"</tr>")
        body.append("</tbody></table></div>"); table.clear()
    for line in lines:
        if line.startswith("|"):
            table.append(line); continue
        flush_table()
        if line.startswith("# "):
            body.append("<h1>"+escape(line[2:])+"</h1>")
        elif line.startswith("## "):
            body.append("<h2>"+escape(line[3:])+"</h2>")
        elif line.startswith("!["):
            alt, name = line[2:].split("](",1); name=name[:-1]
            body.append(f'<a href="{escape(name)}"><img src="{escape(name)}" alt="{escape(alt)}"></a>')
        elif line:
            body.append("<p>"+escape(line)+"</p>")
    flush_table()
    links = '<nav><a href="metrics.json">原始JSON</a><a href="accuracy.csv">精度CSV</a><a href="speed.csv">速度CSV</a><a href="REPORT.md">Markdown报告</a></nav>'
    html = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>二次曲面曲率验证与速度</title><style>body{margin:0;background:#f3f5f7;color:#243445;font:15px/1.7 system-ui,sans-serif}main{max-width:1280px;margin:auto;background:white;padding:32px}h1{font-size:28px;line-height:1.4}h2{margin-top:40px;border-top:1px solid #dde3e8;padding-top:22px}p{margin:8px 0}img{width:100%;height:auto;margin:20px 0}a{color:#176994}nav{display:flex;gap:20px;flex-wrap:wrap}.scroll{overflow:auto;margin:20px 0}table{border-collapse:collapse;white-space:nowrap;width:100%;font-size:13px}th,td{border-bottom:1px solid #dde3e8;padding:10px;text-align:right}th{background:#edf3f8}td:first-child,th:first-child{text-align:left}@media(max-width:600px){main{padding:16px}h1{font-size:23px}}</style><main>'+links+"".join(body)+'</main></html>'
    html = html.replace("<style>", "<style>html{overflow-wrap:anywhere}main{box-sizing:border-box;width:100%}.scroll{max-width:100%}")
    (output/"index.html").write_text(html)
    viewer = ROOT_VISUALIZATION()/"quadric-curvature"
    viewer.mkdir(parents=True, exist_ok=True)
    for name in ["index.html", "REPORT.md", "metrics.json", "accuracy.csv", "speed.csv", "kitti_speed.csv", *image_names]:
        source = output/name
        if source.exists():
            shutil.copy2(source, viewer/name)


def ROOT_VISUALIZATION():
    return Path(__file__).resolve().parents[1]/"fsgp_bgk_repro/outputs/visualization"
