"""Render the actual quadric zero sets from the frozen terrain experiment.

No benchmark is rerun or retuned. Saved XYZ, segmentation labels and the exact
fit configuration reconstruct each patch; fixed-XY predictions are checked
against the recorded experiment before exporting any visualization.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

from .core import Config, Moments, basis, distance_and_gradient, fit_patch, quadric_matrix
from .curvature import differential_geometry
from .terrain import vertical_geometry

ROOT = Path(__file__).resolve().parent
NAMES = {"hill": "丘陵", "forest": "森林", "quick": "Quick 地形",
         "desert": "沙漠", "mountain": "山地", "snow": "雪地", "vocano": "火山"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def solve_coordinate(a, b, c, target):
    """Real root of a*w²+b*w+c=0 nearest the observed local sheet."""
    roots = np.full_like(b, np.nan)
    if abs(a) < 1e-12:
        return np.divide(-c, b, out=roots, where=np.abs(b) > 1e-12)
    discriminant = b*b-4*a*c
    square = np.sqrt(np.maximum(discriminant, 0))
    q = -.5*(b+np.where(b >= 0, 1., -1.)*square)
    first = q/a
    second = np.divide(c, q, out=roots.copy(), where=np.abs(q) > 1e-20)
    roots = np.where(np.abs(first-target) <= np.abs(second-target), first, second)
    repeated = (square < 1e-14) & (np.abs(q) < 1e-20)
    roots[repeated] = -b[repeated]/(2*a)
    roots[discriminant < 0] = np.nan
    return roots


def polynomial_distance(patch, points):
    """Scale-invariant |F|/|grad F|, for auditing vertices of the zero set."""
    c = patch.coefficients/max(np.max(np.abs(patch.coefficients)), 1e-300)
    local = points-patch.moments.origin
    gradient = 2*local@quadric_matrix(c)+c[6:9]
    return np.abs(basis(local)@c)/np.maximum(np.linalg.norm(gradient, axis=1), 1e-30)


def surface_mesh(patch, points, step=.065):
    """Sample F=0 in a PCA tangent chart, retaining only observed support.

    This is an analytic root surface, not a triangulation of observed XYZ.
    Real roots are selected near observed points; convex-hull, 3D support and
    edge checks prevent extending the unbounded quadric across empty regions.
    """
    empty = {"vertices": [], "triangles": [], "support_radius_m": 0.,
             "zero_set_max_error_m": 0., "wire_rounding_max_error_m": 0.,
             "facet_sample_max_error_m": 0.}
    if patch.kind == "distribution":
        return empty
    _, vectors = np.linalg.eigh(patch.moments.covariance)
    first, normal = vectors[:, 2], vectors[:, 0]
    second = np.cross(normal, first)
    rotation = np.column_stack((first, second, normal))
    local = (points-patch.moments.origin)@rotation
    uv = local[:, :2]
    hull = ConvexHull(uv)
    uv_tree, point_tree = cKDTree(uv), cKDTree(points)
    spacing = np.median(uv_tree.query(uv, k=2)[0][:, 1])
    support = float(np.clip(3.2*spacing, .15, .28))
    axes = [np.linspace(uv[:, i].min(), uv[:, i].max(),
                        max(3, int(np.ceil(np.ptp(uv[:, i])/step))+1)) for i in range(2)]
    u, v = np.meshgrid(*axes, indexing="xy")
    coords = np.column_stack((u.ravel(), v.ravel()))
    inside = np.all(coords@hull.equations[:, :2].T+hull.equations[:, 2] <= 1e-10, axis=1)
    distance, nearest = uv_tree.query(coords)
    c = patch.coefficients/max(np.max(np.abs(patch.coefficients)), 1e-300)
    a = rotation.T@quadric_matrix(c)@rotation
    b = rotation.T@c[6:9]
    x, y = coords.T
    linear = 2*a[0, 2]*x+2*a[1, 2]*y+b[2]
    constant = a[0, 0]*x*x+2*a[0, 1]*x*y+a[1, 1]*y*y+b[0]*x+b[1]*y+c[9]
    w = solve_coordinate(a[2, 2], linear, constant, local[nearest, 2])
    valid = inside & (distance <= support) & np.isfinite(w) & (np.abs(w-local[nearest, 2]) <= .4)
    vertices = np.column_stack((coords, np.nan_to_num(w)))@rotation.T+patch.moments.origin
    valid &= point_tree.query(vertices)[0] <= support
    nu, nv = len(axes[0]), len(axes[1])
    start = (np.arange(nv-1)[:, None]*nu+np.arange(nu-1)[None, :]).ravel()
    faces = np.vstack((np.column_stack((start, start+1, start+nu)),
                       np.column_stack((start+1, start+nu+1, start+nu))))
    faces = faces[np.all(valid[faces], axis=1)]
    if not len(faces):
        return empty
    edge = np.linalg.norm(vertices[faces]-vertices[np.roll(faces, 1, axis=1)], axis=2)
    faces = faces[np.max(edge, axis=1) <= .22]
    faces = faces[point_tree.query(vertices[faces].mean(axis=1))[0] <= support]
    # Reject facets bridging two real roots of a folded quadric. Both edge
    # midpoints and the centroid must stay within 3mm of the analytic zero set.
    triangle = vertices[faces]
    samples = np.stack((triangle.mean(axis=1), (triangle[:, 0]+triangle[:, 1])/2,
                        (triangle[:, 1]+triangle[:, 2])/2, (triangle[:, 2]+triangle[:, 0])/2), axis=1)
    sample_error = polynomial_distance(patch, samples.reshape(-1, 3)).reshape(-1, 4)
    faces = faces[np.max(sample_error, axis=1) <= .003]
    if not len(faces):
        return empty
    used, inverse = np.unique(faces, return_inverse=True)
    vertices, faces = vertices[used], inverse.reshape(-1, 3)
    error = float(np.max(polynomial_distance(patch, vertices)))
    assert error < 1e-7, error
    # Browser payload uses six decimal places; check that serialization itself
    # cannot move the displayed vertices appreciably off the fitted surface.
    rounded = vertices.round(6)
    rounded_error = float(np.max(polynomial_distance(patch, rounded)))
    assert rounded_error < 2e-6, rounded_error
    assert np.all(point_tree.query(rounded)[0] <= support+2e-6)
    triangle = rounded[faces]
    samples = np.stack((triangle.mean(axis=1), (triangle[:, 0]+triangle[:, 1])/2,
                        (triangle[:, 1]+triangle[:, 2])/2, (triangle[:, 2]+triangle[:, 0])/2), axis=1)
    facet_error = float(np.max(polynomial_distance(patch, samples.reshape(-1, 3))))
    assert facet_error < .003002
    return {"vertices": rounded.ravel().tolist(), "triangles": faces.ravel().tolist(),
            "support_radius_m": support, "zero_set_max_error_m": error,
            "wire_rounding_max_error_m": rounded_error, "facet_sample_max_error_m": facet_error}


def stats(values):
    return {"rmse_cm": float(np.sqrt(np.mean(values*values))*100),
            "median_cm": float(np.median(values)*100),
            "p95_cm": float(np.percentile(values, 95)*100)} if len(values) else None


def export_frame(path, config, support_radius):
    with np.load(path) as saved:
        points, labels = saved["scan"], saved["labels"]
        xy = saved["xy"]
        distance, nearest = cKDTree(points[:, :2]).query(xy)
        predicted, prediction_valid = np.full(len(xy), np.nan), np.zeros(len(xy), dtype=bool)
        residuals = np.full(len(points), np.nan)
        patches = []
        for label in np.unique(labels[labels >= 0]):
            ids = np.flatnonzero(labels == label)
            support_points = points[ids]
            patch = fit_patch(Moments.from_points(support_points), config)
            selected = (labels[nearest] == label) & (distance <= support_radius)
            result = vertical_geometry(patch, xy[selected], points[nearest[selected], 2])
            predicted[selected], prediction_valid[selected] = result["height"], result["valid"]
            if patch.kind != "distribution":
                residuals[ids] = np.sqrt(distance_and_gradient(patch, support_points)[0])
            projection = differential_geometry(patch, support_points)
            line_ids = np.flatnonzero(projection["valid"])
            line_ids = line_ids[np.linspace(0, len(line_ids)-1, min(40, len(line_ids)), dtype=int)]
            lines = np.stack((support_points[line_ids], projection["points"][line_ids]), axis=1)
            mesh = surface_mesh(patch, support_points)
            patches.append({"id": int(label), "kind": patch.kind, "count": len(ids),
                            "origin": patch.moments.origin.tolist(),
                            "coefficients": patch.coefficients.tolist(),
                            "residual": stats(residuals[ids][np.isfinite(residuals[ids])]),
                            "projection_lines": lines.round(6).ravel().tolist(), **mesh})
        assert np.array_equal(prediction_valid, saved["Quadric_valid"]), path
        delta = float(np.max(np.abs(predicted[prediction_valid]-saved["Quadric_height"][prediction_valid])))
        assert delta < 1e-10, (path, delta)
        name, frame = path.stem.rsplit("_", 1)
        return {"scene": name, "frame": int(frame), "title": NAMES[name],
                "point_count": len(points), "points": points.round(6).ravel().tolist(),
                "labels": labels.tolist(),
                "residual_cm": [round(v*100, 5) if np.isfinite(v) else None for v in residuals],
                "patches": patches, "residual": stats(residuals[np.isfinite(residuals)]),
                "unassigned_points": int(np.sum(labels < 0)),
                "bounds": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                "prediction_reconstruction_max_error_m": delta,
                "source_frame_sha256": digest(path)}


def figure(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family": font.get_name(), "axes.unicode_minus": False})
    points = np.asarray(data["points"]).reshape(-1, 3)
    labels = np.asarray(data["labels"])
    residual = np.array([np.nan if x is None else x for x in data["residual_cm"]])
    fig = plt.figure(figsize=(15.5, 5.6), facecolor="#f7f9fc")
    axes = [fig.add_subplot(1, 3, i+1, projection="3d") for i in range(3)]
    colors = ("#44a0eb", "#a1adbf")
    axes[0].scatter(*points.T, c="#344b63", s=2.4, alpha=.7, depthshade=False)
    axes[1].scatter(*points.T, c="#182d45", s=1.8, alpha=.45, depthshade=False)
    for patch in data["patches"]:
        vertices = np.asarray(patch["vertices"]).reshape(-1, 3)
        faces = np.asarray(patch["triangles"]).reshape(-1, 3)
        if len(faces):
            mesh = Poly3DCollection(vertices[faces], facecolor=colors[patch["kind"] == "plane"],
                                    edgecolor="#2582c4", linewidths=.08, alpha=.65)
            axes[1].add_collection3d(mesh)
    assigned = np.isfinite(residual)
    axes[2].scatter(*points[assigned].T, c=residual[assigned], s=3.2,
                    cmap="turbo", vmin=0, vmax=10, depthshade=False)
    axes[2].scatter(*points[~assigned].T, c="#b5bfcb", s=2.4, depthshade=False)
    for ax, title in zip(axes, ["输入点云", "拟合二次曲面 + 原始点云", "点到曲面残差（Taubin 近似）"]):
        ax.set_title(title, pad=12, fontsize=12)
        for i, axis in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            axis(points[:, i].min(), points[:, i].max())
        ax.set_box_aspect(np.maximum(np.ptp(points, axis=0), .1))
        ax.view_init(elev=30, azim=-68)
        ax.set_xlabel("X (m)", labelpad=0); ax.set_ylabel("Y (m)", labelpad=0)
        ax.set_zlabel("Z (m)", labelpad=0)
        ax.tick_params(labelsize=8)
    bar = fig.colorbar(ScalarMappable(norm=Normalize(0, 10), cmap="turbo"), ax=axes[2], shrink=.55, pad=.04)
    bar.set_label("近似距离 (cm)，颜色上限 10 cm", fontsize=9)
    bar.set_ticks([0, 2, 4, 6, 8, 10]); bar.set_ticklabels(["0", "2", "4", "6", "8", "≥10"])
    q = sum(p["kind"] == "quadric" for p in data["patches"])
    p = sum(p["kind"] == "plane" for p in data["patches"])
    fig.suptitle(f"官方 {data['scene']}.pcd · 第 {data['frame']+1} / 8 帧 · {len(points)} 点 · {q} 个二次曲面 / {p} 个平面", fontsize=15)
    fig.text(.5, .025, "蓝色为二次曲面，灰色为平面分支；保留实验中的原始分割与拟合参数。曲面仅绘于观测支持范围内。", ha="center", fontsize=10)
    fig.subplots_adjust(left=.015, right=.955, top=.86, bottom=.13, wspace=.12)
    fig.savefig(output/"fit-overview.png", dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)

    # Detail selects the largest genuine quadric in the displayed frame. Both
    # the selection rule and all other patches remain available in the viewer.
    patch = max((p for p in data["patches"] if p["kind"] == "quadric" and p["triangles"]), key=lambda p: p["count"])
    sample = points[labels == patch["id"]]
    vertices = np.asarray(patch["vertices"]).reshape(-1, 3)
    faces = np.asarray(patch["triangles"]).reshape(-1, 3)
    fig = plt.figure(figsize=(10, 6), facecolor="#f7f9fc")
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(vertices[faces], facecolor="#54abec", edgecolor="#2283c1", linewidths=.15, alpha=.5))
    ax.scatter(*sample.T, c="#10283e", s=15, depthshade=False, label="观测点")
    lines = np.asarray(patch["projection_lines"]).reshape(-1, 2, 3)
    ax.add_collection3d(Line3DCollection(lines, colors="#e67a28", linewidths=1.1))
    ax.set_box_aspect(np.maximum(np.ptp(sample, axis=0), .06))
    ax.view_init(elev=26, azim=-65)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title(f"{data['scene']} · 面片 #{patch['id']} · {patch['count']} 点\n二次曲面叠加，橙线连接观测点与法向迭代投影点", fontsize=13, pad=18)
    fig.text(.5, .045, f"Taubin 近似距离 RMSE {patch['residual']['rmse_cm']:.2f} cm · P95 {patch['residual']['p95_cm']:.2f} cm", ha="center", fontsize=11)
    fig.savefig(output/"fit-detail.png", dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    opaque_figure(data, output)
    return patch["id"]


def opaque_figure(data, output):
    """An always-visible comparison, also usable without WebGL."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    points = np.asarray(data["points"]).reshape(-1, 3)

    def draw_surface(ax, patches):
        light = np.array([-.4, -.6, .8]); light /= np.linalg.norm(light)
        for patch in patches:
            vertices = np.asarray(patch["vertices"]).reshape(-1, 3)
            faces = np.asarray(patch["triangles"]).reshape(-1, 3)
            if not len(faces):
                continue
            centers = vertices[faces].mean(axis=1)
            coeff = np.asarray(patch["coefficients"])
            normal = 2*(centers-patch["origin"])@quadric_matrix(coeff)+coeff[6:9]
            normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-30)
            intensity = .50+.50*np.abs(normal@light)
            base = np.array(to_rgb("#258bd5" if patch["kind"] == "quadric" else "#9aa9b9"))
            ax.add_collection3d(Poly3DCollection(vertices[faces], facecolors=intensity[:, None]*base,
                edgecolors="none", linewidths=0, alpha=1., antialiased=False))

    def style(ax, cloud):
        for i, axis in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            lo, hi = cloud[:, i].min(), cloud[:, i].max()
            margin = max((hi-lo)*.04, .04)
            axis(lo-margin, hi+margin)
        ax.set_box_aspect(np.maximum(np.ptp(cloud, axis=0), .1))
        ax.view_init(elev=30, azim=-65)
        ax.set_xlabel("X (m)", labelpad=1); ax.set_ylabel("Y (m)", labelpad=1); ax.set_zlabel("Z (m)", labelpad=1)
        ax.tick_params(labelsize=8)

    fig = plt.figure(figsize=(12, 5.6), facecolor="#f7f9fc")
    left, right = [fig.add_subplot(1, 2, i, projection="3d") for i in [1, 2]]
    left.scatter(*points.T, color="#213e58", s=3.2, alpha=1., depthshade=False)
    draw_surface(right, data["patches"])
    for ax in [left, right]:
        style(ax, points)
    left.set_title("① 输入点云", fontsize=15, pad=14)
    right.set_title("② 拟合二次曲面（蓝色实面）", fontsize=15, pad=14)
    fig.suptitle(f"{data['scene']}.pcd · 第 {data['frame']+1}/8 帧 · 相同视角", fontsize=15, y=.98)
    fig.text(.5, .025, "右侧蓝色为分割后拟合的二次曲面，灰色为平面分支；不叠加输入点。空隙保留。", ha="center", fontsize=10)
    fig.subplots_adjust(left=.015, right=.985, top=.88, bottom=.13, wspace=.02)
    fig.savefig(output/"fit-compare.png", dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


def add_home_entry(home):
    """Expose this viewer on the existing local visualization landing page."""
    if not home.exists():
        return
    original = home.read_text()
    if 'id="quadric-fit-entry"' in original:
        return
    banner = ('<nav id="quadric-fit-entry" style="display:flex;align-items:center;justify-content:space-between;'
              'flex-wrap:wrap;gap:12px;margin-bottom:14px;padding:16px 20px;background:#eaf4ff;'
              'border:1px solid #9bc5e8;border-radius:10px;font:14px system-ui;color:#214a6b">'
              '<strong>新增：二次曲面拟合效果（输入点云与拟合实面对比）</strong>'
              '<a href="./quadric-fit/?v=2" style="padding:9px 14px;background:#176fa8;color:white;'
              'border-radius:6px;text-decoration:none">打开二次曲面拟合 →</a></nav>')
    if "<body>" not in original:
        raise ValueError("Landing page has no body element")
    home.write_text(original.replace("<body>", "<body>\n"+banner, 1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, default=ROOT/"outputs/fsgp_terrain")
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/fitted_surfaces")
    parser.add_argument("--publish", type=Path, default=ROOT.parent/"fsgp_bgk_repro/outputs/visualization/quadric-fit")
    args = parser.parse_args()
    protocol = json.loads((args.experiment/"protocol.json").read_text())
    for name, expected in protocol["source_sha256"].items():
        assert digest(ROOT/name) == expected, f"Frozen experiment source changed: {name}"
    config = Config(**protocol["fit_config"])
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"frames").mkdir(exist_ok=True)
    manifest, audit = [], []
    default_data = None
    for scene in protocol["scenes"]:
        name = scene["scene"]
        for frame in range(protocol["frames"]):
            path = args.experiment/"frames"/f"{name}_{frame:03d}.npz"
            data = export_frame(path, config, protocol["segmentation"]["support_radius"])
            destination = args.output/"frames"/(path.stem+".json")
            destination.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
            audit.append({"frame": path.stem, "input_sha256": data["source_frame_sha256"],
                          "output_sha256": digest(destination),
                          "grid_reconstruction_max_error_m": data["prediction_reconstruction_max_error_m"],
                          "mesh_vertices": sum(len(p["vertices"])//3 for p in data["patches"]),
                          "mesh_triangles": sum(len(p["triangles"])//3 for p in data["patches"]),
                          "mesh_zero_set_max_error_m": max(p["zero_set_max_error_m"] for p in data["patches"]),
                          "serialized_zero_set_max_error_m": max(p["wire_rounding_max_error_m"] for p in data["patches"]),
                          "facet_sample_max_error_m": max(p["facet_sample_max_error_m"] for p in data["patches"]),
                          "patch_types": {k: sum(p["kind"] == k for p in data["patches"]) for k in ["quadric", "plane", "distribution"]}})
            if name == "hill" and frame == protocol["frames"]-1:
                default_data = data
        manifest.append({"id": name, "name": NAMES[name], "frames": protocol["frames"],
                         "source": scene["source"]["url"]})
        print(f"Exported {name}: {protocol['frames']} frames", flush=True)
    detail_id = figure(default_data, args.output)
    (args.output/"manifest.json").write_text(json.dumps({"scenes": manifest, "default_scene": "hill", "default_frame": 7,
        "detail_patch_id": detail_id, "source_protocol_sha256": digest(args.experiment/"protocol.json"),
        "note": "Saved experimental scans include 0.06m thinning, 15% dropout and 1.5cm added Z noise. Same labels and Taubin fitting as frozen benchmark; not the paper's original quadric solver or a complete FSGP reproduction."}, ensure_ascii=False, indent=2))
    report = {"passed": True, "frames": audit, "checks": ["Frozen experiment source hashes match",
        "All reconstructed valid masks and height predictions match the saved benchmark (<1e-10m)",
        "All analytic mesh vertices satisfy |F|/|grad F|<1e-7m before serialization",
        "Serialized mesh vertices satisfy |F|/|grad F|<2e-6m",
        "Each facet's centroid and edge midpoints satisfy |F|/|grad F|<=0.003002m",
        "Mesh is limited to each patch's tangent convex hull and measured 3D support; edges <=0.22m"],
        "exporter_sha256": digest(Path(__file__)), "source_protocol_sha256": digest(args.experiment/"protocol.json")}
    (args.output/"audit.json").write_text(json.dumps(report, indent=2))
    for path in (ROOT/"visualization/fit").iterdir():
        if path.is_file():
            shutil.copy2(path, args.output/path.name)
    vendor = ROOT.parent/"m20_3d_planner/visualization/assets/vendor"
    (args.output/"vendor").mkdir(exist_ok=True)
    for name in ["three.module.min.js", "OrbitControls.js", "LICENSE.txt", "README.md"]:
        shutil.copy2(vendor/name, args.output/"vendor"/name)
    if args.publish:
        args.publish.mkdir(parents=True, exist_ok=True)
        shutil.copytree(args.output, args.publish, dirs_exist_ok=True)
        add_home_entry(args.publish.parent/"index.html")
    print(f"Verified {len(audit)} frames; wrote {args.output}; served copy {args.publish}", flush=True)


if __name__ == "__main__":
    main()
