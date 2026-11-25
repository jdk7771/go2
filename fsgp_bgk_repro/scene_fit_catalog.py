"""Unified full-scene local quadratic experiments, served on port 42482.

Space out fit centers, retaining the full XYZ cloud for exact height-gated KNN.
The polynomial and consistency assembly match table_local_surfaces.py. This is
an additional quadratic experiment, not the FSGP paper's GP method.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial import cKDTree, ConvexHull, QhullError

from .experiment import make_scene
from .io import read_cloud
from .table_local_surfaces import LocalConfig, assemble, evaluate_models

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "outputs/visualization"
OUT = WEB / "scene-fits"
SEED_VOXEL = .12
NAMES = {"flat":"平地", "ramp":"斜坡", "steps":"高低组合多级台阶", "steps-single":"原单级台阶（历史）", "flat_ramp":"平地与斜坡组合",
         "hill":"丘陵", "forest":"森林", "quick":"Quick 地形", "desert":"沙漠",
         "mountain":"山地", "snow":"雪地", "vocano":"火山", "table":"大桌子",
         "combined":"丘陵、森林与桌子", "corridor-040":"独立通道 · 净空 0.40 m",
         "corridor-060":"独立通道 · 净空 0.60 m", "corridor-080":"独立通道 · 净空 0.80 m",
         "flat-corridor-platforms":"平地 · 通道与高低台", "surface-route-test":"A* 初版 · 矮台、立墙与顶板"}
NAMES.update({f'table-low-{n}':f'低桌 · 净空 {h:.2f} m' for n,h in [('040',.40),('042',.42),('060',.60)]})
NAMES['layer-choice']='封闭通道 · 降身或逐级上桥'
NAMES['four-platforms']='五平台通行链 · 绕行、下穿、降身、上台、小路'
NAMES.update({f'platform-underpass-{n}':f'高低台 · 第二台下方净空 {h:.2f} m · 向机器人右侧延伸'
              for n,h in [('070',.70),('042',.42)]})
ORDER = ["corridor-040", "corridor-060", "corridor-080", "combined", "table", "hill", "forest",
         "flat", "ramp", "steps", "steps-single", "flat_ramp", "quick", "desert", "mountain", "snow", "vocano",
         "flat-corridor-platforms", "surface-route-test"]
ORDER += ['table-low-040','table-low-042','table-low-060']
ORDER += ['layer-choice']
ORDER += ['platform-underpass-070','platform-underpass-042']
ORDER += ['four-platforms']
# Current compact assessment; ORDER above remains the historical scene index.
CURRENT_ASSESSMENT_SCENES = ("hill", "forest", "flat", "desert", "mountain", "snow", "vocano", "steps",
                             "flat-corridor-platforms", "surface-route-test", "table", "table-low-040", "table-low-042", "table-low-060", "layer-choice",
                             "platform-underpass-070", "platform-underpass-042", "four-platforms")


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_cloud(name):
    if name == "steps-single":
        return make_scene("steps").astype("<f4"), {"kind":"generated", "generator":"experiment.make_scene", "scene":"steps"}
    if name in ("flat", "ramp"):
        return make_scene(name).astype("<f4"), {"kind":"generated", "generator":"experiment.make_scene", "scene":name}
    if name == "steps":
        path = WEB / "mixed-steps/mixed_steps.npz"
    elif name == "flat_ramp":
        path = ROOT / "outputs/regression_fits/flat_ramp_clean.npz"
    elif name in ("hill", "quick"):
        path = ROOT / "data" / (name + ".pcd")
    elif name in ("forest", "desert", "mountain", "snow", "vocano"):
        path = ROOT.parent / "quadric_lidar_repro/data/fsgp_terrain" / (name + ".pcd")
    elif name == "table":
        path = WEB / "large-table/large_table.npz"
    elif name == "combined":
        path = WEB / "combined-scene/combined_scene.npz"
    elif name in ("flat-corridor-platforms", "surface-route-test", "layer-choice", "four-platforms") or name.startswith(('table-low-','platform-underpass-')):
        path = WEB / name / "scene.npz"
    else:
        path = WEB / name / "corridor_scene.npz"
    return read_cloud(path).astype("<f4"), {"kind":"file", "path":str(path.relative_to(ROOT.parent)), "sha256":file_sha(path)}


def exact_neighbors(points, seeds, config, batch_size=512):
    """Exact nearest K in the radius/height intersection, bounded working RAM.

    Expand ordinary 3D KNN until K eligible points are available. Refine around
    the Kth radius to include all distance ties; ties use the raw input index,
    matching the old sorted-radius-list implementation. No candidate cap.
    """
    tree = cKDTree(points)
    ids = np.full((len(seeds), config.neighbors), -1, dtype=np.int32)
    for start in range(0, len(seeds), batch_size):
        pending = np.arange(start, min(start + batch_size, len(seeds)))
        k = min(64, len(points))
        while len(pending):
            distance, nearest = tree.query(seeds[pending], k=k, distance_upper_bound=config.radius_m + 1e-12)
            if nearest.ndim == 1:
                nearest, distance = nearest[:, None], distance[:, None]
            exists = nearest < len(points)
            safe = np.minimum(nearest, len(points) - 1)
            eligible = exists & (np.abs(points[safe, 2] - seeds[pending, None, 2]) <= config.seed_height_gate_m)
            enough = eligible.sum(axis=1) >= config.neighbors
            complete = ~exists[:, -1] | (k == len(points))
            for row in np.flatnonzero(enough):
                seed = pending[row]
                radius = distance[row, np.flatnonzero(eligible[row])[config.neighbors - 1]]
                candidates = np.asarray(tree.query_ball_point(seeds[seed], min(config.radius_m + 1e-12, radius + 1e-10)), dtype=np.int32)
                delta = points[candidates] - seeds[seed]
                d2 = np.einsum("ni,ni->n", delta, delta)
                take = (np.abs(delta[:, 2]) <= config.seed_height_gate_m) & (d2 <= config.radius_m**2 + 1e-14)
                candidates, d2 = candidates[take], d2[take]
                ids[seed] = candidates[np.lexsort((candidates, d2))[:config.neighbors]]
            pending = pending[~enough & ~complete]
            k = min(len(points), k * 2)
    return ids


def fit_centers(points, seeds, ids, config):
    n = len(seeds)
    active = np.flatnonzero(ids[:, 0] >= 0)
    coefficient = np.full((n, 6), np.nan)
    height, rmse = np.full(n, np.nan), np.full(n, np.nan)
    rank = np.zeros(n, dtype=bool)
    if len(active):
        local = points[ids[active]]
        delta = local[:, :, :2] - seeds[active, None, :2]
        scale = np.maximum(np.sqrt(np.mean(np.sum(delta**2, axis=2), axis=1)), 1e-10)
        u, v = (delta / scale[:, None, None]).transpose(2, 0, 1)
        design = np.stack((np.ones_like(u), u, v, u*u, u*v, v*v), axis=2)
        gram = np.einsum("nki,nkj->nij", design, design)
        spectrum = np.linalg.eigvalsh(gram)
        rank[active] = spectrum[:, 0] > np.maximum(spectrum[:, -1], 1e-30) * config.rank_ratio
        target = local[:, :, 2] - seeds[active, None, 2]
        rhs = np.einsum("nki,nk->ni", design, target)
        ridge = config.relative_ridge * np.trace(gram, axis1=1, axis2=2) / 6
        normalized = np.linalg.solve(gram + ridge[:, None, None]*np.eye(6), rhs)
        coefficient[active] = normalized / np.column_stack((np.ones(len(active)), scale, scale, scale**2, scale**2, scale**2))
        prediction = seeds[active, 2] + normalized[:, 0]
        residual = np.einsum("nki,ni->nk", design, normalized) - target
        rmse[active] = np.sqrt(np.mean(residual**2, axis=1))
        valid = rank[active] & (np.abs(prediction-seeds[active, 2]) <= config.seed_height_gate_m) & (rmse[active] <= config.max_fit_rmse_m)
        height[active[valid]] = prediction[valid]
    normal = np.column_stack((-coefficient[:, 1], -coefficient[:, 2], np.ones(n)))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    return dict(neighbor_ids=ids, coefficient=coefficient, height=height, rmse=rmse, full_rank=rank, normal=normal)


def pipeline(points, config=LocalConfig()):
    start = time.perf_counter()
    _, source_ids = np.unique(np.floor(points / SEED_VOXEL).astype(np.int64), axis=0, return_index=True)
    # Work in float64 after fixing the delivered full input cloud in float32.
    full = points.astype(float)
    seeds = full[source_ids]
    selected = time.perf_counter()
    ids = exact_neighbors(full, seeds, config)
    neighbors_done = time.perf_counter()
    fits = fit_centers(full, seeds, ids, config)
    fits_done = time.perf_counter()
    labels, components, edges, summary = assemble(seeds, fits, config)
    finished = time.perf_counter()
    times = dict(seed_selection_ms=(selected-start)*1000, neighbor_query_ms=(neighbors_done-selected)*1000,
                 solve_ms=(fits_done-neighbors_done)*1000, assembly_ms=(finished-fits_done)*1000,
                 xyz_to_mesh_ms=(finished-start)*1000)
    return dict(points=seeds, source_ids=source_ids, fits=fits, labels=labels,
                components=components, edges=edges, summary=summary, timings=times)


def patch_geometry(full, result, limit=3000):
    seeds, fits = result["points"], result["fits"]
    valid = np.flatnonzero(np.isfinite(fits["height"]))
    shown = valid[np.linspace(0, len(valid)-1, min(len(valid), limit), dtype=int)] if len(valid) else valid
    vertices, triangles, owners = [], [], []
    axis = np.linspace(-1, 1, 5)
    a, b = np.meshgrid(axis, axis)
    offsets = np.column_stack((a.ravel(), b.ravel()))
    ix = (np.arange(4)[:, None]*5 + np.arange(4)).ravel()
    faces = np.concatenate((np.column_stack((ix, ix+1, ix+5)), np.column_stack((ix+1, ix+6, ix+5))))
    for seed in shown:
        delta = full[fits["neighbor_ids"][seed], :2] - seeds[seed, :2]
        try:
            hull = ConvexHull(delta)
        except QhullError:
            continue
        half = np.minimum(.12, np.maximum(np.max(np.abs(delta), axis=0)*.9, 1e-5))
        xy_delta = offsets * half
        inside = np.all(xy_delta @ hull.equations[:, :2].T + hull.equations[:, 2] <= 1e-10, axis=1)
        xy = xy_delta + seeds[seed, :2]
        z = evaluate_models(seeds, fits["coefficient"], np.full(25, seed), xy)
        inside &= np.abs(z-seeds[seed, 2]) <= .04
        keep = faces[np.all(inside[faces], axis=1)]
        start = len(vertices)
        vertices.extend(np.column_stack((xy, z)))
        owners.extend([seed]*25)
        triangles.extend(keep+start)
    return (np.asarray(vertices, dtype="<f4").reshape(-1, 3),
            np.asarray(triangles, dtype="<u4").reshape(-1, 3),
            np.asarray(owners, dtype="<u4"), shown)


def statistics(values):
    values = values[np.isfinite(values)]
    return {"count": len(values), "median_m":float(np.median(values)), "p95_m":float(np.percentile(values, 95)), "max_m":float(values.max())} if len(values) else None


def export_status_patches(output):
    """Export every valid local patch for fit-versus-assembly color comparison."""
    output = Path(output)
    meta = json.loads((output / "data.json").read_text())
    before = file_sha(output / "results.npz")
    full = np.fromfile(output / "points.bin", dtype="<f4").reshape(-1, 3)
    with np.load(output / "results.npz", allow_pickle=False) as saved:
        fits = dict(coefficient=saved["coefficients"], height=saved["height"], neighbor_ids=saved["neighbor_ids"])
        result = dict(points=saved["points"], fits=fits)
        p, faces, owners, shown = patch_geometry(full, result, limit=meta["seed_count"])
        used = np.zeros(meta["seed_count"], dtype=bool)
        used[np.unique(saved["triangles"])] = True
        valid = np.isfinite(fits["height"])
        np.testing.assert_array_equal(shown, np.flatnonzero(valid))
        assert np.all(valid[owners]) and np.all(used <= valid)
        assert np.all(owners[faces] == owners[faces[:, :1]])
        represented = np.unique(owners[faces])
        meta["status_patches"] = dict(
            valid_functions=int(valid.sum()), functions_with_display_triangles=len(represented),
            valid_without_display_triangles=int(valid.sum())-len(represented),
            assembled_functions=int(used.sum()), valid_unassembled_functions=int((valid & ~used).sum()),
            all_centers=meta["seed_count"], vertices=len(p), triangles=len(faces),
            geometry="All valid independent polynomial patches; same support clipping and 5x5 display grid, no 3000-function sampling cap",
            assembly_definition="A function is assembled iff its seed appears in at least one final retained triangle; some of its other candidate triangles can still be rejected",
            missing="Invalid/unsolved functions have no fabricated surface; status-colored center markers represent them",
            binary_files=["status-patches.bin", "status-patch-faces.bin", "status-patch-owners.bin"])
    p.tofile(output / "status-patches.bin")
    faces.tofile(output / "status-patch-faces.bin")
    owners.tofile(output / "status-patch-owners.bin")
    assert file_sha(output / "results.npz") == before
    (output / "data.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2)+"\n")
    (output / "status-patch-audit.json").write_text(json.dumps(dict(
        passed=True, all_valid_functions_included=True, no_sampling=True,
        all_triangle_vertices_same_owner=True, original_result_unchanged=True,
        result_sha256=before, counts=meta["status_patches"]), ensure_ascii=False, indent=2)+"\n")
    return meta


def export_max_errors(output):
    """Add per-function max absolute training errors without changing any fits."""
    output = Path(output)
    meta = json.loads((output / "data.json").read_text())
    result_sha = file_sha(output / "results.npz")
    full = np.fromfile(output / "points.bin", dtype="<f4").reshape(-1, 3).astype(float)
    with np.load(output / "results.npz", allow_pickle=False) as saved:
        seeds, neighbors = saved["points"], saved["neighbor_ids"]
        coefficients, rmse = saved["coefficients"], saved["rmse"]
        valid = np.isfinite(saved["height"])
        active = np.flatnonzero(neighbors[:, 0] >= 0)
        maximum = np.full(len(seeds), np.nan)
        signed = np.full(len(seeds), np.nan)
        worst_point = np.full(len(seeds), -1, dtype=np.int32)
        for start in range(0, len(active), 4096):
            ids = active[start:start + 4096]
            delta = full[neighbors[ids]] - seeds[ids, None, :]
            u, v = delta[:, :, 0], delta[:, :, 1]
            c = coefficients[ids]
            residual = (c[:, None, 0] + c[:, None, 1]*u + c[:, None, 2]*v
                        + c[:, None, 3]*u*u + c[:, None, 4]*u*v
                        + c[:, None, 5]*v*v - delta[:, :, 2])
            np.testing.assert_allclose(np.sqrt(np.mean(residual**2, axis=1)), rmse[ids], atol=1e-11)
            rows = np.arange(len(ids))
            worst = np.argmax(np.abs(residual), axis=1)
            signed[ids] = residual[rows, worst]
            maximum[ids] = np.abs(signed[ids])
            worst_point[ids] = neighbors[ids, worst]
        assert np.isfinite(maximum[active]).all()
        assert np.isnan(maximum[neighbors[:, 0] < 0]).all()
        assert np.all(maximum[active] >= rmse[active] - 1e-11)
        maximum.astype("<f8").tofile(output / "max-error.bin")
        with (output / "max-error.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(["seed", "x", "y", "z", "solved", "valid", "training_max_abs_error_m",
                             "training_rmse_m", "worst_neighbor_point", "worst_observed_z_m",
                             "worst_predicted_z_m", "worst_signed_residual_m"])
            for i, p in enumerate(seeds):
                solved = worst_point[i] >= 0
                observed = full[worst_point[i], 2] if solved else np.nan
                writer.writerow([i, *p, int(solved), int(valid[i]),
                                 maximum[i] if solved else "", rmse[i] if solved else "",
                                 int(worst_point[i]) if solved else "", observed if solved else "",
                                 observed + signed[i] if solved else "", signed[i] if solved else ""])
        top = int(np.nanargmax(maximum)) if len(active) else None
        meta["max_abs_error"] = dict(
            definition="max_j(abs(f_i(x_j,y_j)-z_j)) over the same 30 training neighbors; vertical error in meters",
            binary_file="max-error.bin", binary_type="little-endian float64", csv_file="max-error.csv",
            all=statistics(maximum), valid=statistics(maximum[valid]), worst_seed=top,
            worst_valid_seed=int(np.flatnonzero(valid)[np.argmax(maximum[valid])]) if np.any(valid) else None,
            color_scale_max_m=max(float(np.nanmax(maximum)), 1e-9) if len(active) else 1e-9,
            unsolved="NaN in binary, empty fields in CSV; never zero-filled",
            caveat="Training residual, not independent accuracy or adjacent-model disagreement. Rejected solved fits retain errors.")
    assert file_sha(output / "results.npz") == result_sha
    (output / "data.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
    checks = dict(passed=True, functions=len(seeds), solved=len(active),
                  all_training_rmse_reproduced=True, missing_unsolved_errors=True,
                  result_unchanged=True, result_sha256=result_sha)
    (output / "max-error-audit.json").write_text(json.dumps(checks, indent=2) + "\n")
    return meta


def validate(full, result, config):
    seeds, fits = result["points"], result["fits"]
    solved = np.flatnonzero(fits["neighbor_ids"][:, 0] >= 0)
    ids = fits["neighbor_ids"][solved]
    delta = full[ids] - seeds[solved, None, :]
    assert np.all(np.abs(delta[:, :, 2]) <= config.seed_height_gate_m + 1e-12)
    assert np.all(np.sum(delta**2, axis=2) <= config.radius_m**2 + 1e-12)
    assert np.all(np.any(ids == result["source_ids"][solved, None], axis=1))
    c = fits["coefficient"][solved]
    u, v = delta[:, :, 0], delta[:, :, 1]
    prediction = seeds[solved, None, 2]+c[:, None, 0]+c[:, None, 1]*u+c[:, None, 2]*v+c[:, None, 3]*u*u+c[:, None, 4]*u*v+c[:, None, 5]*v*v
    residual = prediction-full[ids, 2]
    np.testing.assert_allclose(np.sqrt(np.mean(residual**2, axis=1)), fits["rmse"][solved], atol=1e-11)
    for i, component in enumerate(result["components"]):
        assert np.all(result["labels"][component["triangles"]] == i)
        assert np.isfinite(fits["height"][component["triangles"]]).all()
    return dict(passed=True, exact_30_neighbors=True, radius_and_height_gate_pass=True,
                seed_included=True, all_residuals_recomputed=True, no_cross_component_triangles=True,
                labels_or_preset_heights_used=False, full_xyz_support_cloud=True)


def export_scene(name, repeats=3):
    output = OUT / name
    output.mkdir(parents=True, exist_ok=True)
    points, source = source_cloud(name)
    config = LocalConfig()
    times, result = [], None
    # Each repeat recomputes center selection, tree, neighbors, fits and assembly.
    for _ in range(repeats):
        current = pipeline(points, config)
        times.append(current["timings"])
        if result is None:
            result = current
        else:
            np.testing.assert_array_equal(current["fits"]["height"], result["fits"]["height"])
    seeds, fit = result["points"], result["fits"]
    audit = validate(points.astype(float), result, config)
    patches, patch_faces, owners, shown = patch_geometry(points, result)
    solved = fit["neighbor_ids"][:, 0] >= 0
    valid = np.isfinite(fit["height"])
    radius = np.full(len(seeds), np.nan)
    radius[solved] = np.linalg.norm(points[fit["neighbor_ids"][solved], :2]-seeds[solved, None, :2], axis=2).max(axis=1)
    packed = np.column_stack((seeds, fit["coefficient"], fit["height"], fit["rmse"], result["labels"], valid.astype(int), radius)).astype("<f8")
    assert packed.shape == (len(seeds), 14)
    points.tofile(output / "points.bin")
    packed.tofile(output / "seeds.bin")
    fit["neighbor_ids"].astype("<i4").tofile(output / "neighbors.bin")
    patches.tofile(output / "patches.bin")
    patch_faces.tofile(output / "patch-faces.bin")
    owners.tofile(output / "patch-owners.bin")
    components, faces, offset = [], [], 0
    for i, component in enumerate(result["components"]):
        ids = component["point_ids"]
        components.append(dict(id=i, points=len(ids), median_z=float(np.median(fit["height"][ids])),
                               triangle_start=offset, triangle_count=len(component["triangles"])))
        faces.append(component["triangles"])
        offset += len(component["triangles"])
    triangles = np.concatenate(faces).astype("<u4") if faces else np.empty((0, 3), dtype="<u4")
    triangles.tofile(output / "mesh.bin")
    np.savez_compressed(output / "results.npz", points=seeds, source_ids=result["source_ids"],
                        coefficients=fit["coefficient"], height=fit["height"], rmse=fit["rmse"],
                        full_rank=fit["full_rank"], neighbor_ids=fit["neighbor_ids"],
                        labels=result["labels"], triangles=triangles)
    with (output / "per-fit.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["seed", "source_point", "x", "y", "z", "solved", "valid", "rank_ok", "training_rmse_m", "neighbor_xy_radius_m", "component"])
        for i, p in enumerate(seeds):
            writer.writerow([i, result["source_ids"][i], *p, int(solved[i]), int(valid[i]), int(fit["full_rank"][i]),
                             float(fit["rmse"][i]) if solved[i] else "", float(radius[i]) if solved[i] else "", result["labels"][i]])
    meta = dict(id=name, name=NAMES[name], raw_points=len(points), seed_count=len(seeds),
                solved=int(solved.sum()), valid=int(valid.sum()), insufficient=int((~solved).sum()),
                rejected=int((solved & ~valid).sum()), components=components, triangles=len(triangles),
                independent_patches_shown=len(shown), seed_voxel_m=SEED_VOXEL,
                settings=asdict(config), bounds=[points.min(0).tolist(), points.max(0).tolist()],
                training_residual_all=statistics(fit["rmse"]), training_residual_valid=statistics(fit["rmse"][valid]),
                neighbor_xy_radius_valid=statistics(radius[valid]), source=source,
                timings=dict(repeats=times, medians={key:float(np.median([t[key] for t in times])) for key in times[0]},
                             repetitions=repeats, includes="XYZ to seed selection, full-cloud tree and neighbors, quadratic solves, consistency assembly",
                             excludes="file IO, patch display geometry, serialization, HTTP, browser rendering"),
                binary=dict(seed_columns=14, seed_type="little-endian float64", neighbor_columns=30,
                            point_type="little-endian float32"),
                polynomial="z=z_seed+c0+c1*u+c2*v+c3*u*u+c4*u*v+c5*v*v; u=x-x_seed, v=y-y_seed",
                caveat="Training residual is not independent test accuracy. z=f(x,y) does not reliably represent vertical walls; rejected observations remain visible. No FSGP-paper accuracy claim.")
    if name == "steps":
        meta["scene_geometry"] = json.loads((WEB / "mixed-steps/scene.json").read_text())
    (output / "data.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
    (output / "audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    meta = export_max_errors(output)
    meta = export_status_patches(output)
    print(json.dumps({"scene":name, "raw":len(points), "seeds":len(seeds), "valid":int(valid.sum()), "triangles":len(triangles), "ms":meta["timings"]["medians"]["xyz_to_mesh_ms"]}, ensure_ascii=False), flush=True)
    return meta


def publish_index():
    cases = []
    for name in ORDER:
        path = OUT / name / "data.json"
        if path.exists():
            cases.append(json.loads(path.read_text()))
    catalog = {"port":42482, "scenes":cases, "default_scene":"corridor-040", "method":"local bivariate quadratic z=f(x,y), exact K=30 from full XYZ support"}
    (OUT / "catalog.json").write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + "\n")
    assets = ROOT / "visualization/catalog"
    for name in ("index.html", "viewer.js", "viewer.css"):
        if (assets / name).exists():
            shutil.copyfile(assets / name, OUT / name)
    (OUT / "vendor").mkdir(exist_ok=True)
    for name in ("three.module.min.js", "OrbitControls.js", "LICENSE.txt"):
        shutil.copyfile(WEB / "large-table/vendor" / name, OUT / "vendor" / name)
    # Make port 42482 itself the unified viewer, retaining the complete old
    # home page and every existing deep link as independently reachable files.
    template = assets / "index.html"
    if template.exists():
        home, archive = WEB / "index.html", WEB / "legacy-results.html"
        if home.exists() and not archive.exists():
            shutil.copyfile(home, archive)
        # A cold checkout builds the historical fit inputs before the current
        # convex-domain assessment. Publish the homepage only when it is ready.
        if (WEB / 'convex-assessment/profiles.json').exists():
            from .publish_metric_viewer import publish
            publish()
    with (OUT / "summary.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scene", "raw_points", "seed_count", "solved", "valid", "rejected", "triangles", "median_training_rmse_m", "xyz_to_mesh_median_ms"])
        for case in cases:
            writer.writerow([case["id"], *[case[k] for k in ["raw_points", "seed_count", "solved", "valid", "rejected", "triangles"]],
                             (case["training_residual_valid"] or {}).get("median_m", ""), case["timings"]["medians"]["xyz_to_mesh_ms"]])
    return catalog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", nargs="+", choices=ORDER, default=ORDER)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    OUT.mkdir(parents=True, exist_ok=True)
    for name in args.scenes:
        export_scene(name, args.repeats)
        publish_index()


if __name__ == "__main__":
    main()
