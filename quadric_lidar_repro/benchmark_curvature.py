"""Validate analytic geometry and compare matched-query CPU latencies.

Run with PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
MKL_NUM_THREADS=1 /usr/bin/python3 -m quadric_lidar_repro.benchmark_curvature
"""

import argparse
import csv
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import time

import numpy as np
import scipy
from scipy.spatial import cKDTree

from .core import Config, Moments, fit_patch
from .curvature import differential_geometry, local_geometry
from .segmentation import segment_scan

ROOT = Path(__file__).resolve().parent
METHODS = ("schur", "taubin", "taubin_general", "jet", "jet256", "pca")
SHAPES = ("plane", "sphere", "cylinder", "saddle", "ellipsoid", "shallow_sphere", "ripple")


def surface(shape, xy):
    """Independent graph-based truth from first/second fundamental forms."""
    x, y = np.asarray(xy).T
    zero = np.zeros_like(x)
    if shape == "plane":
        z = .3*x-.2*y
        fx, fy, fxx, fxy, fyy = zero+.3, zero-.2, zero, zero, zero
    elif shape in ("sphere", "shallow_sphere"):
        radius = 2. if shape == "sphere" else 10.
        z = np.sqrt(radius**2-x*x-y*y)
        fx, fy = -x/z, -y/z
        fxx, fxy, fyy = -1/z-x*x/z**3, -x*y/z**3, -1/z-y*y/z**3
    elif shape == "cylinder":
        z = np.sqrt(4-x*x)
        fx, fy, fxx, fxy, fyy = -x/z, zero, -1/z-x*x/z**3, zero, zero
    elif shape == "saddle":
        z = .35*x*x-.25*y*y
        fx, fy, fxx, fxy, fyy = .7*x, -.5*y, zero+.7, zero, zero-.5
    elif shape == "ellipsoid":
        a, b, c = 2., 1.5, 1.
        s = np.sqrt(1-(x/a)**2-(y/b)**2)
        z = c*s
        fx, fy = -c*x/(a*a*s), -c*y/(b*b*s)
        fxx = -c/(a*a*s)-c*x*x/(a**4*s**3)
        fxy = -c*x*y/(a*a*b*b*s**3)
        fyy = -c/(b*b*s)-c*y*y/(b**4*s**3)
    elif shape == "ripple":
        z = .12*np.sin(3*x)*np.cos(2*y)
        fx, fy = .36*np.cos(3*x)*np.cos(2*y), -.24*np.sin(3*x)*np.sin(2*y)
        fxx, fxy, fyy = (-1.08*np.sin(3*x)*np.cos(2*y),
                         -.72*np.cos(3*x)*np.sin(2*y), -.48*np.sin(3*x)*np.cos(2*y))
    else:
        raise ValueError(shape)
    normal = np.column_stack((-fx, -fy, np.ones_like(x)))
    norm = np.linalg.norm(normal, axis=1)
    normal /= norm[:, None]
    mean = ((1+fy*fy)*fxx-2*fx*fy*fxy+(1+fx*fx)*fyy)/(2*norm**3)
    gaussian = (fxx*fyy-fxy*fxy)/norm**4
    spread = np.sqrt(np.maximum(mean*mean-gaussian, 0))
    return np.column_stack((x, y, z))+[8., -3., .7], {
        "normal": normal, "principal": np.column_stack((mean-spread, mean+spread)),
        "slope_deg": np.degrees(np.arctan(np.hypot(fx, fy))),
    }


def score(estimate, truth):
    valid = estimate["valid"] & np.isfinite(estimate["normal"]).all(axis=1)
    metrics = {"valid_fraction": float(np.mean(valid))}
    if not valid.any():
        return {**metrics, "normal_rmse_deg": None, "slope_mae_deg": None,
                "curvature_rmse_per_m": None, "curvature_mae_per_m": None}
    dot = np.sum(estimate["normal"][valid]*truth["normal"][valid], axis=1)
    angle = np.degrees(np.arccos(np.clip(np.abs(dot), 0, 1)))
    metrics["normal_rmse_deg"] = float(np.sqrt(np.mean(angle*angle)))
    metrics["slope_mae_deg"] = float(np.mean(np.abs(estimate["slope_deg"][valid]-truth["slope_deg"][valid])))
    principal = estimate["principal"][valid].copy()
    flip = dot < 0
    principal[flip] = -principal[flip, ::-1]
    if np.isfinite(principal).all():
        error = principal-truth["principal"][valid]
        metrics["curvature_rmse_per_m"] = float(np.sqrt(np.mean(error*error)))
        metrics["curvature_mae_per_m"] = float(np.mean(np.abs(error)))
    else:
        metrics["curvature_rmse_per_m"] = metrics["curvature_mae_per_m"] = None
    return metrics


def time_paired(functions, repeats, warmup=3):
    for function in functions.values():
        for _ in range(warmup):
            function()
    rng = np.random.default_rng(71)
    names = list(functions)
    samples = {name: [] for name in names}
    for _ in range(repeats):
        for index in rng.permutation(len(names)):
            name = names[index]
            start = time.perf_counter_ns()
            result = functions[name]()
            samples[name].append((time.perf_counter_ns()-start)/1e6)
            del result
    return {name: {"median_ms": float(np.median(values)), "p95_ms": float(np.percentile(values, 95)),
                   "samples_ms": values, "repeats": repeats} for name, values in samples.items()}


def write_csv(path, rows):
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader(); writer.writerows(rows)


def accuracy_experiment(output, config, args):
    rows, examples = [], {}
    axis = np.linspace(-.65, .65, args.query_side)
    u, v = np.meshgrid(axis, axis)
    xy_query = np.column_stack((u.ravel(), v.ravel()))
    configs = {"schur": config, "taubin": replace(config, fitting="taubin"),
               "taubin_general": replace(config, fitting="taubin", planar_ratio=0.)}
    for shape in SHAPES:
        query, truth = surface(shape, xy_query)
        for sigma in args.noise:
            for seed in range(args.seeds):
                rng = np.random.default_rng(700+seed)
                clean, train_truth = surface(shape, rng.uniform(-1, 1, (args.points, 2)))
                cloud = clean+rng.normal(0, sigma, (args.points, 1))*train_truth["normal"]
                moments = Moments.from_points(cloud)
                for method in METHODS:
                    if method in configs:
                        patch = fit_patch(moments, configs[method])
                        estimate = differential_geometry(patch, query)
                        kind = patch.kind
                    else:
                        estimate = local_geometry(cloud, query, 256 if method == "jet256" else args.neighbors,
                                                  mode="jet" if method == "jet256" else method)
                        kind = "local_pca" if method == "pca" else "local_polynomial"
                    row = {"shape": shape, "noise_m": sigma, "seed": seed, "method": method,
                           "fit_kind": kind, "training_points": len(cloud), "queries": len(query),
                           **score(estimate, truth)}
                    rows.append(row)
                    if seed == 0 and np.isclose(sigma, .01):
                        examples[f"{shape}_{method}_principal"] = estimate["principal"]
                        examples[f"{shape}_{method}_valid"] = estimate["valid"]
                        examples[f"{shape}_{method}_slope"] = estimate["slope_deg"]
                if seed == 0 and np.isclose(sigma, .01):
                    examples[shape+"_query"] = query
                    examples[shape+"_cloud"] = cloud
                    examples[shape+"_truth_principal"] = truth["principal"]
                    examples[shape+"_truth_slope"] = truth["slope_deg"]
            selected = [r for r in rows if r["shape"] == shape and r["noise_m"] == sigma and r["method"] == "schur"]
            error = [r["curvature_rmse_per_m"] for r in selected if r["curvature_rmse_per_m"] is not None]
            print(f"Accuracy {shape:14} sigma={sigma:.3f}: Schur k-RMSE "
                  f"{np.mean(error) if error else float('nan'):.5f} 1/m, kinds={sorted(set(r['fit_kind'] for r in selected))}", flush=True)
    write_csv(output/"accuracy.csv", rows)
    np.savez_compressed(output/"examples.npz", **examples)
    return rows


def speed_experiment(config, args):
    rng = np.random.default_rng(8)
    clean, truth = surface("sphere", rng.uniform(-1, 1, (args.points, 2)))
    cloud = clean+rng.normal(0, .01, (len(clean), 1))*truth["normal"]
    patch = fit_patch(Moments.from_points(cloud), config)
    taubin_config = replace(config, fitting="taubin")
    taubin_patch = fit_patch(Moments.from_points(cloud), taubin_config)
    if patch.kind != "quadric":
        raise AssertionError("speed scene must use a general quadric")
    tree = cKDTree(cloud)
    rows = []
    for count in args.speed_queries:
        query, _ = surface("sphere", rng.uniform(-.65, .65, (count, 2)))
        functions = {
            "analytic_derivatives_only": lambda: differential_geometry(patch, query, project=False),
            "analytic_with_projection": lambda: differential_geometry(patch, query),
            "quadric_fit_and_geometry": lambda: differential_geometry(fit_patch(Moments.from_points(cloud), config), query),
            "taubin_existing_model": lambda: differential_geometry(taubin_patch, query),
            "taubin_fit_and_geometry": lambda: differential_geometry(fit_patch(Moments.from_points(cloud), taubin_config), query),
            "local_jet_full": lambda: local_geometry(cloud, query, args.neighbors),
            "local_jet256_full": lambda: local_geometry(cloud, query, 256),
            "local_jet_reuse_tree": lambda: local_geometry(cloud, query, args.neighbors, tree=tree),
            "pca_normal_slope_only": lambda: local_geometry(cloud, query, args.neighbors, mode="pca"),
        }
        repeats = args.speed_repeats if count <= 1000 else max(10, args.speed_repeats//4)
        row = {"training_points": len(cloud), "queries": count, "neighbors": args.neighbors,
               "timings": time_paired(functions, repeats)}
        rows.append(row)
        print(f"Speed {count:5} queries: model+projection {row['timings']['analytic_with_projection']['median_ms']:.3f} ms, "
              f"fit+geometry {row['timings']['quadric_fit_and_geometry']['median_ms']:.3f} ms, "
              f"local jet {row['timings']['local_jet_full']['median_ms']:.3f} ms", flush=True)
    return rows


def real_experiment(output, config, args):
    rows = []
    for frame in args.frames:
        path = ROOT/f"data/kitti/sequences/04/velodyne/{frame:06}.bin"
        points = np.fromfile(path, dtype="<f4").reshape(-1, 4)[:, :3]
        patches, lists, stats = segment_scan(points, config)
        taubin_config = replace(config, fitting="taubin")
        taubin_patches = [fit_patch(p.moments, taubin_config) for p in patches]
        def analytic_full():
            return [differential_geometry(fit_patch(Moments.from_points(p), config), p) for p in lists]
        functions = {
            "fit_only": lambda: [fit_patch(Moments.from_points(p), config) for p in lists],
            "analytic_existing_models": lambda: [differential_geometry(patch, p) for patch, p in zip(patches, lists)],
            "quadric_fit_and_geometry": analytic_full,
            "taubin_existing_models": lambda: [differential_geometry(patch, p) for patch, p in zip(taubin_patches, lists)],
            "taubin_fit_and_geometry": lambda: [differential_geometry(fit_patch(Moments.from_points(p), taubin_config), p) for p in lists],
            "local_jet_full": lambda: [local_geometry(p, p, args.neighbors) for p in lists],
            "pca_normal_slope_only": lambda: [local_geometry(p, p, args.neighbors, mode="pca") for p in lists],
        }
        timings = time_paired(functions, args.frame_repeats, warmup=1)
        result = functions["analytic_existing_models"]()
        taubin_valid = np.concatenate([r["valid"] for r in functions["taubin_existing_models"]()])
        valid = np.concatenate([r["valid"] for r in result])
        row = {"frame": frame, "input_points": len(points), **stats, "query_points": sum(map(len, lists)),
               "valid_fraction": float(np.mean(valid)), "taubin_valid_fraction": float(np.mean(taubin_valid)), "timings": timings,
               "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        rows.append(row)
        if frame == args.frames[0]:
            np.savez_compressed(output/"kitti_geometry.npz", points=np.vstack(lists), valid=valid,
                                principal=np.vstack([r["principal"] for r in result]),
                                slope_deg=np.concatenate([r["slope_deg"] for r in result]))
        print(f"KITTI/{frame:06}: {row['query_points']} point queries, fit+geometry "
              f"{timings['quadric_fit_and_geometry']['median_ms']:.1f} ms, local jet "
              f"{timings['local_jet_full']['median_ms']:.1f} ms; valid {row['valid_fraction']:.2%}", flush=True)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/curvature_validation")
    parser.add_argument("--points", type=int, default=4096)
    parser.add_argument("--query-side", type=int, default=20)
    parser.add_argument("--noise", nargs="+", type=float, default=[0, .002, .01, .03])
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--neighbors", type=int, default=64)
    parser.add_argument("--speed-queries", nargs="+", type=int, default=[1, 100, 1000, 10000])
    parser.add_argument("--speed-repeats", type=int, default=40)
    parser.add_argument("--frames", nargs="*", type=int, default=[0, 120, 240])
    parser.add_argument("--frame-repeats", type=int, default=5)
    args = parser.parse_args()
    if min(args.points, args.neighbors) < 6 or min(args.query_side, args.seeds, args.speed_repeats, args.frame_repeats, *args.speed_queries) < 1:
        parser.error("need >=6 support points/neighbors and positive counts")
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        if os.environ.get(variable) != "1":
            parser.error(f"set {variable}=1 before launching Python")
    args.output.mkdir(parents=True, exist_ok=True)
    config = Config()
    accuracy = accuracy_experiment(args.output, config, args)
    speed = speed_experiment(config, args)
    real = real_experiment(args.output, config, args)
    cpu = next(line.split(":", 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
               if line.startswith("model name"))
    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"cpu": cpu, "python": platform.python_version(), "numpy": np.__version__,
                        "scipy": scipy.__version__, "blas_threads": 1, "platform": platform.platform()},
        "protocol": {"config": config.as_dict(), "args": {k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()},
                     "support": "Each synthetic dataset is ONE pre-segmented patch over [-1,1]^2; independent queries over [-.65,.65]^2.",
                     "noise": "Independent Gaussian displacement along clean surface normal. Same points and queries for every method.",
                     "normal_orientation": "Unsigned normal angle; principal curvatures sign-aligned to truth solely for scoring.",
                     "geometry": "Four Newton normal-projection steps then analytic gradient/Hessian/tangent 2x2 shape operator.",
                     "baseline": "Local 64-NN and 256-NN PCA-frame degree-2 polynomial, batched normalized normal equations. Not CGAL binaries.",
                     "pca": "Normal and slope only; surface variation is dimensionless and NOT compared to physical curvature.",
                     "taubin_general": "Ablation planar_ratio=0, fitting=taubin; original pipeline and default thresholds unchanged.",
                     "speed": "Warm cache; randomized interleaved order; construction included, destruction excluded; no disk I/O or segmentation.",
                     "real": "Same pre-segmented supports for all methods; query ALL retained points. No real curvature ground truth.",
                     "limitations": "No outliers, occlusion or segmentation errors in synthetic scenes; no claim of full odometry accuracy or segmentation robustness."},
        "sources": [{"title": "Goldman (2005), Curvature formulas for implicit curves and surfaces",
                     "url": "https://www.cgeo.uliege.be/CAO/Goldman_Curvature_formulas_implicit.pdf"},
                    {"title": "CGAL Jet_fitting_3 official manual", "url": "https://doc.cgal.org/latest/Jet_fitting_3/index.html"}],
        "source_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
                          ("core.py", "curvature.py", "benchmark_curvature.py", "curvature_report.py", "segmentation.py", "segment.cpp")},
        "accuracy": accuracy, "speed": speed, "real": real,
    }
    (args.output/"metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    speed_rows = [{"queries": r["queries"], "stage": name, "median_ms": t["median_ms"], "p95_ms": t["p95_ms"], "repeats": t["repeats"]}
                  for r in speed for name,t in r["timings"].items()]
    write_csv(args.output/"speed.csv", speed_rows)
    if real:
        write_csv(args.output/"kitti_speed.csv", [{"frame": r["frame"], "queries": r["query_points"], "stage": name,
                  "median_ms": t["median_ms"], "p95_ms": t["p95_ms"]} for r in real for name,t in r["timings"].items()])
    from .curvature_report import create_report
    create_report(args.output, result)
    print(f"Saved {args.output}", flush=True)


if __name__ == "__main__":
    main()
