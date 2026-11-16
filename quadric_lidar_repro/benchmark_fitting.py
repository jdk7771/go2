"""Warm-cache CPU benchmark of fitting, excluding segmentation and registration.

Run with single-threaded BLAS, e.g. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 /usr/bin/python3 -m
quadric_lidar_repro.benchmark_fitting
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

from .core import Config, Moments, distance_and_gradient, fit_patch
from .segmentation import segment_scan

ROOT = Path(__file__).resolve().parent


def measure(call, repeats, warmup=8):
    for _ in range(warmup):
        call()
    samples = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        result = call()
        samples.append((time.perf_counter_ns()-start)/1000)
        # Include construction in the timer, but release the result outside it.
        del result
    return {
        "repeats": repeats,
        "median_us": float(np.median(samples)),
        "p95_us": float(np.percentile(samples, 95)),
        "mean_us": float(np.mean(samples)),
        "samples_us": samples,
    }


def synthetic_points(shape, count, seed):
    rng = np.random.default_rng(seed)
    if shape == "sphere":
        directions = rng.normal(size=(count, 3))
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        points = directions*(2+rng.normal(0, .002, (count, 1)))
    else:
        xy = rng.uniform(-2, 2, (count, 2))
        points = np.column_stack((xy, .3*xy[:, 0]-.2*xy[:, 1]+rng.normal(0, .002, count)))
    return points+np.array([8., -3., .7])


def summarize(values):
    return {"median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "min": float(np.min(values)), "max": float(np.max(values))}


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/fitting_speed")
    parser.add_argument("--repeats", type=int, default=300)
    parser.add_argument("--frame-repeats", type=int, default=15)
    parser.add_argument("--frames", nargs="+", type=int, default=list(range(0, 271, 30)))
    parser.add_argument("--sizes", nargs="+", type=int, default=[50, 100, 500, 1000, 5000, 10000, 100000])
    args = parser.parse_args()
    if min(args.repeats, args.frame_repeats, *args.sizes) < 1:
        parser.error("repeats and point counts must be positive")
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        if os.environ.get(variable) != "1":
            parser.error(f"set {variable}=1 before launching Python")
    args.output.mkdir(parents=True, exist_ok=True)
    cpuinfo = Path("/proc/cpuinfo").read_text()
    cpu = next(line.split(":", 1)[1].strip() for line in cpuinfo.splitlines()
               if line.startswith("model name"))
    config = Config()
    output = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"cpu": cpu, "platform": platform.platform(),
                        "python": platform.python_version(), "numpy": np.__version__,
                        "scipy": scipy.__version__, "blas_threads": 1,
                        "cpu_affinity": sorted(os.sched_getaffinity(0))},
        "protocol": {
            "config": config.as_dict(), "clock": "time.perf_counter_ns",
            "cache": "warm; points already in memory; no disk I/O in timing",
            "full": "Moments.from_points(points) followed by fit_patch(moments, config)",
            "moments": "Moments.from_points only: mean, monomials, 10x10 Gram matrix",
            "from_moments": "fit_patch only: classification, extent, inverse covariance, coefficients, MSE",
            "excluded": "point generation, disk I/O, segmentation, association, registration, map update",
            "synthetic": "2m sphere or tilted plane, 2mm standard-deviation normal noise; float64 XYZ",
            "warmup_calls": {"synthetic": 8, "real_frame": 2},
            "note": "Per-call latencies of the independent Python implementation, not author C++ timings. No nonlinear ML refinement.",
        },
        "source_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                          for name in ("core.py", "segmentation.py", "segment.cpp", "benchmark_fitting.py")},
        "synthetic": [], "kitti_frames": [],
    }
    synthetic_csv = []
    for method in ("schur", "taubin"):
        cfg = replace(config, fitting=method)
        for shape in ("sphere", "plane"):
            for count in args.sizes:
                points = synthetic_points(shape, count, config.seed+count)
                moments = Moments.from_points(points)
                patch = fit_patch(moments, cfg)
                expected = "quadric" if shape == "sphere" else "plane"
                if patch.kind != expected:
                    raise AssertionError(f"{method}/{shape}/{count}: expected {expected}, got {patch.kind}")
                residual, _ = distance_and_gradient(patch, points)
                rmse = float(np.sqrt(np.mean(residual)))
                if not np.isfinite(rmse) or rmse > .01:
                    raise AssertionError(f"invalid synthetic fit: RMSE {rmse}")
                repeats = args.repeats if count <= 10000 else max(20, args.repeats//10)
                result = {"method": method, "shape": shape, "points": count,
                          "kind": patch.kind, "taubin_distance_rmse_m": rmse}
                result["moments"] = measure(lambda: Moments.from_points(points), repeats)
                result["from_moments"] = measure(lambda: fit_patch(moments, cfg), repeats)
                result["full"] = measure(lambda: fit_patch(Moments.from_points(points), cfg), repeats)
                output["synthetic"].append(result)
                row = {key: result[key] for key in ("method", "shape", "points", "kind", "taubin_distance_rmse_m")}
                for stage in ("moments", "from_moments", "full"):
                    row.update({stage+"_"+key: result[stage][key] for key in ("median_us", "p95_us", "repeats")})
                synthetic_csv.append(row)
                print(f"{method:6} {shape:6} {count:6} points: full {result['full']['median_us']:.1f} us; "
                      f"from moments {result['from_moments']['median_us']:.1f} us", flush=True)
    frame_csv = []
    for frame in args.frames:
        path = ROOT/f"data/kitti/sequences/04/velodyne/{frame:06}.bin"
        points = np.fromfile(path, dtype="<f4").reshape(-1, 4)[:, :3]
        patches, point_lists, stats = segment_scan(points, config)
        if not patches:
            raise AssertionError(f"KITTI frame {frame} has no retained patches")
        moments_list = [p.moments for p in patches]
        full = measure(lambda: [fit_patch(Moments.from_points(p), config) for p in point_lists],
                       args.frame_repeats, warmup=2)
        from_moments = measure(lambda: [fit_patch(m, config) for m in moments_list],
                               args.frame_repeats, warmup=2)
        row = {"frame": frame, "input_points": len(points), **stats,
               "patch_points_median": float(np.median([len(p) for p in point_lists])),
               "patch_points_max": max(len(p) for p in point_lists),
               "full_median_ms": full["median_us"]/1000, "full_p95_ms": full["p95_us"]/1000,
               "from_moments_median_ms": from_moments["median_us"]/1000,
               "from_moments_p95_ms": from_moments["p95_us"]/1000}
        output["kitti_frames"].append({**row, "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                      "full": full, "from_moments": from_moments})
        frame_csv.append(row)
        print(f"KITTI04/{frame:06}: {len(patches)} patches, {stats['quadric_patches']} quadrics, "
              f"full {row['full_median_ms']:.2f} ms/frame", flush=True)
    output["kitti_summary"] = {
        "frames": len(frame_csv), "frame_repeats": args.frame_repeats,
        "full_frame_median_ms": summarize([r["full_median_ms"] for r in frame_csv]),
        "from_moments_frame_median_ms": summarize([r["from_moments_median_ms"] for r in frame_csv]),
        "patches_per_frame": summarize([r["patches"] for r in frame_csv]),
        "retained_points_per_frame": summarize([r["retained_points"] for r in frame_csv]),
        "all_full_calls_ms": summarize([v/1000 for f in output["kitti_frames"] for v in f["full"]["samples_us"]]),
        "total_patch_counts": {kind: sum(r[kind+"_patches"] for r in frame_csv)
                               for kind in ("plane", "quadric", "distribution")},
    }
    (args.output/"metrics.json").write_text(json.dumps(output, indent=2)+"\n")
    write_csv(args.output/"synthetic.csv", synthetic_csv)
    write_csv(args.output/"kitti_frames.csv", frame_csv)
    report = ["# 二次曲面拟合 CPU 实测", "", f"CPU：{cpu}；Python {platform.python_version()}，NumPy {np.__version__}，单线程 BLAS。",
              "", "点云已在内存，每项预热后重复计时；不含分割、关联、位姿优化和读盘。",
              "完整拟合包含统计量计算、类型判定、系数求解、MSE和协方差处理。预存统计量一列包含 fit_patch 的全部工作，不只是特征值分解。",
              "球面必须判为一般二次曲面，平面必须判为平面；两者均检查 Taubin 距离 RMSE < 1cm。",
              "", "|方法|场景|点数|完整拟合中位数(ms)|P95(ms)|预存统计量后(ms)|", "|---|---|---:|---:|---:|---:|"]
    for r in output["synthetic"]:
        report.append(f"|{r['method']}|{r['shape']}|{r['points']}|{r['full']['median_us']/1000:.4f}|"
                      f"{r['full']['p95_us']/1000:.4f}|{r['from_moments']['median_us']/1000:.4f}|")
    report += ["", "KITTI04 每30帧取1帧，共10帧（使用默认 Schur 方法；帧列表以 metrics.json 为准）。",
               "计时前完成分割，计时内对该帧保留的全部面片重新拟合，没有抽点或减少面片。",
               "", "|帧|保留点|面片|一般二次曲面|完整拟合中位数(ms)|P95(ms)|", "|---:|---:|---:|---:|---:|---:|"]
    for r in frame_csv:
        report.append(f"|{r['frame']}|{r['retained_points']}|{r['patches']}|{r['quadric_patches']}|"
                      f"{r['full_median_ms']:.3f}|{r['full_p95_ms']:.3f}|")
    report += ["", "拟合速度与整条里程计链路的精度分别评估；本次计时没有解决先前 KITTI 轨迹误差问题。",
               "原始逐次耗时、输入与源码 SHA256 见 metrics.json；汇总见 synthetic.csv / kitti_frames.csv。", ""]
    (args.output/"REPORT.md").write_text("\n".join(report))
    print(json.dumps(output["kitti_summary"], indent=2), flush=True)
    print(f"Saved: {args.output}", flush=True)


if __name__ == "__main__":
    main()
