import argparse
from dataclasses import replace
from pathlib import Path
import hashlib
import json

import numpy as np

from .core import Config, TraversabilityMapper
from .experiment import ROOT, benchmark, environment, sample_scan
from .io import read_cloud, save_result


def main():
    parser = argparse.ArgumentParser(description="FSGP-BGK paper-equation CPU reproduction")
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("verify-sources", help="verify the pinned official source and data snapshots")
    check.set_defaults(command="verify-sources")
    bench = sub.add_parser("benchmark", help="run diagnostic comparisons and inducing-point ablations")
    bench.add_argument("--output", type=Path, default=ROOT / "outputs" / "cpu")
    bench.add_argument("--scenes", nargs="+", choices=["flat", "ramp", "steps", "hill", "quick"],
                       default=["flat", "ramp", "steps", "hill", "quick"])
    bench.add_argument("--inducing", type=int, nargs="+", default=[50, 125, 500])
    bench.add_argument("--frames", type=int, default=8)
    bench.add_argument("--config", type=Path)
    mapping = sub.add_parser("map", help="assess a gravity-aligned cloud (PCD/NPY/NPZ/XYZ/CSV)")
    mapping.add_argument("--input", type=Path, required=True)
    mapping.add_argument("--output", type=Path, default=ROOT / "outputs" / "map")
    mapping.add_argument("--center", type=float, nargs=2, default=[0, 0])
    mapping.add_argument("--inducing", type=int, default=125)
    mapping.add_argument("--max-scan-points", type=int, default=4000)
    mapping.add_argument("--config", type=Path)
    args = parser.parse_args()
    if args.command == "verify-sources":
        lock = json.loads((ROOT / "sources.lock.json").read_text())
        bad = []
        for entry in lock["files"]:
            path = ROOT / entry["local_path"]
            if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
                bad.append(entry["local_path"])
        if bad:
            raise SystemExit("source mismatch: " + ", ".join(bad))
        print(f"Verified {len(lock['files'])} upstream files at {lock['commit']}")
        return
    config = Config(**json.loads(args.config.read_text())) if args.config else Config()
    if args.command == "benchmark":
        benchmark(args.output, args.scenes, args.inducing, args.frames, config)
        print(f"Report: {args.output / 'REPORT.md'}")
        return
    if args.max_scan_points < 3:
        parser.error("--max-scan-points must be >= 3")
    config = replace(config, inducing_points=args.inducing)
    points = read_cloud(args.input)
    scan = sample_scan(points, args.center, config, np.random.default_rng(config.seed),
                       max_points=args.max_scan_points, occlusion=0, noise=0)
    mapper = TraversabilityMapper(config)
    result = mapper.fuse(mapper.assess(scan, args.center, timestamp=0))
    args.output.mkdir(parents=True, exist_ok=True)
    save_result(args.output / "map.npz", result)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), constrained_layout=True)
    xy = result["xy"]
    half = config.resolution / 2
    extent = [xy[:, 0].min() - half, xy[:, 0].max() + half,
              xy[:, 1].min() - half, xy[:, 1].max() + half]
    for ax, name in zip(axes, ["height", "variance", "cost"]):
        values = result[name].reshape(result["shape"])
        im = ax.imshow(values, origin="lower", extent=extent, cmap="viridis" if name != "cost" else "magma")
        ax.set(title=name, xlabel="world x (m)", ylabel="world y (m)")
        fig.colorbar(im, ax=ax, shrink=0.7)
    fig.savefig(args.output / "map.png", dpi=160)
    plt.close(fig)
    metadata = {"config": config.as_dict(), "input": str(args.input.resolve()),
                "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
                "center": args.center, "raw_points": len(points), "scan_points": len(scan),
                "known_coverage": float(result["known"].mean()), "timing_ms": result["timing_ms"],
                "note": "one frame only: BGK has no temporal evidence yet", "environment": environment()}
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))
    print(f"Map: {args.output / 'map.png'}")


if __name__ == "__main__":
    main()
