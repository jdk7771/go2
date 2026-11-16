"""Export a compact, source-backed inline visualization without rerunning GP."""

import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path
import shutil
import zlib

import numpy as np

from ..experiment import ROOT, make_scene
from ..table_scene import export as export_table_scene


SCENE_LABELS = {"flat": "平地", "ramp": "斜坡", "steps": "台阶与凸起", "hill": "丘陵 hill", "quick": "官方 quick"}
METHODS = {"sgp": "SGP-XY", "fsgp": "FSGP", "bgk": "FSGP-BGK"}


def sample(points, limit, seed):
    if len(points) > limit:
        points = points[np.random.default_rng(seed).choice(len(points), limit, replace=False)]
    return points[:, :3].tolist()


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def safe_matrix(array):
    return [[float(value) if np.isfinite(value) else None for value in row] for row in array]


def heatmap(key, title, xy, shape, values, domain):
    x = xy.reshape(*shape, 2)[0, :, 0].tolist()
    y = xy.reshape(*shape, 2)[:, 0, 1].tolist()
    # Spatial grids need equal metric scale and positive world Y pointing up.
    # These are drawn as spatial SVGs, rather than categorical heatmaps.
    return {"id": f"fsgp-map-{key}", "title": title, "x": x, "y": y,
            "values": safe_matrix(np.asarray(values).reshape(shape)), "domain": domain}


def table_cloud(destination):
    """Keep every component visible without inventing a benchmark result."""
    folder = destination.parent / "large-table"
    if not (folder / "large_table.npz").exists():
        export_table_scene(folder)
    metadata = json.loads((folder / "scene.json").read_text())
    with np.load(folder / "large_table.npz", allow_pickle=False) as data:
        points, labels = data["points"], data["labels"]
    # Stratify by component so the large floor cannot crowd out slender legs.
    displayed = []
    for label, limit in enumerate((2200, 2200, 400, 400, 400, 400)):
        displayed.extend(sample(points[labels == label], limit, 100 + label))
    return {"label": "大桌子（桌下空旷）", "inspection_only": True,
            "source": "本地生成的完整多层表面点云", "terrain": displayed,
            "point_count": len(points), "dimensions": metadata["dimensions"],
            "pcd_sha256": metadata["pcd_sha256"], "configs": {}}


def export(run, destination, table_run=None):
    run = run.resolve()
    protocol = json.loads((run / "protocol.json").read_text())
    per_frame, summary = read_csv(run / "per_frame.csv"), read_csv(run / "summary.csv")
    bank = {"scenes": {}, "counts": protocol["inducing_counts"], "frames": protocol["frames"],
            "resolution": protocol["config"]["resolution"], "length": protocol["config"]["length"]}
    table_info = table_cloud(destination)
    sources = {scene: run for scene in protocol["scenes"]}
    table_run = Path(table_run) if table_run else ROOT / "outputs/table_cpu"
    if (table_run / "protocol.json").exists():
        table_protocol = json.loads((table_run / "protocol.json").read_text())
        for key in ("inducing_counts", "frames"):
            if table_protocol[key] != protocol[key]:
                raise ValueError(f"table and existing benchmark disagree on {key}")
        for key in ("resolution", "length"):
            if table_protocol["config"][key] != protocol["config"][key]:
                raise ValueError(f"table and existing benchmark disagree on {key}")
        if table_protocol["source_pcd_sha256"] != table_info["pcd_sha256"]:
            raise ValueError("table results were computed from a different point cloud")
        sources["table"] = table_run
        table_images = destination.parent / "large-table-test"
        table_images.mkdir(exist_ok=True)
        for name in ("controls.png", "table_m50.png", "table_m125.png", "table_m500.png"):
            shutil.copyfile(table_run / name, table_images / name)
    for scene, scene_run in sources.items():
        frame_records = per_frame if scene_run == run else read_csv(scene_run / "per_frame.csv")
        summary_records = summary if scene_run == run else read_csv(scene_run / "summary.csv")
        configurations, maximum = {}, 0.0
        for count in protocol["inducing_counts"]:
            prefix = scene_run / f"{scene}_m{count}"
            with np.load(str(prefix) + "_evidence.npz", allow_pickle=False) as data:
                scan, reference = data["points"], data["reference_cost"]
            maps = {}
            for suffix in METHODS:
                with np.load(str(prefix) + f"_{suffix}.npz", allow_pickle=False) as data:
                    maps[suffix] = {name: data[name] for name in ("xy", "shape", "cost", "inducing", "known", "observed")}
            values = np.concatenate([reference.ravel()] + [maps[key]["cost"].ravel() for key in maps])
            maximum = max(maximum, float(np.nanmax(values)))
            configurations[str(count)] = {"maps": maps, "reference": reference}
        scene_data = {"label": table_info["label"] if scene == "table" else SCENE_LABELS.get(scene, scene), "configs": {},
                      "source": "官方示例点云" if scene in ("hill", "quick") else "本地生成的诊断地形",
                      "scan": sample(scan, 900, 7), "scan_count": len(scan),
                      "terrain": table_info["terrain"] if scene == "table" else sample(make_scene(scene), 1200, 8)}
        if scene == "table":
            scene_data.update(point_count=table_info["point_count"], dimensions=table_info["dimensions"],
                              reference_label="平坦地面解析参考（桌腿格排除）",
                              reference_note="共享色标 · 越高越难通行 · 平坦地面参考为 0，桌腿相交格与地面外格排除；这是地面代价偏差，不是净空或碰撞分类。")
        domain = [0, max(maximum, 0.01)]
        for count in protocol["inducing_counts"]:
            item = configurations[str(count)]
            xy, shape = item["maps"]["fsgp"]["xy"], tuple(item["maps"]["fsgp"]["shape"])
            charts = {"reference": heatmap("reference", scene_data.get("reference_label", "完整点云几何参考"), xy, shape, item["reference"], domain)}
            stats = {}
            for key, label in METHODS.items():
                record = next(row for row in summary_records if row["scene"] == scene and int(row["inducing_points"]) == count and row["method"] == label)
                stats[key] = {"mae": float(record["cost_mae"]), "ms": float(record["median_runtime_ms"]),
                              "coverage": float(record["mean_known_coverage"])}
                if scene == "table":
                    stats[key].update({name: float(record[name]) for name in
                                       ("under_height_median_m", "under_slope_mean", "under_cost_mean")})
                charts[key] = heatmap(key, label, xy, shape, item["maps"][key]["cost"], domain)
            matching = [row for row in frame_records if row["scene"] == scene and int(row["inducing_points"]) == count]
            charts["error"] = {"id": "fsgp-frame-error", "title": "8 帧平坦地面代价 MAE · 参考为 0" if scene == "table" else "8 帧代价 MAE · 同一当前支持区域", "height": 280,
                               "spec": {"type": "line", "x": "frame", "y": "mae", "series": "method",
                                        "xLabel": "帧", "yLabel": "代价 MAE（越低越好）", "startAtZero": True,
                                        "showLegend": True, "valueFormat": {"style": "decimal", "maximumFractionDigits": 5},
                                        "colors": {"SGP-XY": "var(--chart-1)", "FSGP": "var(--chart-2)", "FSGP-BGK": "var(--chart-3)"}},
                               "columns": ["frame", "mae", "method"],
                               "rows": [[int(row["frame"]) + 1, float(row["cost_mae"]) if row["cost_mae"] else None, row["method"]]
                                       for row in sorted(matching, key=lambda row: int(row["frame"]))]}
            scene_data["configs"][str(count)] = {"charts": charts, "stats": stats,
                                                "inducing": item["maps"]["fsgp"]["inducing"][:, :3].tolist(),
                                                "window": [float(xy[:, 0].min()) - bank["resolution"] / 2,
                                                           float(xy[:, 0].max()) + bank["resolution"] / 2,
                                                           float(xy[:, 1].min()) - bank["resolution"] / 2,
                                                           float(xy[:, 1].max()) + bank["resolution"] / 2]}
        bank["scenes"][scene] = scene_data
    if "table" not in bank["scenes"]:
        bank["scenes"]["table"] = table_info
    serialized = json.dumps(bank, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    payload = base64.b64encode(zlib.compress(serialized.encode(), level=9)).decode()
    initial = bank["scenes"]["hill"]["configs"]["125"]["charts"]
    initial_scripts = "\n".join(
        '<script type="application/json" data-viz-chart data-chart-key="' + key + '">' +
        json.dumps(config, ensure_ascii=False, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c") + '</script>'
        for key, config in initial.items() if key == "error")
    template = (Path(__file__).parent / "templates" / "fsgp-results.html").read_text()
    fragment = template.replace("<!--FSGP_DATA-->", payload).replace("<!--FSGP_INITIAL_CHARTS-->", initial_scripts)
    if len(fragment.encode()) >= 1_000_000:
        raise ValueError(f"inline visualization exceeds 1 MB: {len(fragment.encode())} bytes; compressed bank {len(payload)} bytes")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(fragment)
    checks = destination.parent / "checks"
    checks.mkdir(exist_ok=True)
    for scene, count, chart in (("hill", 125, "error"), ("steps", 50, "error"),
                                ("flat", 500, "error"), ("quick", 500, "error"), ("ramp", 125, "error")):
        state = bank["scenes"][scene]["configs"][str(count)]["charts"][chart]
        (checks / f"{scene}-{count}-{chart}.json").write_text(json.dumps(state, ensure_ascii=False, allow_nan=False))
    manifest = {"input_run": str(run), "protocol_sha256": hashlib.sha256((run / "protocol.json").read_bytes()).hexdigest(),
                "scenes": list(bank["scenes"]), "experiment_scenes": list(sources),
                "inspection_scenes": [s for s, value in bank["scenes"].items() if value.get("inspection_only")],
                "input_table_run": str(table_run.resolve()) if "table" in sources else None,
                "table_protocol_sha256": hashlib.sha256((table_run / "protocol.json").read_bytes()).hexdigest() if "table" in sources else None,
                "counts": bank["counts"], "map_frame": protocol["frames"],
                "cost_grid": "full 25x25 saved values, no rounding or imputation",
                "point_display": "deterministic subsample, max 900 scan points / 1200 terrain points; table: 6000 points stratified by component",
                "shared_color_scale": "shared across all methods and M within each scene",
                "fragment_bytes": len(fragment.encode()), "raw_json_bytes": len(serialized.encode())}
    (destination.parent / "visualization-manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=ROOT / "outputs" / "cpu")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "visualization" / "fsgp-results.html")
    parser.add_argument("--table-run", type=Path, default=ROOT / "outputs/table_cpu")
    arguments = parser.parse_args()
    export(arguments.run, arguments.output, arguments.table_run)
