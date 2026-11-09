"""Export an offline browser viewer; optionally serve it on localhost.

JavaScript dependencies are embedded as data modules, and scene payloads use
local script tags. Opening index.html directly also works without a server.
"""

from __future__ import annotations

import argparse
import base64
import errno
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import webbrowser

import numpy as np

from .data import SavedRun
from ..settings import DEFAULT_RUN, MODES, LABELS

ASSETS = Path(__file__).with_name("assets")


def scene_label(name: str) -> str:
    if name == "open_entry_flat_floor_channel_30deg":
        return "空旷区进入 · 水平地面＋30° 倾斜顶板窄通道"
    if name == "flat_ceiling_bent_floor_30deg":
        return "水平顶板 · 下方水平接 30° 斜面"
    if name == "flat_floor_sloped_ceiling_30deg":
        return "水平地面 · 上方倾斜 30°"
    if name.startswith("tilted_corridor_"):
        return name.removeprefix("tilted_corridor_").removesuffix("deg") + "° 倾斜通道"
    if name == "asymmetric_overhang_slalom":
        return "非对称障碍 · 侧倾与转向"
    if name == "double_overhang_offset_pillar":
        return "双悬障与偏置立柱"
    if not name.startswith("beam_h"):
        return name
    return (
        name.replace("beam_h", "低梁 ")
        .replace("_yaw", " mm · yaw ")
        .replace("_tilt", "° · roll ")
        .replace("_pitch", "° · pitch ")
        + "°"
    )


def compact(array: np.ndarray) -> list:
    return np.round(array, 7).tolist()


def selection_label(metadata: dict) -> str:
    config = metadata.get("point_selection")
    if not config:
        return ""
    method = config["method"]
    if method == "all":
        return "完整点云对照"
    length = config["max_length"]
    if method == "radius":
        return f"仅距离截取 {length:g} m"
    return f"中心射线 {config['angle_deg']:g}° / {length:g} m"


def registered_runs(
    run_path: Path, include_runs: list[Path] | None = None
) -> list[SavedRun]:
    """Persist added data sources; importing only affects the existing viewer."""
    destination = run_path.resolve()
    registry = destination / "viewer_sources.json"
    paths = [destination]
    if registry.exists():
        paths.extend(
            (destination / relative).resolve()
            for relative in json.loads(registry.read_text())["runs"]
        )
    paths.extend(path.resolve() for path in (include_runs or []))
    saved = [SavedRun(path) for path in dict.fromkeys(paths)]
    aliases = [run.path.name for run in saved]
    if len(aliases) != len(set(aliases)):
        raise ValueError("Viewer source folders must have distinct names")
    if include_runs:
        registry.write_text(
            json.dumps(
                {"runs": [os.path.relpath(run.path, destination) for run in saved[1:]]},
                ensure_ascii=False,
                indent=2,
            )
            + "\n"
        )
    return saved


def mount_run(destination: Path, source: Path) -> str:
    """Link reports/videos into the same server root without copying results."""
    if source == destination:
        return "."
    link = destination / "viewer_runs" / source.name
    link.parent.mkdir(exist_ok=True)
    if link.exists() or link.is_symlink():
        if link.resolve() != source:
            raise ValueError(f"Viewer source mount already points elsewhere: {link}")
    else:
        link.symlink_to(os.path.relpath(source, link.parent), target_is_directory=True)
    return link.relative_to(destination).as_posix()


def export(run_path: Path, include_runs: list[Path] | None = None) -> Path:
    runs = registered_runs(run_path, include_runs)
    destination = runs[0].path
    data_dir = destination / "viewer_data"
    data_dir.mkdir(exist_ok=True)
    manifest = {"run": destination.name, "modes": LABELS, "scenes": [], "videos": []}
    preferred = [
        "open_entry_flat_floor_channel_30deg",
        "flat_ceiling_bent_floor_30deg",
        "flat_floor_sloped_ceiling_30deg",
        "tilted_corridor_30deg",
        "asymmetric_overhang_slalom",
        "beam_h430_yaw00_tilt+0",
    ]
    entries, used_ids = [], set()
    for saved in runs:
        directory = mount_run(destination, saved.path)
        scene_ids = {}
        for name in saved.scenes:
            scene_id = name if name not in used_ids else f"{saved.path.name}__{name}"
            if scene_id in used_ids:
                raise ValueError(f"Duplicate viewer scene ID: {scene_id}")
            used_ids.add(scene_id)
            scene_ids[name] = scene_id
            entries.append((saved, name, scene_id, directory))
        video_manifest = saved.path / "videos/manifest.json"
        if video_manifest.exists():
            for name, item in json.loads(video_manifest.read_text()).items():
                if name in scene_ids:
                    manifest["videos"].append(
                        {
                            "scene": scene_ids[name],
                            "label": scene_label(name)
                            + " · "
                            + (selection_label(saved.metadata) or "原始结果"),
                            "file": f"{directory}/videos/{item['file']}",
                        }
                    )
    entries.sort(
        key=lambda item: (
            preferred.index(item[1]) if item[1] in preferred else len(preferred)
        )
    )
    for saved, name, scene_id, directory in entries:
        raw = saved.scene(name)
        method_label = selection_label(saved.metadata)
        label = scene_label(name) + (f" · {method_label}" if method_label else "")
        modes = saved.metadata.get("modes", MODES)
        payload = {
            "name": scene_id,
            "sourceScene": name,
            "sourceRun": saved.path.name,
            "label": label,
            "cloud": compact(raw["planning_point_cloud"]),
            "initial": compact(raw["initial_trajectory"]),
            "half": compact(raw["body_half_extents"]),
            "description": raw["params"].get("description", ""),
            "crossSection": raw["params"].get("cross_section", []),
            "defaultView": raw["params"].get("default_view", "iso"),
            "defaultZoom": raw["params"].get(
                "default_zoom", 1.8 if raw["params"].get("cross_section") else 1
            ),
            "report": f"{directory}/report/REPORT.md",
            "trials": {},
        }
        for mode in modes:
            trial = saved.trial(name, mode)
            payload["trials"][mode] = {
                "final": compact(trial["final"]),
                "dense": compact(trial["dense"]),
                "clearance": compact(trial["clearance"]),
                "metrics": trial["metrics"],
                "history": compact(trial["history"]) if "history" in trial else [],
                "costs": compact(trial["costs"]) if "costs" in trial else [],
            }
        serialized = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        )
        (data_dir / f"{scene_id}.js").write_text(
            f"window.PLANNER_SCENES ??= {{}}; window.PLANNER_SCENES[{json.dumps(scene_id)}] = {serialized};\n",
            encoding="utf-8",
        )
        manifest["scenes"].append(
            {
                "id": scene_id,
                "sourceScene": name,
                "run": saved.path.name,
                "directory": directory,
                "report": payload["report"],
                "label": label,
                "sceneLabel": scene_label(name),
                "selectionLabel": method_label or "原始结果",
                "modes": list(modes),
                "file": f"viewer_data/{scene_id}.js",
            }
        )
    imports = {}
    for module, filename in (
        ("three", "three.module.min.js"),
        ("three/addons/controls/OrbitControls.js", "OrbitControls.js"),
    ):
        content = (ASSETS / "vendor" / filename).read_bytes()
        imports[module] = (
            "data:text/javascript;base64," + base64.b64encode(content).decode()
        )
    html = (ASSETS / "index.html").read_text()
    replacements = {
        "__STYLE__": (ASSETS / "viewer.css").read_text(),
        "__IMPORT_MAP__": json.dumps({"imports": imports}),
        "__MANIFEST__": json.dumps(manifest, ensure_ascii=False),
        "__VIEWER_JS__": (ASSETS / "viewer.js").read_text(),
    }
    for marker, content in replacements.items():
        html = html.replace(marker, content)
    entry = destination / "index.html"
    entry.write_text(html, encoding="utf-8")
    print(
        f"Viewer: {entry} ({len(entries)} entries, {sum(len(e['modes']) for e in manifest['scenes'])} trials, {len(runs)} source runs)",
        flush=True,
    )
    return entry


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="交互式点云轨迹查看器，可直接打开导出的 index.html"
    )
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument(
        "--include-run",
        type=Path,
        nargs="+",
        help="把已有实验加入当前查看器；后续导出自动保留",
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-open", action="store_true", help="不自动打开浏览器")
    parser.add_argument("--export-only", action="store_true", help="只导出离线 HTML")
    args = parser.parse_args(argv)
    entry = export(args.run, args.include_run)
    if args.export_only:
        return
    handler = functools.partial(SimpleHTTPRequestHandler, directory=str(entry.parent))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        print(f"Port {args.port} is in use; selected an available port.", flush=True)
    with server:
        address = f"http://127.0.0.1:{server.server_port}/"
        print(f"Open {address}  (Ctrl+C to stop)", flush=True)
        if not args.no_open:
            webbrowser.open(address)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
