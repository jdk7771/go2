"""64-beam range image + range continuity and declared normal-angle splitting."""

import ctypes
from pathlib import Path
import subprocess
import numpy as np

from .core import Moments, fit_patch

ROOT = Path(__file__).resolve().parent
_LIB = None


def _library():
    global _LIB
    if _LIB is None:
        source, output = ROOT / "segment.cpp", ROOT / "build/segment.so"
        output.parent.mkdir(exist_ok=True)
        if not output.exists() or output.stat().st_mtime < source.stat().st_mtime:
            subprocess.run(["g++", "-O3", "-std=c++17", "-shared", "-fPIC", str(source), "-o", str(output)], check=True)
        _LIB = ctypes.CDLL(str(output))
        _LIB.segment.argtypes = [ctypes.c_void_p]*3+[ctypes.c_int]*2+[ctypes.c_double]*2+[ctypes.c_void_p]
        _LIB.segment.restype = ctypes.c_int
    return _LIB


def segment_scan(points, config, width=2048):
    points = np.asarray(points, dtype=np.float32)
    distance = np.linalg.norm(points, axis=1)
    points = points[np.isfinite(points).all(axis=1) & (distance>2) & (distance<config.max_range)]
    distance = np.linalg.norm(points, axis=1)
    pitch = np.degrees(np.arcsin(np.clip(points[:, 2]/distance, -1, 1)))
    row = np.where(pitch>=-8.83, np.rint((2-pitch)*3), 32+np.rint((-8.83-pitch)*2)).astype(int)
    col = np.floor((np.arctan2(points[:, 1], points[:, 0])+np.pi)/(2*np.pi)*width).astype(int)%width
    good = (row>=0)&(row<64)
    points, row, col, distance = points[good], row[good], col[good], distance[good]
    ids = row*width+col
    order = np.argsort(distance, kind="stable")
    _, first = np.unique(ids[order], return_index=True)
    keep = order[first]
    xyz = np.zeros((64, width, 3), dtype=np.float32)
    valid = np.zeros((64, width), dtype=np.int32)
    xyz[row[keep], col[keep]], valid[row[keep], col[keep]] = points[keep], 1
    left, right = np.roll(xyz, 1, axis=1), np.roll(xyz, -1, axis=1)
    lv, rv = np.roll(valid, 1, axis=1), np.roll(valid, -1, axis=1)
    horizontal = np.where(lv[...,None], xyz-left, 0)+np.where(rv[...,None], right-xyz, 0)
    up, down = np.roll(xyz, 1, axis=0), np.roll(xyz, -1, axis=0)
    uv, dv = np.roll(valid, 1, axis=0), np.roll(valid, -1, axis=0)
    uv[0], dv[-1] = 0, 0
    vertical = np.where(uv[...,None], xyz-up, 0)+np.where(dv[...,None], down-xyz, 0)
    normals = np.cross(horizontal, vertical)
    normals /= np.maximum(np.linalg.norm(normals, axis=2, keepdims=True), 1e-9)
    normals = np.ascontiguousarray(normals, dtype=np.float32)
    labels = np.empty(valid.shape, dtype=np.int32)
    _library().segment(xyz.ctypes.data, normals.ctypes.data, valid.ctypes.data, 64, width,
                       np.radians(config.segmentation_angle_deg), np.cos(np.radians(config.normal_angle_deg)), labels.ctypes.data)
    flat_labels = labels.ravel()
    used = np.flatnonzero(flat_labels>=0)
    ordered = used[np.argsort(flat_labels[used], kind="stable")]
    _, starts, counts = np.unique(flat_labels[ordered], return_index=True, return_counts=True)
    point_lists, patches = [], []
    xyz = xyz.reshape(-1, 3)
    for start, count in zip(starts, counts):
        if count < config.min_patch:
            continue
        p = xyz[ordered[start:start+count]].astype(float)
        if np.linalg.norm(np.ptp(p, axis=0)) < .15:
            continue
        patch = fit_patch(Moments.from_points(p), config)
        patches.append(patch)
        point_lists.append(p)
    return patches, point_lists, {"projected_points": int(valid.sum()), "patches": len(patches),
                                   "retained_points": sum(len(p) for p in point_lists),
                                   "plane_patches": sum(p.kind=="plane" for p in patches),
                                   "quadric_patches": sum(p.kind=="quadric" for p in patches),
                                   "distribution_patches": sum(p.kind=="distribution" for p in patches)}
