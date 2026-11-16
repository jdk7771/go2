"""Raw unorganized cloud -> geometry segmentation -> quadric -> map attributes.

This is a local extension for FSGP-BGK's official terrain examples. Its spatial
components are not the quadric paper's 64-beam segmentation or the FSGP method.
Neither labels, file normals nor clean reference points enter this pipeline.
"""

import ctypes
from dataclasses import dataclass, asdict
from pathlib import Path
import subprocess
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree

from .core import Config, Moments, fit_patch, quadric_matrix
from .curvature import differential_geometry, from_derivatives

ROOT = Path(__file__).resolve().parent
_LIB = None


@dataclass(frozen=True)
class SegmentationConfig:
    neighbors: int = 20
    cell_size: float = 1.5
    max_link: float = .45
    normal_angle_deg: float = 30.
    plane_distance: float = .08
    min_patch: int = 20
    support_radius: float = .45

    def as_dict(self):
        return asdict(self)


def library():
    global _LIB
    if _LIB is None:
        source, output = ROOT/"terrain_segment.cpp", ROOT/"build/terrain_segment.so"
        output.parent.mkdir(exist_ok=True)
        if not output.exists() or output.stat().st_mtime < source.stat().st_mtime:
            subprocess.run(["g++", "-O3", "-std=c++17", "-shared", "-fPIC", str(source), "-o", str(output)], check=True)
        _LIB = ctypes.CDLL(str(output))
        _LIB.terrain_components.argtypes = [ctypes.c_void_p]*4+[ctypes.c_int]*2+[ctypes.c_double]*3+[ctypes.c_void_p]
        _LIB.terrain_components.restype = ctypes.c_int
    return _LIB


def estimate_normals(points, indices):
    local = points[indices]
    delta = local-local.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", delta, delta)/indices.shape[1]
    values, vectors = np.linalg.eigh(covariance)
    return vectors[:, :, 0], values


def segment_unorganized(points, config):
    start = perf_counter()
    points = np.ascontiguousarray(points, dtype=float)
    tree = cKDTree(points)
    _, neighbors = tree.query(points, k=min(config.neighbors, len(points)), workers=1)
    normals, _ = estimate_normals(points, neighbors)
    normal_ms = (perf_counter()-start)*1000
    start = perf_counter()
    _, cells = np.unique(np.floor(points[:,:2]/config.cell_size).astype(np.int64), axis=0, return_inverse=True)
    cells = np.ascontiguousarray(cells, dtype=np.int64)
    neighbors = np.ascontiguousarray(neighbors, dtype=np.int64)
    normals = np.ascontiguousarray(normals)
    labels = np.full(len(points), -1, dtype=np.int32)
    library().terrain_components(points.ctypes.data, normals.ctypes.data, neighbors.ctypes.data,
        cells.ctypes.data, len(points), neighbors.shape[1], config.max_link,
        np.cos(np.radians(config.normal_angle_deg)), config.plane_distance, labels.ctypes.data)
    unique, counts = np.unique(labels, return_counts=True)
    accepted = unique[counts>=config.min_patch]
    labels[~np.isin(labels, accepted)] = -1
    indices = [np.flatnonzero(labels==label) for label in accepted]
    return labels, indices, {"normal_knn_ms": normal_ms, "region_growing_ms": (perf_counter()-start)*1000}


def vertical_geometry(patch, xy, anchor_z):
    """Select the vertical root closest to the observed anchor, with fixed XY.

    Multiple height layers remain distinct patches. Negative discriminants and
    vertical planes without an isolated height root stay unknown, never zeroed.
    """
    c = patch.coefficients.copy()
    c /= max(float(np.max(np.abs(c))), 1e-300)
    a, b = quadric_matrix(c), c[6:9]
    local_xy = xy-patch.moments.origin[:2]
    x, y = local_xy.T
    target = anchor_z-patch.moments.origin[2]
    quadratic = a[2,2]
    linear = 2*a[0,2]*x+2*a[1,2]*y+b[2]
    constant = a[0,0]*x*x+2*a[0,1]*x*y+a[1,1]*y*y+b[0]*x+b[1]*y+c[9]
    roots = np.full(len(x), np.nan)
    if abs(quadratic) < 1e-12:
        roots = np.divide(-constant, linear, out=roots, where=np.abs(linear)>1e-12)
    else:
        discriminant = linear*linear-4*quadratic*constant
        sqrt_d = np.sqrt(np.maximum(discriminant, 0))
        q = -.5*(linear+np.where(linear>=0, 1., -1.)*sqrt_d)
        first = q/quadratic
        second = np.divide(constant, q, out=np.full_like(q, np.nan), where=np.abs(q)>1e-20)
        roots = np.where(np.abs(first-target)<=np.abs(second-target), first, second)
        repeated = (sqrt_d<1e-14) & (np.abs(q)<1e-20)
        roots[repeated] = -linear[repeated]/(2*quadratic)
        roots[discriminant<0] = np.nan
    valid = np.isfinite(roots) & (np.abs(roots-target)<=.5) & (patch.kind!="distribution")
    points = np.column_stack((xy, np.where(valid, roots, target)+patch.moments.origin[2]))
    result = differential_geometry(patch, points, project=False)
    result["valid"] &= valid
    result["height"] = np.where(result["valid"], points[:, 2], np.nan)
    for name in ("normal", "principal", "slope_deg", "mean", "gaussian"):
        result[name][~result["valid"]] = np.nan
    return result


def analytic_map(points, xy, fit_config=None, segmentation=None, center=(0.,0.), length=5.):
    start = perf_counter()
    fit_config = fit_config or Config(fitting="taubin")
    segmentation = segmentation or SegmentationConfig()
    points = np.asarray(points, dtype=float)
    keep = np.isfinite(points).all(axis=1) & (np.max(np.abs(points[:,:2]-center), axis=1)<=length/2+segmentation.support_radius)
    points = np.ascontiguousarray(points[keep])
    if len(points)<segmentation.min_patch:
        raise ValueError("too few local points")
    preparation_ms = (perf_counter()-start)*1000
    labels, lists, timing = segment_unorganized(points, segmentation)
    fitting_start = perf_counter()
    patches = [fit_patch(Moments.from_points(points[ids]), fit_config) for ids in lists]
    fitting_ms = (perf_counter()-fitting_start)*1000
    evaluation_start = perf_counter()
    distance, nearest = cKDTree(points[:,:2]).query(xy, workers=1)
    query_labels = labels[nearest]
    output = {"height": np.full(len(xy), np.nan), "normal": np.full((len(xy),3), np.nan),
              "principal": np.full((len(xy),2), np.nan), "slope_deg": np.full(len(xy), np.nan),
              "mean": np.full(len(xy), np.nan), "gaussian": np.full(len(xy), np.nan),
              "valid": np.zeros(len(xy), dtype=bool)}
    for ids, patch in zip(lists, patches):
        selected = np.flatnonzero((query_labels==labels[ids[0]]) & (distance<=segmentation.support_radius))
        if not len(selected):
            continue
        values = vertical_geometry(patch, xy[selected], points[nearest[selected],2])
        for key in output:
            output[key][selected] = values[key]
    timing = {"preparation_ms": preparation_ms, **timing, "fitting_ms": fitting_ms,
              "evaluation_ms": (perf_counter()-evaluation_start)*1000,
              "total_ms": (perf_counter()-start)*1000}
    return {**output, "xy": xy, "points": points, "labels": labels, "patches": patches,
            "patch_indices": lists, "timing_ms": timing,
            "segmented_point_fraction": float(np.mean(labels>=0)),
            "patch_counts": {kind: sum(p.kind==kind for p in patches) for kind in ["plane","quadric","distribution"]}}


def local_height_map(points, xy, neighbors=64, support_radius=.45):
    """Independent height-graph local quadratic baseline on the same grid."""
    points = np.asarray(points, dtype=float)
    distance, nearest = cKDTree(points[:,:2]).query(xy, workers=1)
    query = np.column_stack((xy, points[nearest,2]))
    _, ids = cKDTree(points).query(query, k=min(neighbors,len(points)), workers=1)
    nearby = points[ids]
    u,v = (nearby[:,:,:2]-xy[:,None,:]).transpose(2,0,1)
    scale = np.maximum(np.sqrt(np.mean(u*u+v*v, axis=1)),1e-10)
    u,v = u/scale[:,None],v/scale[:,None]
    design = np.stack((np.ones_like(u),u,v,.5*u*u,u*v,.5*v*v),axis=2)
    gram = np.einsum("nki,nkj->nij",design,design)
    z = nearby[:,:,2]-points[nearest,2,None]
    rhs = np.einsum("nki,nk->ni",design,z)
    ridge = 1e-10*np.trace(gram,axis1=1,axis2=2)/6
    coefficient = np.linalg.solve(gram+ridge[:,None,None]*np.eye(6),rhs)
    fx,fy = coefficient[:,1]/scale,coefficient[:,2]/scale
    gradient = np.column_stack((-fx,-fy,np.ones(len(xy))))
    hessian = np.zeros((len(xy),3,3))
    hessian[:,0,0],hessian[:,0,1],hessian[:,1,0],hessian[:,1,1] = (
        -coefficient[:,3]/scale**2,-coefficient[:,4]/scale**2,
        -coefficient[:,4]/scale**2,-coefficient[:,5]/scale**2)
    height = points[nearest,2]+coefficient[:,0]
    result = from_derivatives(gradient,hessian,np.column_stack((xy,height)),distance<=support_radius)
    return {**result,"height":np.where(result["valid"],height,np.nan),"xy":xy}
