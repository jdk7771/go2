"""Analytic differential geometry and an independent local polynomial baseline.

Normal n=grad(F)/|grad(F)|; shape operator S=-T.T Hess(F) T/|grad(F)|.
Signed principal curvatures follow this normal, so an outward sphere has -1/R.
Sources: Goldman (2005), DOI 10.1016/j.cagd.2005.06.005; CGAL Jet_fitting_3
manual for the PCA-frame/local-polynomial comparison (not CGAL's implementation).
"""

import numpy as np
from scipy.spatial import cKDTree

from .core import quadric_matrix


def tangent_basis(normals):
    reference = np.eye(3)[np.argmin(np.abs(normals), axis=1)]
    first = np.cross(normals, reference)
    first /= np.maximum(np.linalg.norm(first, axis=1, keepdims=True), 1e-30)
    return first, np.cross(normals, first)


def from_derivatives(gradient, hessian, points, valid=None):
    """Evaluate at regular surface points; hessian can be 3x3 or Nx3x3."""
    norm = np.linalg.norm(gradient, axis=1)
    good = np.isfinite(gradient).all(axis=1) & (norm > 1e-12)
    if valid is not None:
        good &= valid
    normal = gradient/np.maximum(norm[:, None], 1e-30)
    first, second = tangent_basis(normal)
    h = np.broadcast_to(hessian, (len(normal), 3, 3))
    a = -np.einsum("ni,nij,nj->n", first, h, first)/np.maximum(norm, 1e-30)
    b = -np.einsum("ni,nij,nj->n", first, h, second)/np.maximum(norm, 1e-30)
    d = -np.einsum("ni,nij,nj->n", second, h, second)/np.maximum(norm, 1e-30)
    mean = (a+d)/2
    spread = np.hypot(a-d, 2*b)/2
    principal = np.column_stack((mean-spread, mean+spread))
    good &= np.isfinite(principal).all(axis=1)
    slope = np.degrees(np.arccos(np.clip(np.abs(normal[:, 2]), 0, 1)))
    for array in (normal, principal, mean, slope):
        array[~good] = np.nan
    return {"normal": normal, "slope_deg": slope, "principal": principal,
            "mean": mean, "gaussian": principal[:, 0]*principal[:, 1],
            "valid": good, "points": points}


def differential_geometry(patch, points, project=True, projection_steps=4):
    """Optionally project to F=0 before evaluating its analytic derivatives.

    Fixed Newton normal steps are a zero-set projection, not a certified closest
    point solver. Unconverged and singular points return valid=False and NaNs.
    Coefficient normalization makes validity checks invariant to polynomial scale.
    """
    points = np.atleast_2d(np.asarray(points, dtype=float))
    if points.shape[1] != 3 or projection_steps < 0:
        raise ValueError("expected Nx3 points and nonnegative projection_steps")
    c = patch.coefficients.copy()
    magnitude = np.max(np.abs(c))
    c /= max(magnitude, 1e-300)
    a, b = quadric_matrix(c), c[6:9]
    local = points-patch.moments.origin
    valid = np.isfinite(local).all(axis=1) & (patch.kind != "distribution") & (magnitude > 0)
    if project:
        for _ in range(projection_steps):
            gradient = 2*local@a+b
            norm2 = np.sum(gradient*gradient, axis=1)
            f = np.einsum("ni,ij,nj->n", local, a, local)+local@b+c[9]
            good = valid & (norm2 > 1e-24)
            valid &= good
            step = np.divide(f, norm2, out=np.zeros_like(f), where=good)
            local = local-step[:, None]*gradient
    gradient = 2*local@a+b
    if project:
        f = np.einsum("ni,ij,nj->n", local, a, local)+local@b+c[9]
        distance = np.abs(f)/np.maximum(np.linalg.norm(gradient, axis=1), 1e-30)
        valid &= distance < 1e-6
    return from_derivatives(gradient, 2*a, local+patch.moments.origin, valid)


def local_geometry(points, queries, neighbors=64, mode="jet", tree=None, chunk=1024):
    """KNN + PCA, optionally followed by a degree-2 Monge-frame polynomial fit.

    Batched 6x6 normalized normal equations avoid a Python least-squares loop.
    This comparison is an independent NumPy implementation, not CGAL or PCL.
    PCA returns a dimensionless surface-variation score, NOT principal curvature.
    """
    points, queries = np.asarray(points, dtype=float), np.atleast_2d(queries).astype(float)
    if mode not in ("jet", "pca") or len(points) < 6:
        raise ValueError("mode must be jet/pca and at least 6 support points are required")
    tree = cKDTree(points) if tree is None else tree
    pieces = []
    for start in range(0, len(queries), chunk):
        query = queries[start:start+chunk]
        _, indices = tree.query(query, k=min(neighbors, len(points)), workers=1)
        nearby = points[indices]
        centered = nearby-nearby.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centered, centered)/indices.shape[1]
        values, vectors = np.linalg.eigh(covariance)
        normal, first, second = vectors[:, :, 0], vectors[:, :, 1], vectors[:, :, 2]
        variation = np.maximum(values[:, 0], 0)/np.maximum(values.sum(axis=1), 1e-30)
        if mode == "pca":
            result = {"normal": normal, "slope_deg": np.degrees(np.arccos(np.clip(np.abs(normal[:, 2]), 0, 1))),
                      "principal": np.full((len(query), 2), np.nan),
                      "valid": np.isfinite(normal).all(axis=1), "points": query,
                      "mean": np.full(len(query), np.nan), "gaussian": np.full(len(query), np.nan)}
        else:
            delta = nearby-query[:, None, :]
            u, v, w = [np.einsum("nki,ni->nk", delta, axis) for axis in (first, second, normal)]
            scale = np.sqrt(np.mean(u*u+v*v, axis=1))
            scale = np.maximum(scale, 1e-10)
            u, v = u/scale[:, None], v/scale[:, None]
            design = np.stack((np.ones_like(u), u, v, .5*u*u, u*v, .5*v*v), axis=2)
            gram = np.einsum("nki,nkj->nij", design, design)
            rhs = np.einsum("nki,nk->ni", design, w)
            regularizer = 1e-10*np.trace(gram, axis1=1, axis2=2)/6
            coeff = np.linalg.solve(gram+regularizer[:, None, None]*np.eye(6), rhs)
            du, dv = coeff[:, 1]/scale, coeff[:, 2]/scale
            duu, duv, dvv = [coeff[:, i]/scale**2 for i in (3, 4, 5)]
            gradient = normal-du[:, None]*first-dv[:, None]*second
            hessian = -(duu[:, None, None]*first[:, :, None]*first[:, None, :]
                        +duv[:, None, None]*(first[:, :, None]*second[:, None, :]
                                            +second[:, :, None]*first[:, None, :])
                        +dvv[:, None, None]*second[:, :, None]*second[:, None, :])
            result = from_derivatives(gradient, hessian, query+coeff[:, :1]*normal,
                                      np.isfinite(coeff).all(axis=1))
        result["surface_variation"] = variation
        pieces.append(result)
    return {key: np.concatenate([p[key] for p in pieces], axis=0) for key in pieces[0]}
