"""Explain saved Q20 fitting/display gaps without changing fitted results.

Rechecks the local fit rejection criteria and counts the owners of actual
triangles, rather than assuming every saved patch vertex belongs to a face.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

from .terrain_assessment import AssessmentConfig, evaluate, fit_pools, window_pools

ROOT = Path(__file__).resolve().parent
WEB = ROOT / 'outputs/visualization'


def audit(name):
    folder = WEB / 'assessment' / name
    meta = json.loads((folder / 'data.json').read_text())
    models = np.fromfile(folder / 'models.bin', dtype='<f8').reshape(meta['centers'], -1)
    col = {key: i for i, key in enumerate(meta['fields'])}
    points = np.fromfile(folder / 'points.bin', dtype='<f4').reshape(-1, 3).astype(float)
    seeds = models[:, :3]
    cfg = AssessmentConfig(**meta['config'])
    pools = window_pools(points, seeds, cfg)
    fits = fit_pools(points, seeds, pools, cfg)
    valid = np.isfinite(fits['height'])
    np.testing.assert_array_equal(valid, models[:, col['valid']].astype(bool))
    np.testing.assert_array_equal(fits['counts'], models[:, col['point_count']])
    np.testing.assert_allclose(fits['coefficient'], models[:, 3:9], rtol=1e-10, atol=1e-10, equal_nan=True)
    active = fits['counts'] >= cfg.minimum_points
    rank = fits['full_rank']
    offset = np.abs(fits['coefficient'][:, 0]) > cfg.height_gate_m
    rmse = fits['rmse'] > cfg.max_rmse_m
    # Sequential reasons are disjoint; the raw rejection checks may overlap.
    rejection = dict(
        insufficient_points=int((~active).sum()),
        rank=int((active & ~rank).sum()),
        center_offset_after_rank=int((active & rank & offset).sum()),
        rmse_after_rank_offset=int((active & rank & ~offset & rmse).sum()),
    )
    assert sum(rejection.values()) + int(valid.sum()) == len(seeds)
    # This is a diagnostic ablation of the gate, NOT an alternative valid fit:
    # XY-only counts can include several surfaces, walls, and different layers.
    xy_counts = cKDTree(points[:, :2]).query_ball_point(
        seeds[:, :2], cfg.patch_size_m / 2 * (1 + 1e-7), p=np.inf, return_length=True)
    insufficient = dict(
        xy_window_already_below_minimum=int((~active & (xy_counts < cfg.minimum_points)).sum()),
        xy_enough_but_height_gate_below_minimum=int((~active & (xy_counts >= cfg.minimum_points)).sum()),
    )
    assert sum(insufficient.values()) == rejection['insufficient_points']
    faces = np.fromfile(folder / 'patch-faces.bin', dtype='<u4').reshape(-1, 3)
    owners = np.fromfile(folder / 'patch-owners.bin', dtype='<u4')
    used = np.unique(owners[faces])
    no_face_ids = np.setdiff1d(np.flatnonzero(valid), used)
    axis = np.linspace(-cfg.patch_size_m/2, cfg.patch_size_m/2, 5)
    x, y = np.meshgrid(axis, axis)
    uv = np.column_stack((x.ravel(), y.ravel()))
    cell = (np.arange(4)[:, None]*5 + np.arange(4)).ravel()
    candidates = np.r_[np.column_stack((cell, cell+1, cell+5)),
                       np.column_stack((cell+1, cell+6, cell+5))]
    no_face_reasons = dict(xy_hull_alone_removes_all=0, height_gate_after_xy_removes_all=0)
    for i in no_face_ids:
        hull = ConvexHull(points[pools[i], :2]-seeds[i, :2])
        inside = np.all(uv @ hull.equations[:, :2].T+hull.equations[:, 2] <= 1e-7, axis=1)
        z = evaluate(seeds, fits['coefficient'], np.full(25, i), uv+seeds[i, :2])
        within_z = np.abs(z-seeds[i, 2]) <= cfg.height_gate_m+1e-7
        assert not np.any(np.all((inside & within_z)[candidates], axis=1))
        if not np.any(np.all(inside[candidates], axis=1)):
            no_face_reasons['xy_hull_alone_removes_all'] += 1
        else:
            no_face_reasons['height_gate_after_xy_removes_all'] += 1
    assembled = models[:, col['assembled']].astype(bool)
    mesh = np.fromfile(folder / 'mesh.bin', dtype='<u4')
    np.testing.assert_array_equal(np.flatnonzero(assembled), np.unique(mesh))
    assert np.all(valid[used]) and np.all(valid[assembled])
    metrics = {}
    for field, scale in [('jump_b_m', .12), ('max_error_m', .02), ('slope_deg', 45.), ('body_cost', 1.)]:
        v = models[valid, col[field]]
        metrics[field] = dict(
            valid_functions_with_known_value=int(np.isfinite(v).sum()),
            near_white_below_one_percent_color_scale=int((np.isfinite(v) & (v < .01 * scale)).sum()),
            color_scale_maximum=scale,
        )
    return dict(
        scene=name, raw_points=meta['raw_points'], centers=len(seeds),
        solved=int(active.sum()), valid=int(valid.sum()), valid_center_fraction=float(valid.mean()),
        rejection_exclusive=rejection, insufficient_points_split=insufficient,
        valid_with_display_triangles=len(used), valid_without_display_triangles=int(valid.sum()-len(used)),
        valid_without_display_reasons_exclusive=no_face_reasons,
        independent_patch_triangles=len(faces),
        valid_without_assembly=int((valid & ~assembled).sum()), assembled_triangles=len(mesh)//3,
        metrics=metrics, saved_fit_recheck_passed=True,
    )


def main():
    result = dict(
        counting_unit='local fitting centres; not raw-point fraction or covered surface area',
        config='20x20 cm XY window, +/-4 cm relative Z, minimum 30 points; fit ALL eligible points',
        fit_model='z(x,y) quadratic height graph, not general implicit quadric fitting',
        display='Saved local functions sampled on 5x5 grids; actual triangles clipped to observed XY hull and +/-4 cm relative Z',
        interpretation='Solid white is a valid fitted surface with a low metric. Background gaps have no displayed triangles. A grey solid surface has an unknown metric.',
        insufficient_split_note='XY-only counts diagnose gate effects; including other heights is not a validated fix.',
        scenes=[audit(name) for name in ('forest', 'table')],
    )
    output = WEB / 'metrics' / 'fit-coverage-audit.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(output)
    for scene in result['scenes']:
        print(json.dumps(scene, ensure_ascii=False))


if __name__ == '__main__':
    main()
