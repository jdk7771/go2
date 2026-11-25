"""Export full-scene, single-frame FSGP maps for the existing 42482 viewer.

Uses the saved assessment XYZ verbatim. Independent 5 m local GP windows own
disjoint 20 cm world cells; a 45 cm input margin supplies edge observations.
This reuses the paper-equation CPU reference, not the upstream GPU trainer.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree

from .assessment_export import finite_json
from .core import Config, TraversabilityMapper
from .scene_fit_catalog import ORDER, NAMES, WEB

OUT = WEB / 'fsgp'
FIELDS = ['x', 'y', 'z', 'height', 'valid', 'gp_cost', 'gp_preliminary',
          'slope_deg', 'curvature', 'height_variation_m', 'gp_std_m',
          'gp_variance_m2', 'tile_id']


def grid_faces(shape, known):
    """Only connect adjacent, supported GP queries; never bridge missing cells."""
    ids = np.arange(np.prod(shape)).reshape(shape)
    a, b, c, d = (ids[:-1, :-1].ravel(), ids[:-1, 1:].ravel(),
                  ids[1:, :-1].ravel(), ids[1:, 1:].ravel())
    faces = np.concatenate((np.column_stack((a, b, c)), np.column_stack((b, d, c))))
    return faces[np.asarray(known)[faces].all(axis=1)].astype('<u4')


def compute(points, config=Config()):
    start = perf_counter()
    points = np.asarray(points, dtype='<f4').astype(float)
    lo = np.floor(points[:, :2].min(axis=0) / config.resolution).astype(int)
    hi = np.ceil(points[:, :2].max(axis=0) / config.resolution).astype(int)
    hi = np.maximum(hi, lo + 1)
    shape = (int(hi[1] - lo[1]), int(hi[0] - lo[0]))
    x, y = np.meshgrid(np.arange(lo[0], hi[0]), np.arange(lo[1], hi[1]))
    cell_ids = np.column_stack((x.ravel(), y.ravel()))
    xy = (cell_ids + .5) * config.resolution
    _, nearest = cKDTree(points[:, :2]).query(xy)
    packed = np.full((len(xy), len(FIELDS)), np.nan, dtype='<f8')
    col = {key: i for i, key in enumerate(FIELDS)}
    packed[:, :3] = np.column_stack((xy, points[nearest, 2]))
    packed[:, col['valid']] = 0
    packed[:, col['tile_id']] = -1
    tiles, inducing = [], []
    side = round(config.length / config.resolution)
    # Stable world coordinates, including negative cells, with one owner/query.
    for row in range(0, shape[0], side):
        for column in range(0, shape[1], side):
            origin = lo + [column, row]
            center = (origin + side // 2 + .5) * config.resolution
            margin = config.length / 2 + config.support_radius
            selected = np.max(np.abs(points[:, :2] - center), axis=1) <= margin
            if np.count_nonzero(selected) < 3:
                continue
            result = TraversabilityMapper(config).assess(points[selected], center=center)
            offset = result['cell_ids'] - lo
            keep = ((offset >= 0).all(axis=1) & (offset[:, 0] < shape[1]) &
                    (offset[:, 1] < shape[0]))
            ids = offset[keep, 1] * shape[1] + offset[keep, 0]
            if np.any(packed[ids, col['tile_id']] >= 0):
                raise ValueError('overlapping local map ownership')
            tile = len(tiles)
            for key, values in {
                'height': result['height'], 'valid': result['known'],
                'gp_cost': result['cost'].ravel(), 'gp_preliminary': result['preliminary'],
                'slope_deg': np.rad2deg(np.arctan(result['slope'])),
                'curvature': result['curvature'], 'height_variation_m': result['height_variation'],
                'gp_std_m': np.sqrt(result['variance']), 'gp_variance_m2': result['variance'],
            }.items():
                packed[ids, col[key]] = values[keep]
            packed[ids, col['tile_id']] = tile
            inducing.append(np.column_stack((np.full(len(result['inducing']), tile), result['inducing'])))
            tiles.append(dict(id=tile, center=center.tolist(), input_points=len(result['points']),
                              inducing_points=len(result['inducing']), queries=len(ids),
                              known=int(result['known'][keep].sum()), timing_ms=result['timing_ms']))
    known = packed[:, col['valid']].astype(bool)
    # Unsupported scores remain unknown in both CSV and binary. Keep posterior
    # variance/height for diagnosis without treating unsupported queries as valid.
    packed[~known, col['gp_cost']] = np.nan
    faces = grid_faces(shape, known)
    return dict(models=packed, triangles=faces, cell_ids=cell_ids, shape=shape,
                inducing=np.concatenate(inducing) if inducing else np.empty((0, 6)),
                tiles=tiles, total_ms=(perf_counter() - start) * 1000)


def export_scene(name, count, output_root=OUT):
    source = WEB / 'assessment' / name
    data_path = source / 'points.bin'
    if not data_path.exists():
        raise FileNotFoundError(f'Generate the assessment scene first: {data_path}')
    points = np.fromfile(data_path, dtype='<f4').reshape(-1, 3)
    original = json.loads((source / 'data.json').read_text())
    config = Config(inducing_points=count)
    result = compute(points, config)
    output = Path(output_root) / f'm{count}' / name
    output.mkdir(parents=True, exist_ok=True)
    result['models'].tofile(output / 'models.bin')
    result['triangles'].tofile(output / 'mesh.bin')
    np.savez_compressed(output / 'result.npz', models=result['models'],
                        triangles=result['triangles'], cell_ids=result['cell_ids'],
                        inducing=result['inducing'])
    with (output / 'per-model.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['query', *FIELDS])
        for i, row in enumerate(result['models']):
            writer.writerow([i, *[float(v) if np.isfinite(v) else '' for v in row]])
    known = result['models'][:, FIELDS.index('valid')].astype(bool)
    meta = dict(id=name, name=NAMES[name], method='fsgp', raw_points=len(points),
                source=dict(path=str(data_path.relative_to(WEB)),
                            sha256=hashlib.sha256(data_path.read_bytes()).hexdigest(),
                            assessment_source=original['source']),
                centers=len(known), valid=int(known.sum()), solved=int(np.isfinite(result['models'][:, 3]).sum()),
                triangles=len(result['triangles']), bounds=original['bounds'], fields=FIELDS,
                config=config.as_dict(), shape=result['shape'], tiles=result['tiles'],
                gp_solves=len(result['tiles']), inducing_total=sum(t['inducing_points'] for t in result['tiles']),
                total_ms=result['total_ms'], raw_points_url=f'../../../assessment/{name}/points.bin',
                protocol='Single static full cloud, no extra noise/occlusion or history. Disjoint 5 m local GP windows, 20 cm grid. No per-scene min-max cost normalization.',
                implementation='Paper v2 Eqs. (1)-(8),(11) CPU reference, fixed RQ kernel. GP conditioning on retained observations, not upstream GPU ExactGP + InducingPointKernel optimization.',
                limitations='Single-height z=f(x,y); all height layers retained as input. No body-clearance or contact model. No temporal BGK history in this static view.',
                paper_url='https://arxiv.org/abs/2503.04134v2',
                upstream_url='https://github.com/ZJU-FAST-Lab/FSGP_BGK')
    (output / 'data.json').write_text(json.dumps(finite_json(meta), ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(scene=name, M=count, tiles=meta['gp_solves'], known=meta['valid'],
                          queries=meta['centers'], ms=round(meta['total_ms'], 1))), flush=True)
    return meta


def publish(output_root=OUT):
    output_root = Path(output_root)
    methods = []
    for count in (125, 500):
        scenes = [name for name in ORDER if (output_root / f'm{count}' / name / 'data.json').exists()]
        if scenes:
            methods.append(dict(id=f'fsgp-{count}', label=f'FSGP · M={count}',
                                path=f'../fsgp/m{count}/', scene_ids=scenes))
    output_root.mkdir(exist_ok=True, parents=True)
    (output_root / 'catalog.json').write_text(json.dumps(dict(methods=methods), ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes', nargs='+', choices=ORDER, default=ORDER)
    parser.add_argument('--inducing', nargs='+', choices=[125, 500], type=int, default=[125, 500])
    args = parser.parse_args()
    for name in args.scenes:
        for count in args.inducing:
            export_scene(name, count)
        publish()


if __name__ == '__main__':
    main()
