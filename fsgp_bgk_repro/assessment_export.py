"""Publish configurable assessment experiments on the existing port, preserving history."""
import argparse
from dataclasses import asdict, replace
import csv
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np

from .scene_fit_catalog import ORDER, NAMES, WEB, ROOT, source_cloud
from .table_local_surfaces import assemble, LocalConfig
from .terrain_assessment import (AssessmentConfig, window_pools, fit_pools, patches,
                                jump_metrics, clearance_metrics, local_metrics, body_metrics)

OUT = WEB / 'assessment'
FIELDS = ['x','y','z','c0','c1','c2','c3','c4','c5','height','valid','point_count',
          'rmse_m','max_error_m','std_center_m','std_patch_m','error_plus_2sigma_m',
          'slope_deg','normal_change_deg','jump_a_m','jump_b_m','jump_sigma_m',
          'jump_significance','jump_upper_m','jump_known_directions','clearance_cost',
          'raw_obstacle_hits','surface_obstacle_hits','roughness_cost','foot_cost',
          'body_cost','body_coverage','body_slope_deg','body_plane_rmse_m','body_support_margin_m',
          'body_max_jump_m','body_max_clearance','body_max_roughness','body_max_uncertainty','assembled']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_json(value):
    if isinstance(value, dict):
        return {str(k):finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def stats(values):
    good = np.asarray(values)[np.isfinite(values)]
    return dict(known=len(good), total=len(values), median=float(np.median(good)),
                p95=float(np.percentile(good,95)), maximum=float(np.max(good))) if len(good) else dict(known=0,total=len(values))


def assessment_source(name):
    # For analytic geometry, explicitly regenerate denser observations rather
    # than invent measurements by interpolating a previously delivered scan.
    spacing = .025
    if name in ('flat','ramp','flat_ramp','steps-single'):
        axis = np.linspace(-4,4,321)
        x,y = np.meshgrid(axis,axis)
        z = np.zeros_like(x)
        if name == 'ramp':
            z = .25*x+.10*y
        elif name == 'flat_ramp':
            z = .3*np.clip(x+1,0,2)
        elif name == 'steps-single':
            z = .45*(x>.15)+.15*np.exp(-((x+1.2)**2+(y-.5)**2)/.4)
        return np.column_stack((x.ravel(),y.ravel(),z.ravel())).astype('<f4'),dict(
            kind='analytic_regeneration',spacing_m=spacing,original_geometry_preserved=True,
            reason='A strict 20 cm window in the old 8 cm grid usually contains fewer than 30 points')
    if name == 'steps':
        from .mixed_steps_scene import build
        points, _, _, geometry = build(spacing)
        return points,dict(kind='analytic_regeneration',spacing_m=spacing,geometry=geometry)
    if name.startswith('corridor-'):
        from .corridor_scenes import build
        points, *_, geometry = build(int(name.split('-')[1])/100, spacing)
        return points,dict(kind='analytic_regeneration',spacing_m=spacing,geometry=geometry)
    if name == 'table':
        from .table_scene import make_cloud
        points, _, geometry = make_cloud(spacing)
        return points,dict(kind='analytic_regeneration',spacing_m=spacing,geometry=geometry)
    points, source = source_cloud(name)
    return points,dict(kind='unchanged_source_cloud',source=source)


def compute(points, config=AssessmentConfig()):
    points = np.asarray(points, dtype='<f4').astype(float)
    times={};start=time.perf_counter();last=start
    def mark(name):
        nonlocal last
        now=time.perf_counter();times[name]=(now-last)*1000;last=now
    _,source_ids=np.unique(np.floor(points/config.seed_voxel_m).astype(np.int64),axis=0,return_index=True)
    seeds=points[source_ids];mark('select_centers_ms')
    pools=window_pools(points,seeds,config);mark('window_query_ms')
    fits=fit_pools(points,seeds,pools,config);mark('fit_and_covariance_ms')
    labels,components,edges,assembly_info=assemble(seeds,fits,LocalConfig(patch_half_width_m=config.patch_size_m/2))
    faces=np.concatenate([c['triangles'] for c in components]).astype('<u4') if components else np.empty((0,3),dtype='<u4')
    used=np.zeros(len(seeds),bool);used[np.unique(faces)]=True;mark('assembly_ms')
    vertices,patch_faces,owners,std_max=patches(points,seeds,pools,fits,config);mark('display_patch_geometry_ms')
    jumps=jump_metrics(points,seeds,pools,fits,config);mark('three_jump_methods_ms')
    clearance=clearance_metrics(points,seeds,fits,vertices,patch_faces,config);mark('clearance_ms')
    metrics=local_metrics(seeds,fits,std_max,jumps,clearance,config);mark('local_costs_ms')
    body=body_metrics(seeds,fits,metrics,jumps,clearance,config);mark('body_50x80_ms')
    times['total_assessment_ms']=(time.perf_counter()-start)*1000
    packed=np.column_stack((seeds,fits['coefficient'],fits['height'],np.isfinite(fits['height']).astype(int),fits['counts'],
        fits['rmse'],fits['max_error'],np.sqrt(np.maximum(0,fits['covariance'][:,0,0])),std_max,
        fits['max_error']+config.sigma_multiplier*std_max,metrics['slope_deg'],metrics['normal_change_deg'],
        jumps['one_sided_m'],jumps['bilateral_m'],jumps['jump_sigma_m'],jumps['significance'],jumps['upper_m'],
        jumps['known_directions'],clearance['cost'],clearance['raw_hits'],clearance['surface_hits'],
        metrics['roughness_cost'],metrics['foot_cost'],body['risk'],body['coverage'],body['slope_deg'],
        body['plane_rmse_m'],body['support_margin_m'],body['max_jump_upper_m'],body['max_clearance_cost'],
        body['max_roughness_cost'],body['max_uncertainty_cost'],used.astype(int))).astype('<f8')
    assert packed.shape==(len(seeds),len(FIELDS))
    return dict(points=points,seeds=seeds,source_ids=source_ids,pools=pools,fits=fits,
                triangles=faces,patches=vertices,patch_faces=patch_faces,patch_owners=owners,
                packed=packed,labels=labels,jumps=jumps,clearance=clearance,body=body,
                times=times,components=len(components),assembly_info=assembly_info)


def export_scene(name,config=AssessmentConfig(),*,output_root=OUT):
    points,source=assessment_source(name)
    print(f'Assessing {name}: {len(points)} raw points',flush=True)
    result=compute(points,config)
    return save_scene(name,points,source,result,config,output_root=output_root)


def save_scene(name,points,source,result,config=AssessmentConfig(),*,output_root=OUT):
    """Serialize a computed result; alternate profiles never overwrite OUT."""
    output=Path(output_root)/name;output.mkdir(parents=True,exist_ok=True)
    fits=result['fits'];valid=np.isfinite(fits['height']);n=len(valid)
    for filename,array in (('points.bin',points.astype('<f4')),('models.bin',result['packed']),
                           ('mesh.bin',result['triangles']),('patches.bin',result['patches']),
                           ('patch-faces.bin',result['patch_faces']),('patch-owners.bin',result['patch_owners']),
                           ('covariance.bin',fits['covariance'].astype('<f8')),
                           ('body-contacts.bin',result['body']['contact_ids'].astype('<i4')),
                           ('body-contact-z.bin',result['body']['contact_z'].astype('<f8'))):
        array.tofile(output/filename)
    offsets=np.r_[0,np.cumsum([len(p) for p in result['pools']])].astype(np.int64)
    neighbors=np.concatenate(result['pools']) if n else np.empty(0,np.int32)
    offsets.astype('<i4').tofile(output/'neighbor-offsets.bin')
    neighbors.astype('<i4').tofile(output/'neighbors.bin')
    np.savez_compressed(output/'result.npz',models=result['packed'],covariance=fits['covariance'],
                        neighbor_offsets=offsets,neighbor_ids=neighbors,source_ids=result['source_ids'],
                        triangles=result['triangles'],body_contacts=result['body']['contact_ids'])
    with (output/'per-model.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['seed',*FIELDS])
        for i,row in enumerate(result['packed']):
            writer.writerow([i,*[float(x) if np.isfinite(x) else '' for x in row]])
    with (output/'jump-pairs.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['left','right','boundary_x','boundary_y','method_a_m','method_b_m','sigma_delta_m','method_c_upper_m'])
        for row in result['jumps']['pairs']:
            writer.writerow([float(x) if np.isfinite(x) else '' for x in row])
    meta=dict(id=name,name=NAMES[name],source=source,raw_points=len(points),centers=n,
              valid=int(valid.sum()),solved=int(np.isfinite(fits['rmse']).sum()),
              triangles=len(result['triangles']),patch_triangles=len(result['patch_faces']),components=result['components'],
              bounds=[points.min(0).tolist(),points.max(0).tolist()],config=asdict(config),fields=FIELDS,
              field_stats={key:stats(result['packed'][:,i]) for i,key in enumerate(FIELDS) if i>=11},
              timings=dict(repeats=1,stages_ms=result['times'],note='One CPU run, single-thread BLAS. Includes patch geometry and all assessment stages; excludes source generation, file IO and browser rendering. Not the old fit-only benchmark.'),
              jump_methods={'A':'max of two one-sided quadratic endpoint prediction residuals',
                            'B':'two disjoint half-plane fits evaluated at the same candidate midpoint',
                            'C':'method B + 2*sigma_delta; significance is separate, not traversability'},
              jump_solve_counts=result['jumps']['solve_counts'],
              uncertainty='Finite-ridge sandwich covariance, RSS/(n-6), 3 mm noise floor; prediction variance phi.T@cov@phi. Conditional linear-model estimate, not calibrated sensor uncertainty.',
              clearance=f'Observed occupied 5 cm columns in a {config.patch_size_m*100:g} cm footprint, 10..60 cm above fitted surface. Raw points plus vertices of actual fitted triangles. Sampled surface evidence, not exact continuous collision detection or certified free space.',
              body='80 cm along X, 50 cm along Y, assumed feet +/-35 cm and +/-20 cm; COM height 30 cm. Geometric quasi-static proxy only; no dynamics/friction/real mass model.',
              unknown='NaN in binaries / empty in CSV. Unsupported jump sides and body coverage are never zero-filled.')
    (output/'data.json').write_text(json.dumps(finite_json(meta),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(scene=name,centers=n,valid=meta['valid'],body_known=meta['field_stats']['body_cost']['known'],ms=result['times']['total_assessment_ms'])),flush=True)
    return meta


def publish(output_root=OUT,*,copy_assets=True):
    output_root=Path(output_root);output_root.mkdir(exist_ok=True,parents=True)
    cases=[]
    for name in ORDER:
        path=output_root/name/'data.json'
        if path.exists():cases.append(json.loads(path.read_text()))
    (output_root/'catalog.json').write_text(json.dumps(dict(scenes=cases,default_scene='steps',fields=FIELDS),ensure_ascii=False,indent=2)+'\n')
    assets=ROOT/'visualization/assessment'
    if copy_assets and assets.exists():
        for path in assets.iterdir():
            if path.is_file():shutil.copyfile(path,output_root/path.name)
    return cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes',nargs='+',choices=ORDER,default=ORDER)
    args=parser.parse_args()
    for name in args.scenes:
        export_scene(name);publish()


if __name__=='__main__':main()
