"""Time every saved 20 cm scene without replacing its numerical results."""
import argparse
import csv
import gc
import hashlib
import json
import platform
import time
from pathlib import Path
import numpy as np

from .assessment_export import OUT, ORDER, NAMES, assessment_source, compute

TIMING = OUT/'timing'
STAGES = ['select_centers_ms','window_query_ms','fit_and_covariance_ms','assembly_ms',
          'display_patch_geometry_ms','three_jump_methods_ms','clearance_ms',
          'local_costs_ms','body_50x80_ms']
# A graph shares these stages with other graphs. Sums describe dependencies,
# not independently timed executions or additive per-graph job costs.
BASE = STAGES[:3]
VIEW_STAGES = {
    'cloud': [],
    'fit': BASE+['display_patch_geometry_ms'],
    'mesh': BASE+['assembly_ms'],
    'error': BASE+['display_patch_geometry_ms'],
    'clearance': BASE+['display_patch_geometry_ms','clearance_ms'],
    'jump': BASE+['display_patch_geometry_ms','three_jump_methods_ms'],
    'foot': BASE+['display_patch_geometry_ms','three_jump_methods_ms','clearance_ms','local_costs_ms'],
    'body': BASE+['display_patch_geometry_ms','three_jump_methods_ms','clearance_ms','local_costs_ms','body_50x80_ms'],
}
VIEW_NAMES = dict(cloud='原始点云',fit='独立二次曲面',mesh='组合曲面',error='误差与不确定性',
                  clearance='10–60 cm 净空',jump='A/B/C 跳变',foot='落足综合',body='50×80 cm 机身')


def describe(values):
    values=np.asarray(values,dtype=float)
    return dict(median_ms=float(np.median(values)),min_ms=float(values.min()),max_ms=float(values.max()))


def save_json(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def export(records):
    TIMING.mkdir(parents=True,exist_ok=True)
    with (TIMING/'compute-runs.csv').open('w') as stream:
        fields=['scene','run','raw_points','centers','primary_solves','valid','side_solves',
                'source_prepare_ms',*STAGES,'total_assessment_ms','compute_call_wall_ms']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for scene in records:
            for i,run in enumerate(scene['runs']):
                row=dict(scene=scene['id'],run=i+1,**scene['counts'],**run)
                writer.writerow({key:row[key] for key in fields})
    with (TIMING/'compute-summary.csv').open('w') as stream:
        fields=['scene','name','repeats',*records[0]['counts'],*STAGES,'total_assessment_ms',
                'total_min_ms','total_max_ms','source_prepare_ms','compute_call_wall_ms']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for scene in records:
            row=dict(scene=scene['id'],name=scene['name'],repeats=scene['repeats'],**scene['counts'],
                     **{key:value['median_ms'] for key,value in scene['summary'].items()},
                     total_min_ms=scene['summary']['total_assessment_ms']['min_ms'],
                     total_max_ms=scene['summary']['total_assessment_ms']['max_ms'])
            writer.writerow({key:row[key] for key in fields})
    with (TIMING/'per-view-compute.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['scene','view','view_name','repeats','dependency_sum_median_ms',
            'dependency_sum_min_ms','dependency_sum_max_ms','dependencies','measurement_kind'])
        writer.writeheader()
        for scene in records:
            for key,stages in VIEW_STAGES.items():
                values=describe([sum(run[k] for k in stages) for run in scene['runs']])
                writer.writerow(dict(scene=scene['id'],view=key,view_name=VIEW_NAMES[key],repeats=scene['repeats'],
                    dependency_sum_median_ms=values['median_ms'],dependency_sum_min_ms=values['min_ms'],
                    dependency_sum_max_ms=values['max_ms'],dependencies='+'.join(stages),
                    measurement_kind='sum of measured dependent stages; not an independent graph execution'))
    cpu='unknown'
    for line in Path('/proc/cpuinfo').read_text().splitlines():
        if line.startswith('model name'):
            cpu=line.split(':',1)[1].strip();break
    report=dict(experiment_date='2026-11-17',timezone='Asia/Shanghai',scenes=records,
        environment=dict(cpu=cpu,python=platform.python_version(),numpy=np.__version__,blas_threads=1),
        stage_order=STAGES,view_dependencies=VIEW_STAGES,view_names=VIEW_NAMES,
        notes=[
            'All 17 saved scene IDs use the unchanged strict 20 cm configuration; source XYZ and computed numerical models are checked against the published data.',
            'Each scene is run sequentially 3 times by default. Report median and observed min/max, not a confidence interval.',
            'total_assessment_ms includes the nine recorded compute stages. Source generation/read/decompression, initial dtype conversion, output packing, file export and browser rendering are outside that value.',
            'compute_call_wall_ms includes initial dtype conversion and output packing but not source preparation or result export. source_prepare_ms is reported separately; OS caches are not flushed.',
            'Per-view dependency sums are calculated within each same run, then summarised. They reuse stages and must not be added across views. They are not separate from-XYZ view execution benchmarks.',
            'A/B/C share the jump stage in this implementation. Changing a selector in the webpage recolors saved data and does not rerun the Python algorithm.',
            'These are whole-scene point clouds, not individual 20 cm patches or video image frames.',
        ])
    save_json(TIMING/'compute.json',report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes',nargs='+',choices=ORDER,default=ORDER)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    if args.repeats<1:parser.error('--repeats must be positive')
    records=[]
    for name in args.scenes:
        folder=OUT/name;meta=json.loads((folder/'data.json').read_text())
        frozen=np.fromfile(folder/'models.bin',dtype='<f8').reshape(meta['centers'],-1)
        frozen_hash=hashlib.sha256((folder/'points.bin').read_bytes()).hexdigest()
        runs=[]
        for rep in range(args.repeats):
            gc.collect()
            start=time.perf_counter();points,_=assessment_source(name)
            source_ms=(time.perf_counter()-start)*1000
            assert hashlib.sha256(points.astype('<f4').tobytes()).hexdigest()==frozen_hash, name+' source changed'
            start=time.perf_counter();result=compute(points)
            wall_ms=(time.perf_counter()-start)*1000
            np.testing.assert_allclose(result['packed'],frozen,rtol=1e-10,atol=1e-12,equal_nan=True)
            counts=dict(raw_points=len(points),centers=len(result['seeds']),
                primary_solves=int(np.isfinite(result['fits']['rmse']).sum()),
                valid=int(np.isfinite(result['fits']['height']).sum()),
                side_solves=result['jumps']['solve_counts']['actual_side_solves'])
            run=dict(source_prepare_ms=source_ms,**result['times'],compute_call_wall_ms=wall_ms)
            runs.append(run)
            print(json.dumps(dict(scene=name,run=rep+1,total_ms=round(run['total_assessment_ms'],2)),ensure_ascii=False),flush=True)
            del result,points
        row=dict(id=name,name=NAMES[name],repeats=args.repeats,counts=counts,source_xyz_sha256=frozen_hash,
            models_match_published=True,runs=runs,summary={key:describe([run[key] for run in runs]) for key in runs[0]})
        records.append(row);export(records)
    print('Saved',len(records),'scenes to',TIMING,flush=True)


if __name__=='__main__':main()
