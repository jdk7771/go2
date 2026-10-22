"""Publish compact evidence, pinned model provenance and selected full traces.

Large development CSVs remain local. Every attempted case summary is exported,
including failures and planning rejections. Published trial NPZ files retain
every integrated state; selected telemetry is also stored as gzip CSV.
"""
import csv
from datetime import datetime
import gzip
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from zoneinfo import ZoneInfo

import matplotlib
import mujoco
import numpy as np
import scipy

from model import ROOT, SOURCE, Robot, build_model


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    out=ROOT/'published';out.mkdir(exist_ok=True)
    groups={}
    for name in ['pilot_upstream','pilot_single','pilot_gravity','formal_gravity','sensitivity_dt001',
                 'e1_pilot','e1_search','e1_gravity_gated','e1_feedback','e1_contact_feedback',
                 'e1_height_sweep','e1_dt001','failure_screen','body_reference_screen','robustness_screen',
                 'contact_point_screen','retarget_screen','retarget_validation','landing_reference_screen']:
        p=ROOT/name/'summary.json'
        if p.exists():groups[name]=json.loads(p.read_text())
    coupled=[]
    for p in sorted((ROOT/'coupled_landing_screen').glob('*/searched_shift.json')):
        row=json.loads(p.read_text());row['case_directory']=str(p.parent.relative_to(ROOT));coupled.append(row)
    if coupled:groups['coupled_landing_screen']=coupled
    (out/'trial_summaries.json').write_text(json.dumps(groups,indent=2))
    context=dict(final_height_screen='e1_height_sweep',final_perturbation_screen='robustness_screen',
        retained_historical_groups=['e1_search','e1_pilot','e1_feedback','e1_gravity_gated','e1_contact_feedback'],
        note='Historical e1_search success flags predate the strict 3 cm landing-error gate. Retain them as development history; do not count them as passes under current gates. Each deterministic cell is one development case, not a success-rate estimate.')
    (out/'trial_group_context.json').write_text(json.dumps(context,indent=2))
    fields=['group','method','name','height_m','dt','controller','body_gain','body_raise_m','swing_clearance_m',
            'actual_friction','push_force_y_n','contact_point_feedback','retarget_swing','relax_landing','coupled_landing','retarget_max_ik_error_m',
            'retarget_ik_over_1mm_fraction','terminal_reason','diagnostic_success','landing_error_xy_m',
            'max_tilt_deg','slip_p95_m_s','saturation_fraction','forbidden_contact_steps']
    with (out/'trial_table.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader()
        for group,rows in groups.items():
            for row in rows:
                r={k:row.get(k) for k in fields};r['group']=group
                r['height_m']=row.get('initial_obstacle',{}).get('size',[0,0,None])[2]
                if r['height_m'] is not None:r['height_m']*=2
                w.writerow(r)
    selected={
        'e1_h043':'e1_height_sweep/h0.043',
        'e1_h086':'e1_height_sweep/h0.086',
        'e1_h129_failed':'e1_height_sweep/h0.129',
        'e1_dt001':'e1_dt001',
        'e1_body_only_feedback_failed':'e1_feedback',
        'e1_h129_retarget':'retarget_validation/nominal',
        'e1_h129_lateral_return_rejected':'landing_reference_screen/retarget0',
        'e1_h129_coupled_initial_rejected':'coupled_landing_screen/retarget',
        'e1_h129_coupled_expanded_rejected':'coupled_landing_screen/expanded_support',
    }
    manifest=[]
    for name,source in selected.items():
        origin=ROOT/source;dest=out/name;dest.mkdir(exist_ok=True)
        for file in ['model.xml','searched_shift_states.npz','searched_shift_plan.npz','searched_shift.json',
                     'searched_shift_events.json','searched_shift_sat_validation.json',
                     'searched_shift.mp4','searched_shift_montage.png']:
            p=origin/file
            if p.exists():
                shutil.copy2(p,dest/file)
                if p.suffix=='.xml':
                    (dest/file).write_text('\n'.join(line.rstrip() for line in p.read_text().splitlines())+'\n')
                manifest.append(dict(source=str(p.relative_to(ROOT)),published=str((dest/file).relative_to(ROOT)),
                                     bytes=(dest/file).stat().st_size,sha256=digest(dest/file),source_sha256=digest(p)))
        telemetry=origin/'searched_shift.csv'
        if telemetry.exists():
            with telemetry.open('rb') as src,(dest/'telemetry.csv.gz').open('wb') as raw:
                with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0,filename='') as dst:shutil.copyfileobj(src,dst)
    for name in ['fixed_wheel_body_slices.png','summary.json','samples.csv','samples.npz']:
        p=ROOT/'workspace'/name
        if p.exists():
            target=out/'workspace';target.mkdir(exist_ok=True);shutil.copy2(p,target/name)
            if p.suffix=='.csv':(target/name).write_text(p.read_text())
    checks=ROOT/'recorded_trial_checks.json'
    if checks.exists():shutil.copy2(checks,out/checks.name)
    robust_geometry=ROOT/'robustness_screen'/'geometry_summary.json'
    if robust_geometry.exists():shutil.copy2(robust_geometry,out/'robustness_geometry_summary.json')
    retarget_checks=ROOT/'retarget_validation'/'validation.json'
    if retarget_checks.exists():shutil.copy2(retarget_checks,out/'retarget_validation.json')
    landing_checks=ROOT/'landing_reference_screen'/'validation.json'
    if landing_checks.exists():shutil.copy2(landing_checks,out/'landing_reference_validation.json')
    m,_=build_model(variant='single_collision');robot=Robot(m)
    dependency=SOURCE.parents[2]
    commit=subprocess.check_output(['git','-C',str(dependency),'rev-parse','HEAD'],text=True).strip()
    license_text=subprocess.check_output(['git','-C',str(dependency),'show','HEAD:LICENSE'],text=True)
    (out/'UNITREE_LICENSE.txt').write_text(license_text)
    provenance=dict(generated_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        source_repository='https://github.com/unitreerobotics/unitree_mujoco',source_commit=commit,
        source_model_sha256=digest(SOURCE),
        python=sys.version,platform=platform.platform(),
        packages=dict(mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__,matplotlib=matplotlib.__version__),
        model=dict(mass_kg=float(m.body_mass.sum()),wheel_radius_m=robot.radius,
                   joint_count=m.njnt,motor_count=m.nu,ctrlrange_nm=m.actuator_ctrlrange.tolist()),
        scripts={p.name:digest(p) for p in ROOT.glob('*.py')},
        meshes={p.name:digest(p) for p in (SOURCE.parent/'assets').iterdir() if p.is_file()},
        evidence_scope='Development experiments on a public model. No RL training, hardware calibration, general success-rate claim or real-time guarantee.')
    provenance['code_version_scope']='Scripts are hashed at export time. Original 15 trial groups were checkpointed at 1ad63a12e36a7c4ab2d3c3b3878eb4c7c554355f. New trials may also include execution_code_sha256; defaults preserve the original controller.'
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    by_path={row['published']:row for row in manifest}
    manifest=[]
    for p in sorted(out.rglob('*')):
        if not p.is_file() or p.name=='manifest.json':continue
        relative=str(p.relative_to(ROOT))
        row=by_path.get(relative,dict(published=relative))
        row.update(bytes=p.stat().st_size,sha256=digest(p));manifest.append(row)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(f'Published {len(groups)} result groups and {len(selected)} selected complete traces.')


if __name__=='__main__':main()
