"""Independent geometry replay of integrated states, ignoring planner labels.

Replay only performs kinematics and signed-distance queries, not new execution.
"""
import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np
from model import load_recorded_model


def validate(directory,method='searched_shift'):
    p=Path(directory);m=load_recorded_model(p/'model.xml');d=mujoco.MjData(m)
    states=np.load(p/f'{method}_states.npz')
    obstacle=m.geom('obstacle').id;floor=m.geom('floor').id
    geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g]>0 and m.geom_contype[g]>0]
    rows=[];worst=(float('inf'),None,None);inconsistent=0
    # With MuJoCo 3.8.0, a large (2 m) cutoff produced false zero distances
    # for a distant box-box pair in this model. Use a local clearance band
    # and verify returned witness-point lengths whenever a pair is inside it.
    cutoff=.05
    def distance(g,other):
        nonlocal inconsistent
        witness=np.zeros(6)
        value=float(mujoco.mj_geomDistance(m,d,g,other,cutoff,witness))
        if value < cutoff-1e-6 and abs(abs(value)-np.linalg.norm(witness[:3]-witness[3:]))>1e-4:
            inconsistent+=1
        return value
    for t,q in zip(states['time'],states['qpos']):
        d.qpos[:]=q;mujoco.mj_kinematics(m,d)
        distances=[distance(g,obstacle) for g in geoms]
        ground=[distance(g,floor) for g in geoms
                if 'wheel' not in m.body(int(m.geom_bodyid[g])).name]
        j=int(np.argmin(distances))
        if distances[j]<worst[0]:worst=(distances[j],float(t),m.geom(geoms[j]).name or f'geom{geoms[j]}')
        rows.append([float(t),min(distances),min(ground)])
    a=np.array(rows)
    result=dict(scope='Signed-distance checks at every saved physical state; no inter-step continuous guarantee',
        physical_state_count=len(rows),collision_geometries=len(geoms),
        distance_cutoff_m=cutoff,distances_at_cutoff_are_censored=True,
        inconsistent_witness_queries=inconsistent,
        min_robot_obstacle_clearance_m=float(a[:,1].min()),
        min_nonwheel_floor_clearance_m=float(a[:,2].min()),
        robot_obstacle_penetrating_states=int(np.sum(a[:,1]<-1e-6)),
        nonwheel_floor_penetrating_states=int(np.sum(a[:,2]<-1e-6)),
        closest_obstacle_geom=worst[2],closest_obstacle_time_s=worst[1],
        geometry_validation_passed=bool(inconsistent==0 and np.all(a[:,1:]>=-1e-6)))
    with (p/f'{method}_clearances.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['t','robot_obstacle_clearance_m','nonwheel_floor_clearance_m']);w.writerows(rows)
    (p/f'{method}_geometry_validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);ap.add_argument('--method',default='searched_shift')
    a=ap.parse_args();validate(a.directory,a.method)
