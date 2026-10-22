"""Decompose saved wheel-center error using counterfactual forward kinematics.

These are offline kinematic evaluations, not rerun physical trajectories or
causal proof. No original state or execution result is modified.
"""
import json
from pathlib import Path

import mujoco
import numpy as np

from model import ROOT, LEGS, Robot, load_recorded_model


def diagnose(path):
    path=Path(path)
    m=load_recorded_model(path/'model.xml');r=Robot(m);d=r.d
    states=np.load(path/'searched_shift_states.npz')
    plan=np.load(path/'searched_shift_plan.npz')
    records=[]
    for requested in [6.5,8.,9.,11.5,12.5]:
        k=int(np.argmin(abs(states['time']-requested)));t=float(states['time'][k])
        qactual=states['qpos'][k].copy()
        qplanned=qactual.copy()
        qplanned[:3]=[np.interp(t,plan['time'],plan['body'][:,i]) for i in range(3)]
        qplanned[3:7]=[1.,0.,0.,0.]
        qplanned[r.qa[:12]]=[np.interp(t,plan['time'],plan['q'][:,i]) for i in range(12)]
        targets=np.array([[np.interp(t,plan['time'],plan['centers'][:,i,j]) for j in range(3)] for i in range(4)])
        def centers(q):
            d.qpos[:]=q;mujoco.mj_forward(m,d)
            return r.centers(d).copy()
        nominal=centers(qplanned)
        # Exact telescoping decomposition: change base, then joints.
        base_only=qplanned.copy();base_only[:7]=qactual[:7]
        moved_base=centers(base_only)
        actual=centers(qactual)
        contacts=[]
        for i,b in enumerate(r.wheel_bodies):
            points=[]
            for c in d.contact:
                bodies=[int(m.geom_bodyid[g]) for g in [c.geom1,c.geom2]]
                if r.floor in [c.geom1,c.geom2] and b in bodies and c.efc_address>=0:
                    points.append(c.pos.copy())
            contact=np.mean(points,axis=0) if points else None
            axis=d.xmat[b].reshape(3,3)[:,1]
            contacts.append(dict(leg=LEGS[i],active_contact_count=len(points),
                wheel_axis_elevation_deg=float(np.rad2deg(np.arcsin(np.clip(axis[2],-1,1)))),
                mean_geometric_contact_minus_center_m=(contact-actual[i]).tolist() if contact is not None else None))
        records.append(dict(time_s=t,base_error_xyz_m=(qactual[:3]-qplanned[:3]).tolist(),
            joint_error_rad=(qactual[r.qa[:12]]-qplanned[r.qa[:12]]).tolist(),
            fl_total_error_xyz_m=(actual[0]-targets[0]).tolist(),
            fl_planned_ik_error_xyz_m=(nominal[0]-targets[0]).tolist(),
            fl_base_pose_contribution_xyz_m=(moved_base[0]-nominal[0]).tolist(),
            fl_joint_contribution_after_base_xyz_m=(actual[0]-moved_base[0]).tolist(),
            contact_geometry=contacts))
    return dict(source=str(path.relative_to(ROOT)),
        scope='Kinematic telescoping decomposition; contact points are geometric means, not force-weighted centers of pressure; not causal attribution.',
        records=records)


if __name__=='__main__':
    results=[diagnose(ROOT/'published'/name) for name in ['e1_h043','e1_h086','e1_h129_failed']]
    (ROOT/'published'/'landing_diagnosis.json').write_text(json.dumps(results,indent=2))
    for result in results:
        row=result['records'][-1]
        print(result['source'],json.dumps({k:v for k,v in row.items() if k!='contact_geometry'}))
