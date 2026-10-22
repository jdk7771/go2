"""E0 fixed-wheel-center IK slices on the full Go2W kinematic model.

This is a sampled kinematic set; accepted points are not claimed dynamically
stable. It uses the upstream collision primitives, not all visual mesh surfaces.
"""
import csv
import json
import time

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from model import ROOT, Robot, build_model, quat_pitch


def main():
    out=ROOT/'workspace';out.mkdir(exist_ok=True)
    m,_=build_model(variant='single_collision');robot=Robot(m)
    heights=np.linspace(.30,.48,19)
    pitches=np.linspace(-20,20,21)
    shifts=[-.06,0.,.06]
    rows=[];all_q=[];images=[];start=time.monotonic()
    for y in shifts:
        grid=np.zeros((len(heights),len(pitches)))
        for iz,z in enumerate(heights):
            previous=robot.nominal[:12].copy()
            for ip,pitch in enumerate(pitches):
                base=np.array([0.,y,z]);quat=quat_pitch(np.deg2rad(pitch))
                q,err=robot.ik(base,quat,robot.centers0,previous,centers=True)
                if err>.001:
                    q2,e2=robot.ik(base,quat,robot.centers0,robot.nominal[:12],centers=True)
                    if e2<err:q,err=q2,e2
                previous=q
                # Re-evaluate independently in a fresh data instance.
                d=mujoco.MjData(m);d.qpos[:]=robot.q0
                d.qpos[:3]=base;d.qpos[3:7]=quat;d.qpos[robot.qa[:12]]=q
                mujoco.mj_forward(m,d)
                residual=float(np.linalg.norm(robot.centers(d)-robot.centers0,axis=1).max())
                colliding=[]
                for k in range(d.ncon):
                    c=d.contact[k]
                    if c.dist < -.001:
                        colliding.append([int(c.geom1),int(c.geom2),float(c.dist)])
                feasible=residual<=.001 and not colliding
                grid[iz,ip]=2 if feasible else 1 if residual<=.001 else 0
                rows.append(dict(body_y_m=y,body_z_m=float(z),pitch_deg=float(pitch),
                                 residual_m=residual,collision_count=len(colliding),
                                 max_penetration_m=max([-x[2] for x in colliding],default=0),
                                 feasible=feasible))
                all_q.append(q)
        images.append(grid)
    with (out/'samples.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    np.savez_compressed(out/'samples.npz',joint_positions=all_q,wheel_centers=robot.centers0,
                        heights=heights,pitches=pitches,lateral_shifts=shifts)
    summary=dict(scope='Full Go2W bounded IK and upstream-primitive collision screen; not a stable workspace',
                 samples=len(rows),feasible=sum(r['feasible'] for r in rows),
                 ik_tolerance_m=.001,penetration_tolerance_m=.001,
                 ground_clearance_at_nominal_m=.002,
                 wall_seconds=time.monotonic()-start,
                 slices=[dict(body_y_m=y,feasible=sum(r['feasible'] for r in rows if r['body_y_m']==y),
                              total=len(heights)*len(pitches)) for y in shifts])
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    from matplotlib.colors import ListedColormap,BoundaryNorm
    cmap=ListedColormap(['#dddde1','#dd9460','#287f83'])
    fig,axes=plt.subplots(1,3,figsize=(12,4.5),sharey=True,layout='constrained')
    for ax,y,grid in zip(axes,shifts,images):
        im=ax.imshow(grid,origin='lower',extent=[-21,21,.295,.485],aspect='auto',
                     cmap=cmap,norm=BoundaryNorm([-.5,.5,1.5,2.5],3))
        ax.set(title=f'Body lateral shift {y:+.2f} m',xlabel='Pitch (deg)')
        ax.axhline(robot.z0,color='black',ls='--',lw=.8)
    axes[0].set_ylabel('Body height (m)')
    cb=fig.colorbar(im,ax=axes,ticks=[0,1,2],shrink=.8)
    cb.ax.set_yticklabels(['IK failed','IK + collision','IK + collision clear'])
    fig.suptitle('Fixed four wheel centers: sampled Go2W kinematic feasibility\nNot a dynamically stable workspace')
    fig.savefig(out/'fixed_wheel_body_slices.png',dpi=170)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
