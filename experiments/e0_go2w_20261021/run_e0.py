"""Motor-driven Go2W diagnostics, not an RL evaluation or hardware validation."""
import argparse
import csv
import hashlib
import json
import platform
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from model import ROOT, SOURCE, LEGS, Robot, build_model, smooth, quat_pitch


def run_trial(name,variant='single_collision',dt=.002,out=None,controller='pd'):
    m,xml=build_model(dt,variant)
    robot=Robot(m); d=robot.d
    duration=8. if name.startswith(('roll','turn')) else 10.
    # A time-parameterized reference is computed in separate IK data only.
    # No simulated qpos/qvel are assigned after initialization.
    samples=np.linspace(0,duration,int(duration*50)+1)
    targets=[]; max_ik=0.; qprev=robot.nominal[:12].copy()
    for t in samples:
        body=robot.q0[:3].copy(); axles=robot.axles0.copy(); quat=robot.q0[3:7].copy()
        if name.startswith('height'):
            amplitude=float(name.split('_')[1])
            body[2]+=amplitude*(smooth((t-2)/2)-smooth((t-6)/2))
        if name.startswith('pitch'):
            pitch=float(name.split('_')[1])*(smooth((t-2)/2)-smooth((t-6)/2))
            quat=quat_pitch(np.deg2rad(pitch))
        if name.startswith('lift'):
            if name=='lift_shift':
                body[:2]+=np.array([-.045,-.055])*(smooth((t-2)/1.5)-smooth((t-7)/1.5))
            axles[0,2]+=.06*(smooth((t-3.5)/1)-smooth((t-5.5)/1))
        if name.startswith(('height','pitch','lift')):
            qprev,err=robot.ik(body,quat,axles,qprev);max_ik=max(max_ik,err)
        targets.append(qprev.copy())
    targets=np.array(targets)
    rows=[]; states=[]; bad_events=[]; saturated=0; limit_violation=0.
    start=time.monotonic()
    for step in range(int(round(duration/dt))):
        t=d.time
        q=np.array([np.interp(t,samples,targets[:,i]) for i in range(12)])
        speed=0.
        if name.startswith('roll'):
            speed=float(name.split('_')[1])/robot.radius*(smooth(t-2)-smooth(t-6))
        if name.startswith('turn'):
            yawrate=float(name.split('_')[1]); side=np.array([1,-1,1,-1])
            # Actuator order FR,FL,RR,RL; differential wheel velocity.
            speed=side*yawrate*.19/robot.radius*(smooth(t-2)-smooth(t-6))
        support=None
        if controller=='gravity':
            support=np.ones(4)
            if name.startswith('lift'):
                support[0]=1.-smooth((t-3.2)/.3)+smooth((t-6.5)/.3)
        saturated+=robot.torque(q,speed,gravity_support=support)
        mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d)
        s=robot.measure()
        limit_violation=max(limit_violation,s['joint_limit_violation'])
        if s['bad_contacts']:
            bad_events.append({'time':s['time'],'pairs':s['bad_contacts']})
        rows.append([s['time'],*s['base_pos'],s['tilt'],*s['velocity'],
                     *s['load'],s['slip'],s['penetration'],*s['torque'],
                     *s['qvel'],*s['axles'].ravel()])
        states.append(d.qpos.copy())
        if s['tilt']>np.deg2rad(50) or s['base_pos'][2]<.16:
            break
    a=np.array(rows); ts=a[:,0]; active=(ts>=3.5)&(ts<=5.5)
    final=robot.measure()
    torque=a[:,14:30]; loads=a[:,8:12]
    metrics=dict(name=name,variant=variant,controller=controller,timestep=dt,
        duration_s=float(d.time),wall_seconds=time.monotonic()-start,
        radius_m=robot.radius,ik_max_error_m=max_ik,
        reached_end=bool(d.time>=duration-dt/2),
        max_tilt_deg=float(np.rad2deg(a[:,4].max())),
        base_height_min_m=float(a[:,3].min()),base_height_max_m=float(a[:,3].max()),
        displacement_m=(a[-1,1:4]-a[0,1:4]).tolist(),
        steady_vx_m_s=float(a[active,5].mean()) if active.any() else None,
        slip_mean_m_s=float(a[active,12].mean()) if active.any() else None,
        slip_p95_m_s=float(np.quantile(a[active,12],.95)) if active.any() else None,
        max_penetration_m=float(a[:,13].max()),
        max_torque_nm=np.max(np.abs(torque),axis=0).tolist(),
        saturation_fraction=saturated/len(rows),
        max_joint_limit_violation_rad=limit_violation,
        bad_contact_steps=len(bad_events),
        final_load_n=final['load'].tolist(),
        final_tilt_deg=float(np.rad2deg(final['tilt'])),
        final_speed_m_s=float(np.linalg.norm(final['velocity'])),
        min_stance_load_n=float(np.min(loads[active,1:])) if active.any() else None,
        final_yaw_deg=float(np.rad2deg(np.arctan2(d.xmat[robot.base].reshape(3,3)[1,0],d.xmat[robot.base].reshape(3,3)[0,0]))),
        sim_warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number})
    if name.startswith('lift'):
        hold=(ts>=4.7)&(ts<=5.3)
        metrics['swing_unloaded_fraction']=float(np.mean(loads[hold,0]<1)) if hold.any() else 0.
        metrics['three_other_wheels_loaded_fraction']=float(np.mean(np.all(loads[hold,1:]>5,axis=1))) if hold.any() else 0.
        metrics['swing_axle_rise_m']=float(a[hold,48].max()-np.mean(a[(ts>1.5)&(ts<2),48])) if hold.any() else None
        metrics['landed_and_loaded']=bool(final['load'][0]>5)
    if out:
        p=Path(out);p.mkdir(parents=True,exist_ok=True)
        header=['t','x','y','z','tilt','vx','vy','vz']+[f'{l}_load' for l in LEGS]+['slip','penetration']
        header += [f'tau_{m.actuator(i).name}' for i in range(m.nu)]
        header += [f'qd_{m.actuator(i).name}' for i in range(m.nu)]
        header += [f'{l}_axle_{axis}' for l in LEGS for axis in 'xyz']
        with (p/f'{name}.csv').open('w') as f:
            w=csv.writer(f);w.writerow(header);w.writerows(rows)
        np.savez_compressed(p/f'{name}_states.npz',time=ts,qpos=np.asarray(states),reference_time=samples,reference_q=targets)
        (p/f'{name}.json').write_text(json.dumps(metrics,indent=2))
        (p/f'{name}_events.json').write_text(json.dumps(bad_events,indent=2))
        (p/'model.xml').write_text(xml)
    return metrics


def main():
    p=argparse.ArgumentParser();p.add_argument('--variant',default='single_collision',choices=['upstream','single_collision'])
    p.add_argument('--dt',type=float,default=.002)
    p.add_argument('--controller',default='pd',choices=['pd','gravity'])
    p.add_argument('--trials',nargs='+',default=['stand','roll_0.2','roll_-0.2','height_0.03','height_-0.03','pitch_8','lift_naive','lift_shift','turn_0.3'])
    p.add_argument('--out',type=Path,default=ROOT/'results')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    result=[]
    for name in args.trials:
        row=run_trial(name,args.variant,args.dt,args.out,args.controller);result.append(row)
        print(json.dumps(row),flush=True)
    (args.out/'summary.json').write_text(json.dumps(result,indent=2))


if __name__=='__main__':main()
