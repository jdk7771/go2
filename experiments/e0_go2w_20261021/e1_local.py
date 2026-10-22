"""Conservative E1 diagnostic: rolling approach, one front-wheel crossing,
load recovery, catch-up of the other wheels, and resumed rolling.

No learned policy and no claim of full WBC/MPC. Offline IK references are
executed using the same torque-limited PD + quasi-static gravity controller.
Every physical step is integrated; only initialization sets simulated state.
"""
import argparse
import csv
import json
import time
import itertools
import hashlib
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from model import ROOT, LEGS, Robot, build_model, smooth


def search_shift(robot,origin,feet,landing,swing_height,body_raise=0.):
    """Small explicit candidate search; landing region and obstacle are fixed."""
    hull=ConvexHull(feet[1:,:2]);records=[]
    for dx,dy,dz in itertools.product([0.,.03,.06,.09],[-.04,-.07,-.10,-.13],[-.03,-.06,-.09]):
        shift=np.array([dx,dy,dz]);maxerr=0.;minmargin=1.;bad=0
        for fraction,rise in [(0.,swing_height),(.5,swing_height),(1.,swing_height),(1.,0.)]:
            target=feet.copy();target[0,0]+=(landing-feet[0,0])*fraction;target[0,2]+=rise
            body=origin+shift+np.array([0.,0.,body_raise if rise>0 else 0.])
            q,err=robot.ik(body,np.array([1.,0.,0.,0.]),target,centers=True)
            maxerr=max(maxerr,err)
            com=robot.ikd.subtree_com[robot.base,:2]
            margin=float(np.min(-(hull.equations[:,:2]@com+hull.equations[:,2])))
            minmargin=min(minmargin,margin)
            for k in range(robot.ikd.ncon):
                c=robot.ikd.contact[k]
                if robot.m.geom('obstacle').id in (c.geom1,c.geom2) and c.dist<0:bad+=1
        feasible=maxerr<=.001 and minmargin>=.015 and bad==0
        records.append(dict(shift_m=shift.tolist(),ik_max_error_m=maxerr,
                            support_margin_min_m=minmargin,obstacle_contacts=bad,
                            feasible=feasible,cost=float(np.linalg.norm(shift))))
    feasible=[r for r in records if r['feasible']]
    chosen=min(feasible,key=lambda r:r['cost']) if feasible else None
    return (np.array(chosen['shift_m']) if chosen else None),records


def crossing_plan(robot,method,obstacle,swing_clearance=.07,body_raise=0.,relax_landing=False):
    d=robot.d
    origin=d.qpos[:3].copy();quat=np.array([1.,0.,0.,0.])
    feet=robot.centers(d).copy()
    landing=obstacle['pos'][0]+obstacle['size'][0]+robot.radius+.025
    advance=landing-feet[0,0]
    ts=np.arange(4.5,16.5001,.02)
    qs=[];points=[];body=[];errors=[];collisions=[]
    previous=d.qpos[robot.qa[:12]].copy()
    started=time.monotonic()
    candidates=[]
    shift=np.array([.02,-.08,-.03]) if method=='shift' else np.zeros(3)
    if method=='searched_shift':
        shift,candidates=search_shift(robot,origin,feet,landing,obstacle['size'][2]*2+swing_clearance,body_raise)
        if shift is None:
            shift=np.zeros(3)
    for t in ts:
        b=origin+shift*smooth((t-4.5)/2)
        b[2]+=body_raise*(smooth((t-6.5)/1.5)-smooth((t-10)/1.5))
        f=feet.copy()
        f[0,2]+=(obstacle['size'][2]*2+swing_clearance)*(smooth(t-7)-smooth((t-10)/1.5))
        f[0,0]+=advance*smooth((t-8)/2)
        catch=smooth((t-12.5)/4)
        b+=np.array([advance,0.,0.])*catch-shift*catch
        if relax_landing:
            # The large lateral shift needed at peak lift need not persist
            # through touchdown. Return toward the low-obstacle support pose.
            b[1]+=(max(shift[1],-.07)-shift[1])*smooth((t-10.)/1.5)*(1.-catch)
        f[1:,0]+=advance*catch
        q,e=robot.ik(b,quat,f,previous,centers=True);previous=q
        qs.append(q);points.append(f);body.append(b);errors.append(e)
        # Contact with the obstacle by any link is forbidden in this crossing.
        m,kd=robot.m,robot.ikd
        for k in range(kd.ncon):
            c=kd.contact[k]
            if c.dist<0 and m.geom('obstacle').id in (c.geom1,c.geom2):
                collisions.append(dict(time=float(t),geoms=[int(c.geom1),int(c.geom2)],distance=float(c.dist)))
    qs=np.array(qs);points=np.array(points)
    qd=np.gradient(qs,ts,axis=0)
    pd=np.gradient(points,ts,axis=0)
    wheel=np.zeros((len(ts),4))
    # Motor order FR,FL,RR,RL; position references use FL,FR,RL,RR.
    for a,i in enumerate([1,0,3,2]):
        wheel[:,a]=pd[:,i,0]/robot.radius-qd[:,3*a+1]-qd[:,3*a+2]
    return dict(time=ts,q=qs,wheel_speed=wheel,centers=points,body=np.array(body),
                max_ik_error_m=max(errors),planned_obstacle_collisions=collisions,
                planning_wall_seconds=time.monotonic()-started,
                landing_x_m=landing,advance_m=advance,chosen_body_shift_m=shift.tolist(),
                candidates=candidates)


def run(method,dt,out,feedback=False,height=.043,wheel_feedback=False,body_gain=25.,swing_clearance=.07,body_raise=0.,friction=.8,push_force=0.,contact_points=False,retarget_swing=False,relax_landing=False,coupled_landing=False,coupled_grid='expanded'):
    # Identical world geometry for both methods, fixed before execution.
    obstacle={'pos':[.422,.193,height/2],'size':[.025,.065,height/2]}
    m,xml=build_model(dt,variant='single_collision',obstacle=obstacle,friction=friction)
    robot=Robot(m);d=robot.d
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    plan=None;rows=[];states=[];velocities=[];controls=[];external=[];events=[];reason='duration_complete';saturation=0;push_impulse=0.
    max_joint_limit=0.;max_bad=0;lift_seen=False;land_seen=False;resume_allowed=False
    takeoff_gate=None;landing_error=None;resume_checked=False
    qref=robot.nominal[:12].copy()
    retarget_q=None;retarget_next=7.;retarget_errors=[];retarget_times=[];retarget_wall=[];executed_references=[]
    for step in range(int(round(20/dt))):
        t=d.time;wheel=0.;support=np.ones(4);phase='approach';body_target=None
        d.xfrc_applied[:]=0.
        if 9.<=t<9.15:
            d.xfrc_applied[robot.base,1]=push_force
            push_impulse+=push_force*dt
        if t<4.5:
            wheel=.08/robot.radius*(smooth((t-2)/.5)-smooth((t-3.5)/.5))
        else:
            if plan is None:
                plan=crossing_plan(robot,method,obstacle,swing_clearance,body_raise,relax_landing)
                if coupled_landing:
                    from coupled_plan import refine
                    plan=refine(robot,plan,coupled_grid)
                if plan['max_ik_error_m']>.001 or plan['planned_obstacle_collisions'] or plan.get('coupled_search_failed',False):
                    reason='planning_rejected';events.append(dict(time=t,event=reason));break
            qref=np.array([np.interp(t,plan['time'],plan['q'][:,i]) for i in range(12)])
            wheel=np.array([np.interp(t,plan['time'],plan['wheel_speed'][:,i]) for i in range(4)])
            if feedback and t<16.5:
                body_target=np.array([np.interp(t,plan['time'],plan['body'][:,i]) for i in range(3)])
            phase='shift' if t<6.5 else 'unload' if t<7 else 'raise' if t<8 else 'cross' if t<10 else 'lower' if t<11.5 else 'reload' if t<12.5 else 'catchup' if t<16.5 else 'rolling'
            support[0]=1.-smooth((t-6.5)/.5)+smooth((t-11.5)/.5)
            if retarget_swing and 7.<=t<16.5:
                # Recompute FL targets at 50 Hz in the measured base frame.
                # Other legs keep the original controller. During reload and
                # catch-up, preserve measured height while correcting XY.
                if t>=retarget_next-1e-9:
                    target=robot.centers(d)
                    target[0]=[np.interp(t,plan['time'],plan['centers'][:,0,j]) for j in range(3)]
                    if t>=11.5:target[0,2]=robot.centers(d)[0,2]
                    solve_started=time.monotonic()
                    solved,error=robot.ik(d.qpos[:3],d.qpos[3:7],target,
                        initial=d.qpos[robot.qa[:12]],centers=True)
                    retarget_errors.append(error)
                    retarget_times.append(t);retarget_wall.append(time.monotonic()-solve_started)
                    retarget_q=solved[3:6].copy()
                    retarget_next=t+.02
                blend=smooth((t-7.)/.3)*(1.-smooth((t-16.)/.5))
                qref[3:6]=(1.-blend)*qref[3:6]+blend*retarget_q
            if t>=8 and takeoff_gate is None:
                s=robot.measure()
                takeoff_gate=bool(s['load'][0]<1 and np.all(s['load'][1:]>5)
                    and robot.centers(d)[0,2]>height+robot.radius+.02)
                events.append(dict(time=t,event='takeoff_gate',passed=takeoff_gate))
                if not takeoff_gate:reason='takeoff_gate_failed';break
            if t>=12.5 and not resume_checked:
                resume_checked=True
                s=robot.measure()
                target=plan['centers'][np.argmin(abs(plan['time']-12.5)),0,:2]
                landing_error=float(np.linalg.norm(robot.centers(d)[0,:2]-target))
                resume_allowed=lift_seen and np.all(s['load']>5) and s['tilt']<np.deg2rad(8) and landing_error<=.03
                events.append(dict(time=t,event='resume_gate',passed=bool(resume_allowed),load=s['load'].tolist()))
                if not resume_allowed:reason='landing_support_gate_failed';break
            if wheel_feedback and t<16.5:
                desired_centers=np.array([[np.interp(t,plan['time'],plan['centers'][:,i,j]) for j in range(3)] for i in range(4)])
                # Track rolling-contact trajectories: command wheel spin using
                # actual leg angular velocity, plus world-center x feedback.
                deriv=np.gradient(plan['centers'][:,:,0],plan['time'],axis=0)
                actual_centers=robot.centers(d)
                for actuator,leg in enumerate([1,0,3,2]):
                    if support[leg]>.5:
                        vx=np.interp(t,plan['time'],deriv[:,leg])+4.*(desired_centers[leg,0]-actual_centers[leg,0])
                        jid=robot.joints[12+actuator];vel=np.zeros(6)
                        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,robot.wheel_bodies[leg],vel,0)
                        joint_motion=float(np.dot(vel[:3],d.xaxis[jid])-d.qvel[robot.va[12+actuator]])
                        wheel[actuator]=vx/robot.radius-joint_motion
            if t>=16.5:
                wheel=.08/robot.radius*(smooth((t-16.5)/.5)-smooth((t-18)/.5))
        saturation+=robot.torque(qref,wheel,gravity_support=support,body_target=body_target,body_gain=body_gain,contact_points=contact_points)
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
        s=robot.measure();max_joint_limit=max(max_joint_limit,s['joint_limit_violation'])
        if 7<=t<11.5 and s['load'][0]<1 and not lift_seen:
            lift_seen=True;events.append(dict(time=s['time'],event='actual_liftoff'))
        if t>=11.5 and lift_seen and s['load'][0]>5 and not land_seen:
            land_seen=True;events.append(dict(time=s['time'],event='actual_reloaded'))
        if s['bad_contacts']:
            max_bad+=1;events.append(dict(time=s['time'],event='forbidden_contact',pairs=s['bad_contacts']))
        rows.append([s['time'],phase,*s['base_pos'],s['tilt'],*s['velocity'],*s['load'],s['slip'],
                     *robot.centers(d).ravel(),*s['torque']])
        states.append(d.qpos.copy())
        velocities.append(d.qvel.copy());controls.append(d.ctrl.copy())
        external.append(d.xfrc_applied[robot.base].copy())
        executed_references.append(qref.copy())
        if s['bad_contacts']:
            reason='forbidden_contact';break
        if s['tilt']>np.deg2rad(20) or s['base_pos'][2]<.2:
            reason='unstable';break
    # Retain every attempt, including planning rejection and gate failure.
    header=['t','phase','x','y','z','tilt','vx','vy','vz']+[f'{l}_load' for l in LEGS]+['slip']
    header += [f'{l}_center_{axis}' for l in LEGS for axis in 'xyz']+[f'tau_{m.actuator(i).name}' for i in range(16)]
    with (out/f'{method}.csv').open('w') as f:
        w=csv.writer(f);w.writerow(header);w.writerows(rows)
    np.savez_compressed(out/f'{method}_states.npz',time=[r[0] for r in rows],qpos=states,qvel=velocities,ctrl=controls,external_wrench=external,joint_reference=executed_references,
        retarget_time=retarget_times,retarget_error=retarget_errors,retarget_wall_seconds=retarget_wall)
    (out/'model.xml').write_text(xml)
    (out/f'{method}_events.json').write_text(json.dumps(events,indent=2))
    resumed=[r for r in rows if 17.2<=r[0]<=17.9]
    crossing=[r for r in rows if 8.2<=r[0]<=9.8]
    result=dict(method=method,dt=dt,body_feedback=feedback,wheel_feedback=wheel_feedback,body_gain=body_gain,swing_clearance_m=swing_clearance,body_raise_m=body_raise,terminal_reason=reason,duration_s=float(d.time),
                initial_obstacle=obstacle,actual_liftoff=lift_seen,actual_reload=land_seen,contact_point_feedback=contact_points,
                retarget_swing=retarget_swing,retarget_max_ik_error_m=max(retarget_errors) if retarget_errors else None,
                relax_landing=relax_landing,
                coupled_landing=coupled_landing,
                retarget_update_count=len(retarget_errors),
                retarget_ik_over_1mm_fraction=float(np.mean(np.array(retarget_errors)>.001)) if retarget_errors else None,
                retarget_wall_seconds_p95=float(np.quantile(retarget_wall,.95)) if retarget_wall else None,
                execution_code_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['model.py','e1_local.py','coupled_plan.py']},
                resume_gate_passed=bool(resume_allowed),
                landing_error_xy_m=landing_error,takeoff_gate_passed=takeoff_gate,
                actual_friction=friction,controller_nominal_friction=.8,push_force_y_n=push_force,push_impulse_y_ns=push_impulse,
                max_tilt_deg=float(np.rad2deg(max(r[5] for r in rows))),
                max_joint_limit_violation_rad=max_joint_limit,
                saturation_fraction=saturation/max(len(rows),1),forbidden_contact_steps=max_bad,
                resumed_mean_vx_m_s=float(np.mean([r[6] for r in resumed])) if resumed else None,
                resumed_four_wheel_load_fraction=float(np.mean([all(x>5 for x in r[9:13]) for r in resumed])) if resumed else None,
                crossing_three_support_fraction=float(np.mean([all(x>5 for x in r[10:13]) for r in crossing])) if crossing else None,
                crossing_fl_unloaded_fraction=float(np.mean([r[9]<1 for r in crossing])) if crossing else None,
                slip_p95_m_s=float(np.quantile([r[13] for r in rows],.95)),
                final_position=d.qpos[:3].tolist())
    if plan:
        result.update({k:v for k,v in plan.items() if k not in ('time','q','wheel_speed','centers','body')})
        np.savez_compressed(out/f'{method}_plan.npz',**{k:plan[k] for k in ('time','q','wheel_speed','centers','body')})
    result['diagnostic_success']=bool(reason=='duration_complete' and lift_seen and land_seen
        and resume_allowed and result['resumed_mean_vx_m_s']>.04
        and result['resumed_four_wheel_load_fraction']>.95
        and result['crossing_three_support_fraction']>.95
        and result['crossing_fl_unloaded_fraction']>.95 and max_bad==0
        and max_joint_limit<.005 and result['saturation_fraction']<.01)
    (out/f'{method}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--methods',nargs='+',default=['fixed','shift'])
    p.add_argument('--dt',type=float,default=.002);p.add_argument('--out',type=Path,default=ROOT/'e1_pilot')
    p.add_argument('--feedback',action='store_true');p.add_argument('--height',type=float,default=.043)
    p.add_argument('--wheel-feedback',action='store_true')
    p.add_argument('--body-gain',type=float,default=25.);p.add_argument('--swing-clearance',type=float,default=.07)
    p.add_argument('--body-raise',type=float,default=0.)
    p.add_argument('--friction',type=float,default=.8);p.add_argument('--push-force',type=float,default=0.)
    p.add_argument('--contact-points',action='store_true')
    p.add_argument('--retarget-swing',action='store_true')
    p.add_argument('--relax-landing',action='store_true')
    p.add_argument('--coupled-landing',action='store_true')
    p.add_argument('--coupled-grid',choices=['initial','expanded'],default='expanded')
    a=p.parse_args()
    results=[run(method,a.dt,a.out,a.feedback,a.height,a.wheel_feedback,a.body_gain,a.swing_clearance,a.body_raise,a.friction,a.push_force,a.contact_points,a.retarget_swing,a.relax_landing,a.coupled_landing,a.coupled_grid) for method in a.methods]
    (a.out/'summary.json').write_text(json.dumps(results,indent=2))
