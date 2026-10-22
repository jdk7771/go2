"""Candidate search coupling descent body motion and three support wheels.

The landing target, timing and controller stay fixed. A translation bias
measured in earlier development trials is used only as a reachability probe;
this is not a calibrated uncertainty bound or an online state estimator.
"""
import itertools
import time
import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from model import smooth
from validate_geometry_sat import ConvexGeometry


BIAS=np.array([0.,-.045,-.01])


def amend(plan,t,delta,support_advance):
    body=np.array([np.interp(t,plan['time'],plan['body'][:,j]) for j in range(3)])
    feet=np.array([[np.interp(t,plan['time'],plan['centers'][:,i,j]) for j in range(3)] for i in range(4)])
    catch=smooth((t-12.5)/4.)
    factor=smooth((t-10.)/1.5)*(1.-catch)
    body+=np.asarray(delta)*factor
    feet[1:,0]+=support_advance*factor
    return body,feet


def refine(robot,plan,grid='expanded'):
    m=robot.m;started=time.monotonic()
    obstacle=m.geom('obstacle').id
    shapes=[ConvexGeometry(m,g) for g in range(m.ngeom) if m.geom_bodyid[g]>0 and m.geom_contype[g]>0]
    half=m.geom_size[obstacle];center=m.geom_pos[obstacle]
    candidates=[]
    times=[10.,10.5,11.,11.3,11.5,12.,12.5,13.,14.,15.,16.5]
    def evaluate(delta,advance,ts):
        maxik=0.;maxbiased=0.;clearance=.05;support_margin=1.
        solved=[];points=[];bodies=[]
        previous=plan['q'][np.argmin(abs(plan['time']-ts[0]))]
        reason=None;failed_at=None
        for t in ts:
            b,f=amend(plan,t,delta,advance)
            q,err=robot.ik(b,[1.,0.,0.,0.],f,initial=previous,centers=True)
            maxik=max(maxik,err);previous=q
            if err>.001:reason='nominal_ik';failed_at=t;break
            if 7.<=t<11.5:
                hull=ConvexHull(f[1:,:2]);com=robot.ikd.subtree_com[robot.base,:2]
                margin=float(np.min(-(hull.equations[:,:2]@com+hull.equations[:,2])))
                support_margin=min(support_margin,margin)
                if margin<.015:reason='support_margin';failed_at=t;break
            gaps=[shape.check(robot.ikd,center,half)[0] for shape in shapes]
            clearance=min(clearance,min(gaps))
            if min(gaps)<.005:reason='obstacle_clearance';failed_at=t;break
            # Test FL reach under the previously observed base translation.
            # Other joints stay at the nominal solution in this probe.
            bias=BIAS*(1.-smooth((t-12.5)/4.))
            robot.ikd.qpos[:3]=b+bias;mujoco.mj_forward(m,robot.ikd)
            probe=robot.centers(robot.ikd);probe[0]=f[0]
            _,err=robot.ik(b+bias,[1.,0.,0.,0.],probe,initial=q,centers=True)
            maxbiased=max(maxbiased,err)
            if err>.001:reason='biased_fl_ik';failed_at=t;break
            solved.append(q);points.append(f);bodies.append(b)
        record=dict(body_delta_m=list(delta),support_advance_m=advance,
            nominal_ik_max_m=maxik,biased_fl_ik_max_m=maxbiased,
            obstacle_separation_lower_bound_m=clearance,support_margin_min_m=support_margin,
            feasible=reason is None,rejection=reason,rejection_time_s=float(failed_at) if failed_at is not None else None,
            checked_samples=len(solved)+(reason is not None),
            cost=float(np.dot(delta,delta)+advance**2))
        return record,np.array(solved),np.array(points),np.array(bodies)
    support_advances={'initial':[0.,.04,.08],'expanded':[0.,.08,.16,.24]}[grid]
    for dx,dy,dz,advance in itertools.product([0.,.03,.06,.09],[0.,.02,.04,.06],[0.,.02,.04],support_advances):
        record,*_=evaluate([dx,dy,dz],advance,times)
        candidates.append(record)
    feasible=sorted([r for r in candidates if r['feasible']],key=lambda r:r['cost'])
    dense=[];selected=None;solution=None
    # Recheck the full 50 Hz reference before accepting any candidate.
    for candidate in feasible:
        record,q,feet,body=evaluate(candidate['body_delta_m'],candidate['support_advance_m'],plan['time'])
        dense.append(record)
        if record['feasible']:selected=record;solution=(q,feet,body);break
    plan=plan.copy()
    plan.update(coupled_candidates=candidates,coupled_dense_checks=dense,coupled_selected=selected,
        coupled_grid=grid,coupled_support_advance_candidates_m=support_advances,
        coupled_translation_probe_m=BIAS.tolist(),coupled_search_wall_seconds=time.monotonic()-started,
        coupled_search_failed=selected is None)
    if selected is not None:
        q,feet,body=solution
        qd=np.gradient(q,plan['time'],axis=0);pd=np.gradient(feet,plan['time'],axis=0)
        wheel=np.zeros((len(q),4))
        for a,i in enumerate([1,0,3,2]):wheel[:,a]=pd[:,i,0]/robot.radius-qd[:,3*a+1]-qd[:,3*a+2]
        plan.update(q=q,centers=feet,body=body,wheel_speed=wheel,
            max_ik_error_m=selected['nominal_ik_max_m'],planned_obstacle_collisions=[])
    return plan
