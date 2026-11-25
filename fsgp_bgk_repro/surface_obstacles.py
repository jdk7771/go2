"""Facing quadratic wall fit and bounded virtual-force body adjustment.

This is a local geometric correction of A* poses. Every accepted correction is
checked again by the same contacts, IK, body clearance and quasi-static ZMP.
"""
import time
import numpy as np
from scipy.spatial import ConvexHull


def features(yz):
    y,z=np.asarray(yz).T
    return np.c_[np.ones(len(y)),y,z,y*y,y*z,z*z]


def facing_wall(surface,pose):
    body=pose['body'];yaw=pose['yaw'];forward=np.array([np.cos(yaw),np.sin(yaw),0.]);left=np.array([-np.sin(yaw),np.cos(yaw),0.])
    ids=surface.rawxy.query_ball_point(body[:2],1.15)
    if not hasattr(surface,'vertical_observations'):
        # Raw local normals identify approximately vertical observations; a
        # narrow forward slice of an ordinary hill must not become a wall.
        _,nearest=surface.raw.query(surface.points,k=16)
        delta=surface.points[nearest]-surface.points[nearest].mean(axis=1,keepdims=True)
        covariance=np.einsum('nki,nkj->nij',delta,delta)
        _,vectors=np.linalg.eigh(covariance)
        surface.vertical_observations=np.abs(vectors[:,2,0])<.35
    ids=np.asarray(ids,dtype=int);ids=ids[surface.vertical_observations[ids]]
    p=surface.points[ids];q=p-body
    local=np.c_[q@forward,q@left,q[:,2]]
    keep=(local[:,0]>.12)&(local[:,0]<1.)&(np.abs(local[:,1])<.8)&(local[:,2]>-.18)&(local[:,2]<.35)
    local=local[keep]
    if len(local)<30:return None
    # Fit the nearest well-observed frontal slice. Horizontal ground points are
    # excluded by the height band; a vertical span and full rank are required.
    for group in np.unique(np.floor(local[:,0]/.025).astype(int)):
        points=local[np.floor(local[:,0]/.025).astype(int)==group]
        if len(points)<30 or np.ptp(points[:,2])<.15 or np.ptp(points[:,1])<.15:continue
        # Separate table legs must not become a filled frontal wall. Reject
        # slices with an unobserved lateral/vertical interval over 8 cm.
        if any(np.max(np.diff(np.unique(points[:,axis])),initial=0)>.08 for axis in (1,2)):continue
        yz=points[:,1:];origin=yz.mean(0);D=features(yz-origin)
        c,_,rank,_=np.linalg.lstsq(D,points[:,0],rcond=None)
        if rank<6:continue
        error=D@c-points[:,0];rmse=float(np.sqrt(np.mean(error**2)))
        if rmse>.02 or np.linalg.norm(c[1:3])>1.:continue
        try:hull=ConvexHull(yz-origin)
        except Exception:continue
        if hull.volume<.02:continue
        uv=yz[hull.vertices]-origin
        vertical_derivative=c[2]+c[4]*uv[:,0]+2*c[5]*uv[:,1]
        if np.max(np.abs(vertical_derivative))>.5:continue
        polygon=yz[hull.vertices];sample=np.c_[features(polygon-origin)@c,polygon]
        world=body+sample[:,0,None]*forward+sample[:,1,None]*left+np.c_[np.zeros((len(sample),2)),sample[:,2]]
        # Closest supported wall location to the body front, including wall ends.
        nearest=points[np.argmin(np.linalg.norm(points[:,1:],axis=1))]
        u,v=nearest[1:]-origin
        normal=-forward+(c[1]+2*c[3]*u+c[4]*v)*left+np.array([0.,0.,c[2]+c[4]*u+2*c[5]*v]);normal/=np.linalg.norm(normal)
        return dict(coefficients=c,origin_yz_m=origin,body_origin=body.copy(),forward=forward,left=left,
            polygon_yz_m=polygon,polygon_world=world,rmse_m=rmse,raw_points=len(points),
            nearest_world=body+nearest[0]*forward+nearest[1]*left+np.array([0.,0.,nearest[2]]),normal=normal,
            distance_forward_m=float(nearest[0]),lateral_nearest_m=float(nearest[1]))
    return None


def refine(planner,routes):
    begun=time.perf_counter();patches=[];applied=0;rejected=0;max_shift=0.
    for route in routes:
        for n,pose in enumerate(route['frames']):
            if pose.get('swing_leg') is not None:continue
            wall=facing_wall(planner.surface,pose)
            if wall is None:continue
            if len(patches)<24 and not any(np.linalg.norm(wall['nearest_world']-p['nearest_world'])<.12 for p in patches):patches.append(wall)
            # The facing virtual force acts only if the observed wall overlaps
            # the body laterally. It cannot invent a route through a wall.
            lateral=abs(wall['lateral_nearest_m']);gap=wall['distance_forward_m']-planner.robot.half_body[0]
            shift=np.clip(.50-gap,0,.035)*wall['normal'][:2]
            if lateral>.60 or np.linalg.norm(shift)<1e-6:continue
            trial=planner.fixed_pose(pose['body'][:2]+shift,pose['yaw'],pose['feet'],pose['normals'],pose['owners'],pose['support'])
            if trial is None:rejected+=1;continue
            # Sample the body displacement, retaining all original contact points.
            safe=True
            for t in (.25,.5,.75):
                if planner.fixed_pose(pose['body'][:2]+shift*t,pose['yaw'],pose['feet'],pose['normals'],pose['owners'],pose['support']) is None:safe=False;break
            if not safe:rejected+=1;continue
            for k in ('phase','duration_s','time_s','lift_flag','swing_leg','raw_distances_m'):
                if k in pose:trial[k]=pose[k]
            trial['virtual_force_shift_m']=shift;trial['body_before_force_m']=pose['body'];route['frames'][n]=trial
            applied+=1;max_shift=max(max_shift,float(np.linalg.norm(shift)))
    return dict(elapsed_s=time.perf_counter()-begun,fitted_wall_patches=patches,adjusted_frames=applied,
        rejected_adjustments=rejected,maximum_shift_m=max_shift,formula='forward = c0+c1*lateral+c2*z+c3*lateral²+c4*lateral*z+c5*z²',
        force='Repulsion along fitted wall normal, at most 3.5cm. Accept only after unchanged-contact IK, body clearance and ZMP checks.')
