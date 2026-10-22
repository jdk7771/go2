"""Independent separating-axis checks on actual integrated poses.

Boxes and MuJoCo convex mesh hulls are represented directly. Cylinders are
enclosed by circumscribed 24-sided prisms. A positive returned separation is
a lower bound on Euclidean clearance, not its exact value. For the enclosing
prisms, an overlap is inconclusive for the original cylinder. Sphere-box
distance is analytic. Geometry playback does not execute or reset trials.
"""
import argparse
import csv
import itertools
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from model import load_recorded_model


def unique_axes(axes):
    axes=np.asarray(axes,dtype=float).reshape(-1,3)
    norms=np.linalg.norm(axes,axis=1)
    axes=axes[norms>1e-9]/norms[norms>1e-9,None]
    signs=np.where(np.take_along_axis(axes,np.argmax(abs(axes),axis=1)[:,None],axis=1)[:,0]<0,-1.,1.)
    axes*=signs[:,None]
    return np.unique(np.round(axes,8),axis=0)


class ConvexGeometry:
    def __init__(self,m,g):
        self.g=g;self.sphere=m.geom_type[g]==mujoco.mjtGeom.mjGEOM_SPHERE
        self.radius=float(m.geom_size[g,0])
        self.enclosed_cylinder=False
        if self.sphere:return
        if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:
            points=np.array(list(itertools.product([-1.,1.],repeat=3)))*m.geom_size[g]
        elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER:
            theta=np.arange(24)*2*np.pi/24
            radius=self.radius/np.cos(np.pi/24)
            points=np.array([[radius*np.cos(t),radius*np.sin(t),z]
                              for t in theta for z in [-m.geom_size[g,1],m.geom_size[g,1]]])
            self.enclosed_cylinder=True
        elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
            mesh=m.geom_dataid[g]
            points=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]].astype(float)
        else:
            raise ValueError(f'Unsupported collision geom {g}: {m.geom_type[g]}')
        hull=ConvexHull(points)
        self.vertices=points[hull.vertices]
        self.normals=unique_axes(hull.equations[:,:3])
        # Triangulation diagonals add valid axes; they cannot falsely certify
        # overlap, and true polyhedron edge cross-products are included.
        edges=np.concatenate([points[hull.simplices[:,(k+1)%3]]-points[hull.simplices[:,k]] for k in range(3)])
        self.edges=unique_axes(edges)

    def check(self,d,box_center,box_half,cutoff=.05):
        pos=d.geom_xpos[self.g];R=d.geom_xmat[self.g].reshape(3,3)
        if self.sphere:
            delta=np.abs(pos-box_center)-box_half
            gap=np.linalg.norm(np.maximum(delta,0))+min(float(np.max(delta)),0)-self.radius
            return min(float(gap),cutoff),float(pos[2]-self.radius)
        world=self.vertices@R.T+pos
        floor=float(world[:,2].min())
        gaps=np.maximum(box_center-box_half-world.max(0),world.min(0)-box_center-box_half)
        # Any separating axis proves disjointness. Most of this up-and-over
        # motion is certified by the world axes, avoiding dense mesh SAT.
        if gaps.max()>1e-8:return min(float(gaps.max()),cutoff),floor
        box_axes=np.eye(3)@R
        cross=np.cross(self.edges[:,None,:],box_axes[None,:,:]).reshape(-1,3)
        axes=unique_axes(np.vstack((box_axes,self.normals,cross)))
        projected=axes@self.vertices.T
        center_local=(box_center-pos)@R
        center=axes@center_local
        radius=np.abs(axes@box_axes.T)@box_half
        separation=np.maximum(projected.min(1)-(center+radius),(center-radius)-projected.max(1))
        return min(float(separation.max()),cutoff),floor


def validate(directory,method='searched_shift'):
    p=Path(directory);m=load_recorded_model(p/'model.xml');d=mujoco.MjData(m)
    states=np.load(p/f'{method}_states.npz')
    obstacle=m.geom('obstacle').id
    geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g]>0 and m.geom_contype[g]>0]
    shapes=[ConvexGeometry(m,g) for g in geoms]
    half=m.geom_size[obstacle];center=m.geom_pos[obstacle]
    rows=[];worst=(float('inf'),None,None);cutoff=.05
    for t,q in zip(states['time'],states['qpos']):
        d.qpos[:]=q;mujoco.mj_kinematics(m,d)
        distances=[];ground=[]
        for shape in shapes:
            separation,z=shape.check(d,center,half,cutoff)
            distances.append(separation)
            if 'wheel' not in m.body(int(m.geom_bodyid[shape.g])).name:ground.append(z)
        j=int(np.argmin(distances))
        if distances[j]<worst[0]:worst=(distances[j],float(t),m.geom(geoms[j]).name or f'geom{geoms[j]}')
        rows.append([float(t),min(distances),min(ground)])
    a=np.array(rows)
    result=dict(scope='Independent SAT lower bounds over all saved physical poses; no inter-step guarantee',
        geometry='Exact boxes/convex mesh hulls; cylinders enclosed by circumscribed 24-gon prisms; analytic spheres',
        physical_state_count=len(rows),collision_geometries=len(geoms),distance_cutoff_m=cutoff,
        min_robot_obstacle_separation_lower_bound_m=float(a[:,1].min()),
        min_nonwheel_floor_clearance_lower_bound_m=float(a[:,2].min()),
        potential_robot_obstacle_overlap_states=int(np.sum(a[:,1]<-1e-6)),
        potential_nonwheel_floor_overlap_states=int(np.sum(a[:,2]<-1e-6)),
        closest_obstacle_geom=worst[2],closest_obstacle_time_s=worst[1],
        geometry_validation_passed=bool(np.all(a[:,1:]>=-1e-6)))
    with (p/f'{method}_sat_clearances.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['t','robot_obstacle_separation_lower_bound_m','nonwheel_floor_clearance_lower_bound_m']);w.writerows(rows)
    (p/f'{method}_sat_validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--method',default='searched_shift')
    a=p.parse_args();validate(a.directory,a.method)
