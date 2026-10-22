"""Go2W E0 diagnostics. Dynamics use motor torques; IK uses a separate MjData."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares, lsq_linear
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
_relative='unitree_mujoco/unitree_robots/go2w/go2w.xml'
_cached=ROOT.parents[1]/'.deps'/ _relative
_legacy=ROOT.parents[2]/'go2_dependencies'/ _relative
SOURCE=Path(os.environ.get('GO2W_MODEL_XML',str(_cached if _cached.exists() else _legacy))).resolve()
LEGS = ('FL', 'FR', 'RL', 'RR')


def load_recorded_model(path):
    """Relocate published XML to the locally pinned mesh dependency."""
    root=ET.parse(path).getroot()
    root.find('compiler').set('meshdir',str(SOURCE.parent/'assets'))
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


def build_model(dt=.002, variant='upstream', obstacle=None,friction=None):
    if not SOURCE.exists():
        raise FileNotFoundError(f'Go2W model missing: {SOURCE}. Run bash bootstrap_model.sh or set GO2W_MODEL_XML.')
    root = ET.parse(SOURCE).getroot()
    root.find('compiler').set('meshdir', str(SOURCE.parent / 'assets'))
    root.find('option').set('timestep', str(dt))
    root.find('option').set('integrator', 'implicitfast')
    # Upstream places two identical active mesh geoms on each wheel. Keep a
    # baseline, and explicitly test making the first geom visual-only.
    for leg in LEGS:
        wheel = root.find(f'.//body[@name="{leg}_wheel_link"]')
        for i, geom in enumerate(wheel.findall('geom')):
            geom.set('name', f'{leg}_wheel_geom_{i}')
            if friction is not None and geom.get('class')=='foot':
                geom.set('friction',f'{float(friction)} .02 .01')
        if variant == 'single_collision':
            g = wheel.findall('geom')[0]
            g.set('contype', '0'); g.set('conaffinity', '0'); g.set('group', '2')
    ET.SubElement(root.find('worldbody'), 'geom', name='floor', type='plane',
                  size='8 8 .05', rgba='.22 .25 .3 1', friction='.8 .02 .01')
    ET.SubElement(root.find('worldbody'), 'light', pos='0 -2 3', dir='0 0 -1')
    if obstacle:
        ET.SubElement(root.find('worldbody'), 'geom', name='obstacle', type='box',
                      pos=' '.join(map(str, obstacle['pos'])),
                      size=' '.join(map(str, obstacle['size'])), rgba='.7 .3 .15 1')
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml), xml


class Robot:
    def __init__(self, model):
        self.m = model
        self.d = mujoco.MjData(model)
        self.ikd = mujoco.MjData(model)
        self.joints = model.actuator_trnid[:, 0].astype(int)
        self.qa = model.jnt_qposadr[self.joints]
        self.va = model.jnt_dofadr[self.joints]
        self.wheel_bodies = [model.body(x + '_wheel_link').id for x in LEGS]
        self.floor = model.geom('floor').id
        self.base = model.body('base_link').id
        self.nominal = np.array([0., .8, -1.5]*4+[0.]*4)
        self.d.qpos[self.qa] = self.nominal
        mujoco.mj_forward(model, self.d)
        bounds=[]; radii=[]; self.center_offsets=[]
        for b in self.wheel_bodies:
            g=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==b)
            mid=model.geom_dataid[g]
            verts=model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
            world=verts@self.d.geom_xmat[g].reshape(3,3).T+self.d.geom_xpos[g]
            local=(world-self.d.xpos[b])@self.d.xmat[b].reshape(3,3)
            # Wheel center lies on the hinge axis; use mesh width midpoint.
            self.center_offsets.append([0.,float((local[:,1].min()+local[:,1].max())/2),0.])
            bounds.append(world[:,2].min())
            radii.append(np.linalg.norm((world-self.d.xpos[b])[:,[0,2]],axis=1).max())
        self.radius=float(np.mean(radii))
        self.d.qpos[2] -= min(bounds)-.002
        mujoco.mj_forward(model,self.d)
        self.q0=self.d.qpos.copy()
        self.axles0=self.d.xpos[self.wheel_bodies].copy()
        self.centers0=self.centers(self.d)
        self.z0=float(self.d.qpos[2])

    def centers(self,d):
        return np.array([d.xpos[b]+d.xmat[b].reshape(3,3)@off
                         for b,off in zip(self.wheel_bodies,self.center_offsets)])

    def ik(self, base_pos, base_quat, axles, initial=None, centers=False):
        """Bounded 12-joint IK; joint bounds and contacts checked separately."""
        m,d=self.m,self.ikd
        d.qpos[:]=self.q0
        d.qpos[:3]=base_pos; d.qpos[3:7]=base_quat
        initial=self.nominal[:12] if initial is None else np.asarray(initial)
        lo=m.jnt_range[self.joints[:12],0]+1e-5
        hi=m.jnt_range[self.joints[:12],1]-1e-5
        def fun(q):
            d.qpos[self.qa[:12]]=q
            mujoco.mj_forward(m,d)
            points=self.centers(d) if centers else d.xpos[self.wheel_bodies]
            return (points-axles).ravel()
        def jac(q):
            d.qpos[self.qa[:12]]=q
            mujoco.mj_forward(m,d)
            rows=[]
            for k,b in enumerate(self.wheel_bodies):
                jp=np.zeros((3,m.nv))
                mujoco.mj_jac(m,d,jp,None,self.centers(d)[k] if centers else d.xpos[b],b)
                rows.append(jp[:,self.va[:12]])
            return np.vstack(rows)
        opt=least_squares(fun,np.clip(initial,lo,hi),jac=jac,bounds=(lo,hi),
                          max_nfev=60,ftol=1e-9,xtol=1e-9,gtol=1e-8)
        err=float(np.max(np.linalg.norm(fun(opt.x).reshape(4,3),axis=1)))
        return opt.x,err

    def torque(self,qtarget, wheel_speed=0., kp=80., kd=2., gravity_support=None,body_target=None,body_gain=25.):
        d,m=self.d,self.m
        raw=np.zeros(m.nu)
        raw[:12]=kp*(qtarget-d.qpos[self.qa[:12]])-kd*d.qvel[self.va[:12]]
        raw[12:]=3.*(np.asarray(wheel_speed)-d.qvel[self.va[12:]])
        if gravity_support is not None:
            # Quasi-static vertical contact-force allocation. This is a
            # diagnostic inverse-dynamics feedforward, not a full MPC/WBC.
            ids=[i for i,w in enumerate(gravity_support) if w>1e-4]
            pts=self.centers(d);pts[:,2]-=self.radius
            com=d.subtree_com[self.base]
            A=np.vstack((np.ones(len(ids)),(pts[ids,:2]-com[:2]).T/.2))
            weight=float(m.body_mass.sum()*9.81)
            upper=weight*np.asarray(gravity_support)[ids]
            f=lsq_linear(A,np.array([weight,0.,0.]),bounds=(np.zeros(len(ids)),upper),tol=1e-8).x
            forces=np.column_stack((np.zeros(len(ids)),np.zeros(len(ids)),f))
            if body_target is not None:
                force=np.array([0.,0.,weight])+m.body_mass.sum()*(
                    np.array([body_gain,body_gain,60.])*(np.asarray(body_target)-d.qpos[:3])
                    -np.array([8.*np.sqrt(body_gain/25),8.*np.sqrt(body_gain/25),12.])*d.qvel[:3])
                rot=Rotation.from_matrix(d.xmat[self.base].reshape(3,3)).as_rotvec()
                moment=-35.*rot-4.*d.qvel[3:6]
                blocks=[]
                for i in ids:
                    x,y,z=pts[i]-com
                    skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
                    blocks.append(np.vstack((np.eye(3),skew/.2)))
                mat=np.hstack(blocks)
                lower=np.tile([-weight*.8,-weight*.8,0.],len(ids))
                upper3=np.repeat(upper,3);upper3[0::3]=weight*.8;upper3[1::3]=weight*.8
                sol=lsq_linear(mat,np.r_[force,moment/.2],bounds=(lower,upper3),tol=1e-7).x.reshape(-1,3)
                # Conservative friction pyramid projection. The residual after
                # projection is not claimed to satisfy exact inverse dynamics.
                sol[:,:2]=np.clip(sol[:,:2],-.8/np.sqrt(2)*sol[:,2,None],.8/np.sqrt(2)*sol[:,2,None])
                forces=sol
            generalized=d.qfrc_bias.copy()
            for i,force in zip(ids,forces):
                jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,pts[i],self.wheel_bodies[i])
                generalized-=jp.T@force
            raw[:12]+=generalized[self.va[:12]]
        d.ctrl[:]=np.clip(raw,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
        return bool(np.any(np.abs(raw)>m.actuator_ctrlrange[:,1]+1e-9))

    def measure(self):
        m,d=self.m,self.d
        forces=np.zeros(4); weighted_slip=0.; total_load=0.; bad=[]
        max_pen=0.
        for k in range(d.ncon):
            c=d.contact[k]
            f=np.zeros(6); mujoco.mj_contactForce(m,d,k,f)
            if f[0]<1e-4: continue
            gs=(c.geom1,c.geom2); bs=[int(m.geom_bodyid[g]) for g in gs]
            max_pen=max(max_pen,-float(c.dist))
            wheel=next((i for i,b in enumerate(self.wheel_bodies) if b in bs),None)
            if self.floor in gs and wheel is not None:
                b=self.wheel_bodies[wheel]
                jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jp,None,c.pos,b)
                vel=jp@d.qvel; n=c.frame[:3]
                slip=np.linalg.norm(vel-n*np.dot(n,vel))
                forces[wheel]+=f[0]; weighted_slip+=f[0]*slip; total_load+=f[0]
            elif f[0]>1.:
                bad.append([m.geom(int(g)).name or f'geom{g}' for g in gs])
        R=d.xmat[self.base].reshape(3,3)
        tilt=float(np.arccos(np.clip(R[2,2],-1,1)))
        return dict(time=float(d.time),base_pos=d.qpos[:3].copy(),
                    base_quat=d.qpos[3:7].copy(),tilt=tilt,
                    velocity=d.qvel[:3].copy(),q=d.qpos[self.qa].copy(),
                    qvel=d.qvel[self.va].copy(),torque=d.ctrl.copy(),
                    load=forces,slip=weighted_slip/max(total_load,1e-12),
                    bad_contacts=bad,penetration=max_pen,
                    axles=d.xpos[self.wheel_bodies].copy(),
                    joint_limit_violation=float(max(0.,np.max(m.jnt_range[self.joints[:12],0]-d.qpos[self.qa[:12]]),
                                                       np.max(d.qpos[self.qa[:12]]-m.jnt_range[self.joints[:12],1]))))


def smooth(x):
    x=np.clip(x,0.,1.)
    return x*x*(3-2*x)


def quat_pitch(pitch):
    xyzw=Rotation.from_euler('y',pitch).as_quat()
    return xyzw[[3,0,1,2]]
