"""M20 analytic wheel-center IK/FK and quasi-static support calculations."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from .m20_policy import DEFAULT_XML,DEFAULT_Q,JOINTS,_mujoco,sha

LEG_NAMES=('fl','fr','hl','hr')


def _cross2(o,a,b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def support_margin(point,contacts):
    n=len(contacts)
    if n<3:return -np.inf
    px=float(point[0]);py=float(point[1])
    if n<=4:
        pts=[(float(contacts[k][0]),float(contacts[k][1])) for k in range(n)]
        if n==3:
            p0,p1,p2=pts
            area2=_cross2(p0,p1,p2)
            if abs(area2)<=1e-12:return -np.inf
            hull=(p0,p1,p2) if area2>0 else (p0,p2,p1)
        else:
            pts.sort()
            lower=[]
            for p in pts:
                while len(lower)>=2 and _cross2(lower[-2],lower[-1],p)<=1e-15:lower.pop()
                lower.append(p)
            upper=[]
            for p in reversed(pts):
                while len(upper)>=2 and _cross2(upper[-2],upper[-1],p)<=1e-15:upper.pop()
                upper.append(p)
            hull=lower[:-1]+upper[:-1]
            if len(hull)<3:return -np.inf
        best=float('inf');m=len(hull)
        for k in range(m):
            ax,ay=hull[k];bx,by=hull[(k+1)%m]
            ex=bx-ax;ey=by-ay;elen=(ex*ex+ey*ey)**0.5
            if elen<=1e-15:return -np.inf
            d=(ex*(py-ay)-ey*(px-ax))/elen
            if d<best:best=d
        return best
    xy=np.asarray(contacts)[:,:2]
    try:hull=ConvexHull(xy)
    except Exception:return -np.inf
    return float(np.min(-(hull.equations[:,:2]@np.asarray(point)[:2]+hull.equations[:,2])))


def fit_support_plane(feet):
    """Exact least-squares plane z = a*x + b*y + c via centered 2x2 normal equations."""
    x=feet[:,0];y=feet[:,1];z=feet[:,2]
    mx=float(x.mean());my=float(y.mean());mz=float(z.mean())
    dx=x-mx;dy=y-my;dz=z-mz
    sxx=float(np.dot(dx,dx));sxy=float(np.dot(dx,dy));syy=float(np.dot(dy,dy))
    sxz=float(np.dot(dx,dz));syz=float(np.dot(dy,dz))
    det=sxx*syy-sxy*sxy
    if abs(det)>1e-12:
        inv=1./det
        a=(syy*sxz-sxy*syz)*inv
        b=(sxx*syz-sxy*sxz)*inv
        c=mz-a*mx-b*my
        return np.array([a,b,c],dtype=float)
    return np.linalg.lstsq(np.c_[feet[:,:2],np.ones(len(feet))],feet[:,2],rcond=None)[0]


def body_rotation_and_plane(feet,yaw):
    plane=fit_support_plane(feet);a,b=float(plane[0]),float(plane[1])
    inv_n=1./(a*a+b*b+1.)**0.5
    nx,ny,nz=-a*inv_n,-b*inv_n,inv_n
    cy,sy=float(np.cos(yaw)),float(np.sin(yaw))
    xz=a*cy+b*sy;inv_x=1./(cy*cy+sy*sy+xz*xz)**0.5
    xx,xy,xz=cy*inv_x,sy*inv_x,xz*inv_x
    yx=ny*xz-nz*xy;yy=nz*xx-nx*xz;yz=nx*xy-ny*xx
    return np.array([[xx,yx,nx],[xy,yy,ny],[xz,yz,nz]],dtype=float),plane


def body_rotation(feet,yaw):
    return body_rotation_and_plane(feet,yaw)[0]


class SurfaceRobot:
    def __init__(self):
        root=ET.parse(DEFAULT_XML).getroot();base=root.find(".//body[@name='base_link']")
        self.hips=np.array([np.fromstring(base.find(f"body[@name='{leg}_hipx']").get('pos'),sep=' ') for leg in LEG_NAMES])
        self.lateral=np.array([1,-1,1,-1])*(.0984+.059676);self.upper=self.lower=.25;self.radius=.09
        self.limits=np.array([np.fromstring(root.find(f".//joint[@name='{name}']").get('range'),sep=' ') for name in JOINTS[:12]]).reshape(4,3,2)
        self._lat_sq=self.lateral*self.lateral;self._ul_sq=self.upper*self.upper+self.lower*self.lower
        self._two_ul=2.*self.upper*self.lower;self._knee_sign=np.array([1.,1.,-1.,-1.])
        self._lim_lo=self.limits[:,:,0];self._lim_hi=self.limits[:,:,1]
        self.nominal_feet=self.fk(DEFAULT_Q[:12].reshape(4,3));self.nominal_feet[:,2]=0.
        self.half_body=np.array([.375,.115,.07]);self.nominal_height=.40
        mj=_mujoco();root.find('compiler').set('meshdir',str(DEFAULT_XML.parent/'meshes'))
        self.model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));self.data=mj.MjData(self.model);self.mj=mj
        self.joints=np.array([mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,n) for n in JOINTS]);self.qid=self.model.jnt_qposadr[self.joints]
        self.base=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_BODY,'base_link')
        # Precompute exact analytical subtree CoM parameters from MuJoCo model.
        m=self.model
        self._com_base_mc=m.body_mass[self.base]*m.body_ipos[self.base].copy()
        self._com_total_mass=float(m.body_mass[self.base])
        self._com_mc_hipx=np.zeros((4,3));self._com_mc_hipy=np.zeros((4,3))
        self._com_mc_knee=np.zeros((4,3));self._com_leg_pos_mc=np.zeros(3)
        for k,leg in enumerate(LEG_NAMES):
            ihx=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,f'{leg}_hipx')
            ihy=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,f'{leg}_hipy')
            ikn=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,f'{leg}_knee')
            iwh=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,f'{leg}_wheel')
            mhx,mhy,mkn,mwh=float(m.body_mass[ihx]),float(m.body_mass[ihy]),float(m.body_mass[ikn]),float(m.body_mass[iwh])
            mleg=mhx+mhy+mkn+mwh;self._com_total_mass+=mleg
            self._com_leg_pos_mc+=mleg*m.body_pos[ihx]
            self._com_mc_hipx[k]=mhx*m.body_ipos[ihx]
            self._com_mc_hipy[k]=mhy*m.body_ipos[ihy]+(mkn+mwh)*m.body_pos[ikn]
            self._com_mc_knee[k]=mkn*m.body_ipos[ikn]+mwh*(m.body_pos[iwh]+m.body_ipos[iwh])
        self._com_const_mc=self._com_base_mc+self._com_leg_pos_mc
        self._com_inv_mass=1./self._com_total_mass

    def fk(self,q):
        q=np.asarray(q);a=q[:,0];b=q[:,1];bc=b+q[:,2]
        sb=np.sin(b);cb=np.cos(b);sbc=np.sin(bc);cbc=np.cos(bc)
        sa=np.sin(a);ca=np.cos(a)
        x=self.upper*sb+self.lower*sbc
        z=-self.upper*cb-self.lower*cbc
        y=self.lateral*ca+z*sa;zz=-self.lateral*sa+z*ca
        out=self.hips.copy()
        out[:,0]+=x;out[:,1]+=y;out[:,2]+=zz
        return out

    def ik(self,body,rotation,contacts,normals=None):
        # Spherical wheel-envelope contact approximation for the initial planner.
        if normals is None:
            targets=contacts.copy();targets[:,2]+=self.radius
        else:
            targets=contacts+self.radius*normals
        local=(targets-body)@rotation-self.hips
        x=local[:,0];y=local[:,1];z=local[:,2]
        pre=-np.sqrt(np.maximum(y*y+z*z-self._lat_sq,1e-12))
        a=(np.arctan2(pre,self.lateral)-np.arctan2(z,y)+np.pi)%(2.*np.pi)-np.pi
        cosine=np.clip((x*x+pre*pre-self._ul_sq)/self._two_ul,-1.,1.)
        c=np.arccos(cosine)*self._knee_sign
        b=np.arctan2(x,-pre)-np.arctan2(self.lower*np.sin(c),self.upper+self.lower*np.cos(c))
        ideal=np.empty((4,3),dtype=float);ideal[:,0]=a;ideal[:,1]=b;ideal[:,2]=c
        q=np.clip(ideal,self._lim_lo,self._lim_hi)
        actual=self.fk(q)@rotation.T+body
        diff=actual-targets;residual=np.sqrt((diff*diff).sum(axis=1))
        margin=np.minimum(q-self._lim_lo,self._lim_hi-q)
        soft=float(((residual*100.)**2).mean()+((np.maximum(.12-margin,0.)/.12)**2).mean())
        return dict(q=q,wheels=actual,targets=targets,residual_m=residual,maximum_residual_m=float(residual.max()),
            soft_cost=soft,joint_violation_rad=float(np.abs(ideal-q).max()),joint_margin_rad=float(margin.min()))

    def com_fast(self,body,rotation,q):
        """Exact analytical 17-body CoM matching MuJoCo subtree_com to <1e-13."""
        a=q[:,0];b=q[:,1];bc=b+q[:,2]
        sa=np.sin(a);ca=np.cos(a);sb=np.sin(b);cb=np.cos(b);sbc=np.sin(bc);cbc=np.cos(bc)
        # Rotate hipy and knee+wheel first moments by R_y(-b) and R_y(-(b+c)) in hipx frame.
        hy=self._com_mc_hipy;kn=self._com_mc_knee
        rx=cb*hy[:,0]-sb*hy[:,2]+cbc*kn[:,0]-sbc*kn[:,2]+self._com_mc_hipx[:,0]
        ry=hy[:,1]+kn[:,1]+self._com_mc_hipx[:,1]
        rz=sb*hy[:,0]+cb*hy[:,2]+sbc*kn[:,0]+cbc*kn[:,2]+self._com_mc_hipx[:,2]
        # Rotate from hipx frame to base_link frame by R_x(-a), sum across 4 legs.
        local_mc=self._com_const_mc.copy()
        local_mc[0]+=float(rx.sum())
        local_mc[1]+=float((ca*ry+sa*rz).sum())
        local_mc[2]+=float((-sa*ry+ca*rz).sum())
        return body+rotation@(local_mc*self._com_inv_mass)

    def com(self,body,rotation,q):
        quaternion=Rotation.from_matrix(rotation).as_quat();self.data.qpos[:7]=np.r_[body,quaternion[3],quaternion[:3]]
        self.data.qpos[self.qid]=np.r_[np.asarray(q).ravel(),np.zeros(4)]
        self.mj.mj_kinematics(self.model,self.data);self.mj.mj_comPos(self.model,self.data)
        return self.data.subtree_com[self.base].copy()

    def metadata(self):
        return dict(model_xml=str(DEFAULT_XML),model_xml_sha256=sha(DEFAULT_XML),wheel_radius_m=self.radius,
            hip_positions_m=self.hips,link_lengths_m=[self.upper,self.lower],lateral_offsets_m=self.lateral,
            joint_limits_rad=self.limits,nominal_contact_offsets_m=self.nominal_feet,
            nominal_body_height_m=self.nominal_height,body_size_m=2*self.half_body,
            contact_model='Spherical wheel envelope offset along analytic surface normal; not a full cylinder contact model.',
            zmp='Quasi-static: a_xy=0, ZMP equals full-model center-of-mass horizontal projection; no dynamic execution claim.')
