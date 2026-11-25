"""M20 analytic wheel-center IK/FK and quasi-static support calculations."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from .m20_policy import DEFAULT_XML,DEFAULT_Q,JOINTS,_mujoco,sha

LEG_NAMES=('fl','fr','hl','hr')


def support_margin(point,contacts):
    if len(contacts)<3:return -np.inf
    xy=np.asarray(contacts)[:,:2]
    try:hull=ConvexHull(xy)
    except Exception:return -np.inf
    return float(np.min(-(hull.equations[:,:2]@np.asarray(point)[:2]+hull.equations[:,2])))


def body_rotation(feet,yaw):
    a,b,_=np.linalg.lstsq(np.c_[feet[:,:2],np.ones(len(feet))],feet[:,2],rcond=None)[0]
    n=np.array([-a,-b,1.]);n/=np.linalg.norm(n)
    x=np.array([np.cos(yaw),np.sin(yaw),a*np.cos(yaw)+b*np.sin(yaw)]);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(n,x),n))


class SurfaceRobot:
    def __init__(self):
        root=ET.parse(DEFAULT_XML).getroot();base=root.find(".//body[@name='base_link']")
        self.hips=np.array([np.fromstring(base.find(f"body[@name='{leg}_hipx']").get('pos'),sep=' ') for leg in LEG_NAMES])
        self.lateral=np.array([1,-1,1,-1])*(.0984+.059676);self.upper=self.lower=.25;self.radius=.09
        self.limits=np.array([np.fromstring(root.find(f".//joint[@name='{name}']").get('range'),sep=' ') for name in JOINTS[:12]]).reshape(4,3,2)
        self.nominal_feet=self.fk(DEFAULT_Q[:12].reshape(4,3));self.nominal_feet[:,2]=0.
        self.half_body=np.array([.375,.115,.07]);self.nominal_height=.40
        mj=_mujoco();root.find('compiler').set('meshdir',str(DEFAULT_XML.parent/'meshes'))
        self.model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));self.data=mj.MjData(self.model);self.mj=mj
        self.joints=np.array([mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,n) for n in JOINTS]);self.qid=self.model.jnt_qposadr[self.joints]
        self.base=mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_BODY,'base_link')

    def fk(self,q):
        q=np.asarray(q);a,b,c=q.T
        x=self.upper*np.sin(b)+self.lower*np.sin(b+c)
        z=-self.upper*np.cos(b)-self.lower*np.cos(b+c)
        y=self.lateral*np.cos(a)+z*np.sin(a);zz=-self.lateral*np.sin(a)+z*np.cos(a)
        return self.hips+np.c_[x,y,zz]

    def ik(self,body,rotation,contacts,normals=None):
        # Spherical wheel-envelope contact approximation for the initial planner.
        normals=np.tile([0.,0.,1.],(4,1)) if normals is None else normals
        targets=contacts+self.radius*normals;local=(targets-body)@rotation-self.hips
        x,y,z=local.T;pre=-np.sqrt(np.maximum(y*y+z*z-self.lateral*self.lateral,1e-12))
        a=(np.arctan2(pre,self.lateral)-np.arctan2(z,y)+np.pi)%(2*np.pi)-np.pi
        cosine=np.clip((x*x+pre*pre-self.upper**2-self.lower**2)/(2*self.upper*self.lower),-1.,1.)
        c=np.arccos(cosine)*np.array([1,1,-1,-1])
        b=np.arctan2(x,-pre)-np.arctan2(self.lower*np.sin(c),self.upper+self.lower*np.cos(c))
        ideal=np.c_[a,b,c];q=np.clip(ideal,self.limits[:,:,0],self.limits[:,:,1])
        actual=self.fk(q)@rotation.T+body;residual=np.linalg.norm(actual-targets,axis=1)
        margin=np.minimum(q-self.limits[:,:,0],self.limits[:,:,1]-q)
        soft=float(np.mean((residual/.01)**2)+np.mean((np.maximum(.12-margin,0)/.12)**2))
        return dict(q=q,wheels=actual,targets=targets,residual_m=residual,maximum_residual_m=float(residual.max()),
            soft_cost=soft,joint_violation_rad=float(np.max(np.abs(ideal-q))),joint_margin_rad=float(margin.min()))

    def com(self,body,rotation,q):
        quaternion=Rotation.from_matrix(rotation).as_quat();self.data.qpos[:7]=np.r_[body,quaternion[3],quaternion[:3]]
        self.data.qpos[self.qid]=np.r_[np.asarray(q).ravel(),np.zeros(4)];self.mj.mj_forward(self.model,self.data)
        return self.data.subtree_com[self.base].copy()

    def metadata(self):
        return dict(model_xml=str(DEFAULT_XML),model_xml_sha256=sha(DEFAULT_XML),wheel_radius_m=self.radius,
            hip_positions_m=self.hips,link_lengths_m=[self.upper,self.lower],lateral_offsets_m=self.lateral,
            joint_limits_rad=self.limits,nominal_contact_offsets_m=self.nominal_feet,
            nominal_body_height_m=self.nominal_height,body_size_m=2*self.half_body,
            contact_model='Spherical wheel envelope offset along analytic surface normal; not a full cylinder contact model.',
            zmp='Quasi-static: a_xy=0, ZMP equals full-model center-of-mass horizontal projection; no dynamic execution claim.')
