"""Known shape arrangements independently constrain the clearance checker."""
import mujoco
import numpy as np
from validate_geometry_sat import ConvexGeometry


def test_boxes():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="box" size=".1 .1 .1" mass="1"/></body></worldbody></mujoco>')
    d=mujoco.MjData(m);shape=ConvexGeometry(m,0)
    for center,expected in [(1.,.05),(.21,.01),(.2,0.),(.195,-.005)]:
        d.qpos[:3]=[center,0.,0.];d.qpos[3:7]=[1.,0.,0.,0.];mujoco.mj_kinematics(m,d)
        value,_=shape.check(d,np.zeros(3),np.full(3,.1))
        assert abs(value-expected)<1e-7,(center,value,expected)
    d.qpos[:3]=[.245,0,0];d.qpos[3:7]=[np.cos(np.pi/8),0,0,np.sin(np.pi/8)]
    mujoco.mj_kinematics(m,d);value,_=shape.check(d,np.zeros(3),np.full(3,.1))
    assert abs(value-(.245-.1-.1*np.sqrt(2)))<1e-7,value


def test_sphere():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body pos=".215 0 0"><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    d=mujoco.MjData(m);mujoco.mj_kinematics(m,d)
    gap,_=ConvexGeometry(m,0).check(d,np.zeros(3),np.full(3,.1))
    assert abs(gap-.015)<1e-8,gap


if __name__=='__main__':
    test_boxes();test_sphere();print('Known separated, touching, overlapping and rotated-shape checks passed.')
