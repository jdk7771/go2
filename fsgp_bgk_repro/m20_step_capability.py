"""Small deterministic MuJoCo step-course test of the existing blind M20 actor."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from .m20_policy import DEFAULT_XML,DEFAULT_Q,DEFAULT_CHECKPOINT,JOINTS,WHEELS,NumpyActor,_mujoco,sha
from .scene_fit_catalog import WEB

OUT=WEB/'surface-planner/capability'
LATEST_CHECKPOINT=DEFAULT_CHECKPOINT.parent.parent/'2026-10-19_14-24-03_official_rough_resume_20k/model_19999.pt'


def make_model(height):
    mj=_mujoco();root=ET.parse(DEFAULT_XML).getroot()
    root.find('compiler').set('meshdir',str(DEFAULT_XML.parent/'meshes'))
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    option.set('timestep','.005');option.set('integrator','implicitfast')
    for g in root.findall('.//geom'):g.set('friction','1 .01 .001')
    collision=root.find(".//default[@class='collision']/geom");collision.set('contype','2');collision.set('conaffinity','1')
    world=root.find('worldbody');world.find("geom[@name='floor']").set('conaffinity','2')
    if height>0:ET.SubElement(world,'geom',name='step_course',type='box',pos=f'1.5 0 {height/2}',
        size=f'1.5 2 {height/2}',contype='1',conaffinity='2',friction='1 .01 .001')
    for joint in root.findall('.//joint'):
        if joint.get('name') in WHEELS:joint.set('armature','.00243216')
    return mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


def rollout(actor,height,seed=0,mode='blind',seconds=15.):
    started=time.perf_counter();mj=_mujoco();m=make_model(height);d=mj.MjData(m)
    joints=np.array([mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in JOINTS]);qid=m.jnt_qposadr[joints];vid=m.jnt_dofadr[joints]
    aid=np.array([mj.mj_name2id(m,mj.mjtObj.mjOBJ_ACTUATOR,n) for n in JOINTS])
    wheels=np.array([mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,n[:-6]) for n in WHEELS]);base=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'base_link')
    rng=np.random.default_rng(seed);yaw=0. if seed==0 else rng.uniform(-.025,.025)
    d.qpos[:7]=[-.9,0. if seed==0 else rng.uniform(-.015,.015),.58,np.cos(yaw/2),0,0,np.sin(yaw/2)]
    d.qpos[qid]=DEFAULT_Q;mj.mj_forward(m,d);previous=np.zeros(16);hold=0.;body_contact=0.;trace=[];success=False;reason='timeout'
    for _ in range(round(seconds/.02)):
        rotation=d.xmat[base].reshape(3,3);position=d.qpos[qid]-DEFAULT_Q;position[12:]=0.
        command=np.array([.35 if d.time>=.5 else 0.,0,0])
        obs=np.r_[d.qvel[3:6]*.25,rotation.T@np.array([0,0,-1.]),command,position,d.qvel[vid]*.05,previous]
        action=actor(obs);target=DEFAULT_Q[:12]+np.array([.125,.25,.25]*4)*action[:12]
        if mode=='fixed_legs':target=DEFAULT_Q[:12].copy()
        for _ in range(4):
            torque=np.r_[80*(target-d.qpos[qid[:12]])-2*d.qvel[vid[:12]],.6*(action[12:]*5-d.qvel[vid[12:]])]
            d.ctrl[aid]=np.clip(torque,-np.r_[np.full(12,76.4),np.full(4,21.6)],np.r_[np.full(12,76.4),np.full(4,21.6)])
            mj.mj_step(m,d)
        previous=action;mj.mj_forward(m,d);up=d.xmat[base].reshape(3,3)[2,2]
        wheel=d.xpos[wheels].copy();ground=height if d.qpos[0]>=0 else 0.
        trace.append(np.r_[d.time,d.qpos[:7],wheel.ravel(),up,d.qpos[2]-ground])
        touching=any((m.geom_bodyid[c.geom1]==base and m.geom_bodyid[c.geom2]==0) or
            (m.geom_bodyid[c.geom2]==base and m.geom_bodyid[c.geom1]==0) for c in d.contact)
        body_contact=body_contact+.02 if touching else 0.
        if not np.isfinite(d.qpos).all() or up<np.cos(np.deg2rad(75)) or d.qpos[2]-ground<.14 or body_contact>.10:
            reason='fall_or_body_contact';break
        if abs(d.qpos[1])>.75 or d.qpos[0]<-1.6:reason='left_course';break
        reached=np.min(wheel[:,0])>.10 and d.qpos[0]>.65 and up>.85
        hold=hold+.02 if reached else 0.
        if hold>=.5-1e-9:success=True;reason='all_wheels_up_and_held';break
    return dict(height_m=float(height),seed=seed,mode=mode,success=success,reason=reason,
        final_x_m=float(d.qpos[0]),simulated_s=float(d.time),wall_s=time.perf_counter()-started,trace=np.asarray(trace))


def run():
    parser=argparse.ArgumentParser();parser.add_argument('--heights',nargs='+',type=float,default=[0,.02,.04,.06,.08,.10,.12,.15,.18,.20,.25])
    parser.add_argument('--seeds',nargs='+',type=int,default=[0,1,2]);parser.add_argument('--modes',nargs='+',choices=['blind','fixed_legs'],default=['blind'])
    args=parser.parse_args();OUT.mkdir(parents=True,exist_ok=True);actor=NumpyActor(LATEST_CHECKPOINT)
    path=OUT/'report.json';old=json.loads(path.read_text()) if path.exists() else {};report=old;records=old.get('trials',[]);digest=sha(LATEST_CHECKPOINT)
    if old and old['checkpoint_sha256']!=digest:raise RuntimeError('Existing step tests use a different actor')
    for mode in args.modes:
        for height in args.heights:
            for seed in args.seeds:
                if any(r['height_m']==height and r['seed']==seed and r['mode']==mode for r in records):continue
                result=rollout(actor,height,seed,mode);trace=result.pop('trace');name=f'{mode}-h{height:.3f}-s{seed}.npz'
                np.savez_compressed(OUT/name,trace=trace);result['trace_file']=name;records.append(result)
                summary=[]
                for kind in sorted({r['mode'] for r in records}):
                    for h in sorted({r['height_m'] for r in records if r['mode']==kind}):
                        group=[r for r in records if r['mode']==kind and r['height_m']==h]
                        summary.append(dict(mode=kind,height_m=h,trials=len(group),passed=sum(r['success'] for r in group)))
                contiguous=0.
                for s in [s for s in summary if s['mode']=='blind']:
                    if s['trials']<3 or s['passed']!=s['trials']:break
                    contiguous=s['height_m']
                report=dict(checkpoint=str(LATEST_CHECKPOINT),checkpoint_sha256=digest,iteration=actor.iteration,
                    simulator='MuJoCo',simulator_version=_mujoco().__version__,model_xml_sha256=sha(DEFAULT_XML),
                    speed_m_s=.35,friction=1.,control_period_s=.02,physics_dt_s=.005,wheel_radius_m=.09,
                    conservative_step_limit_m=contiguous,trials=records,summaries=summary,
                    limit_definition='Largest contiguous tested blind step height with at least 3/3 passing initial-pose trials.',
                    limitation='Sim-to-sim finite forward step course, no claim of hardware or path-tracking capability.',
                    success='Four wheels beyond step +0.10 m and base +0.65 m, upright for 0.5 s; no fall or sustained torso contact.')
                path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(result,flush=True)
    return report


if __name__=='__main__':run()
