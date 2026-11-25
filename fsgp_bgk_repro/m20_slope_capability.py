"""Measure blind M20 actors on a finite flat/ramp/flat course in MuJoCo.

Run with OPENBLAS_NUM_THREADS=1 and the nero environment's Python. This reuses
the existing restricted checkpoint reader; it never trains or edits a policy.
The reported boundary belongs to this sim-to-sim port and the stated course.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import platform
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .m20_policy import (
    DEFAULT_CHECKPOINT, DEFAULT_Q, DEFAULT_XML, EXTERNAL, JOINTS, NumpyActor,
    WHEELS, sha,
)

LATEST_CHECKPOINT = EXTERNAL / (
    'outputs/official-rough-local/logs/rsl_rl/deeprobotics_m20_rough/'
    '2026-10-19_14-24-03_official_rough_resume_20k/model_19999.pt'
)
OUTPUT = Path(__file__).resolve().parent / 'outputs/visualization/m20-slope-20261120'
RAMP_START = .8
RAMP_RUN = 2.
HALF_WIDTH = 2.
LANE_HALF_WIDTH = .75
DT = .005
DECIMATION = 4


def surface_height(x, angle_deg):
    return np.tan(np.deg2rad(angle_deg)) * np.clip(np.asarray(x)-RAMP_START, 0, RAMP_RUN)


def make_model(angle_deg, friction=1., dt=DT, xml=DEFAULT_XML):
    """A solid convex wedge, with no entry step, followed by a level plateau."""
    if not 0 <= angle_deg < 70:
        raise ValueError('Slope must be in [0, 70) degrees')
    if friction <= 0:
        raise ValueError('Friction must be positive')
    root = ET.parse(xml).getroot()
    root.find('compiler').set('meshdir', str(Path(xml).parent/'meshes'))
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('timestep', str(dt))
    option.set('integrator', 'implicitfast')
    world = root.find('worldbody')
    # Set all colliding surfaces alike: MuJoCo's contact mixing can otherwise
    # silently retain the higher wheel coefficient during a friction sweep.
    for geom in root.findall('.//geom'):
        geom.set('friction', f'{friction} .01 .001')
    # Training disables robot self collisions. Keep robot/terrain contacts.
    collision_default = root.find(".//default[@class='collision']/geom")
    collision_default.set('contype', '2')
    collision_default.set('conaffinity', '1')
    world.find("geom[@name='floor']").set('conaffinity', '2')
    if angle_deg > 0:
        end = RAMP_START + RAMP_RUN
        height = float(surface_height(end, angle_deg))
        vertices = [(x, y, z) for x, top in [(RAMP_START, 0.), (end, height)]
                    for y in [-HALF_WIDTH, HALF_WIDTH] for z in [-.2, top]]
        ET.SubElement(root.find('asset'), 'mesh', name='evaluation_ramp_mesh',
                      vertex=' '.join(str(v) for p in vertices for v in p))
        ET.SubElement(world, 'geom', name='evaluation_ramp', type='mesh',
                      mesh='evaluation_ramp_mesh', friction=f'{friction} .01 .001',
                      rgba='.48 .62 .77 1', contype='1', conaffinity='2')
        ET.SubElement(world, 'geom', name='evaluation_plateau', type='box',
                      pos=f'{end+2} 0 {height/2}', size=f'2 {HALF_WIDTH} {height/2}',
                      friction=f'{friction} .01 .001', rgba='.55 .68 .78 1', contype='1', conaffinity='2')
    for joint in root.findall('.//joint'):
        if joint.get('name') in WHEELS:
            joint.set('armature', '.00243216')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def rollout(actor, angle_deg, seed=0, speed=.4, seconds=30., friction=1., dt=DT):
    started = time.perf_counter()
    decimation = int(round(.02/dt))
    if decimation < 1 or not np.isclose(decimation*dt, .02):
        raise ValueError('Physics timestep must divide the 0.02 s control period exactly')
    model = make_model(angle_deg, friction, dt)
    data = mujoco.MjData(model)
    jid = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS])
    qid, vid = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    aid = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
    wheel_bodies = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n[:-6]) for n in WHEELS])
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
    rng = np.random.default_rng(seed)
    yaw = rng.uniform(-.025, .025) if seed else 0.
    y = rng.uniform(-.015, .015) if seed else 0.
    x = -.65 + (rng.uniform(-.05, .05) if seed else 0.)
    initial = [x, y, .58, np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    data.qpos[:7] = initial
    data.qpos[qid] = DEFAULT_Q
    mujoco.mj_forward(model, data)
    previous = np.zeros(16)
    trace, actions, states = [], [], []
    scale = np.array([.125, .25, .25]*4)
    torque_limit = np.r_[np.full(12, 76.4), np.full(4, 21.6)]
    control_dt = dt*decimation
    hold = 0.
    success = False
    reason = 'timeout'
    base_contact_s = 0.
    for _ in range(int(round(seconds/control_dt))):
        rotation = data.xmat[base].reshape(3, 3)
        command = np.array([speed if data.time >= .5-1e-9 else 0., 0., 0.])
        position = data.qpos[qid]-DEFAULT_Q
        position[12:] = 0
        obs = np.r_[data.qvel[3:6]*.25, rotation.T@np.array([0, 0, -1.]),
                    command, position, data.qvel[vid]*.05, previous]
        action = actor(obs)
        target = DEFAULT_Q[:12]+scale*action[:12]
        wheel_target = action[12:]*5
        # Zero actuator latency, as in the previous local step-height adapter.
        for _ in range(decimation):
            tau = np.r_[80*(target-data.qpos[qid[:12]])-2*data.qvel[vid[:12]],
                        .6*(wheel_target-data.qvel[vid[12:]])]
            data.ctrl[aid] = np.clip(tau, -torque_limit, torque_limit)
            mujoco.mj_step(model, data)
        previous = action
        # Refresh derived geometry to exactly match the saved post-step qpos.
        mujoco.mj_forward(model, data)
        wheels = data.xpos[wheel_bodies].copy()
        rotation = data.xmat[base].reshape(3, 3)
        up = rotation[2, 2]
        ground = float(surface_height(data.qpos[0], angle_deg))
        on_ramp = RAMP_START < data.qpos[0] < RAMP_START+RAMP_RUN
        clearance = (data.qpos[2]-ground) * (np.cos(np.deg2rad(angle_deg)) if on_ramp else 1.)
        pitch_deg = np.rad2deg(np.arctan2(rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0])))
        base_contact = any(
            (model.geom_bodyid[c.geom1] == base and model.geom_bodyid[c.geom2] == 0)
            or (model.geom_bodyid[c.geom2] == base and model.geom_bodyid[c.geom1] == 0)
            for c in data.contact
        )
        base_contact_s = base_contact_s+control_dt if base_contact else 0.
        trace.append(np.r_[data.time, data.qpos[:7], wheels.ravel(), data.qvel[:3], up, clearance, pitch_deg])
        actions.append(action)
        states.append(data.qpos.copy())
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            reason = 'nonfinite'
            break
        if up < np.cos(np.deg2rad(75)) or clearance < .14 or base_contact_s >= .1-1e-9:
            reason = 'fallen_or_body_ground_contact'
            break
        if abs(data.qpos[1]) > LANE_HALF_WIDTH or np.max(np.abs(wheels[:, 1])) >= HALF_WIDTH-.1:
            reason = 'left_lane'
            break
        if data.qpos[0] < -1.5:
            reason = 'slid_backwards'
            break
        complete = (np.min(wheels[:, 0]) > RAMP_START+RAMP_RUN+.1
                    and data.qpos[0] > RAMP_START+RAMP_RUN+.55 and up > .85)
        hold = hold+control_dt if complete else 0.
        if hold >= .5-1e-9:
            success, reason = True, 'reached_plateau_and_held_0.5s'
            break
    trace = np.asarray(trace)
    rise = float(surface_height(RAMP_START+RAMP_RUN, angle_deg))
    return dict(
        angle_deg=float(angle_deg), ramp_rise_m=rise, seed=int(seed),
        command_speed_m_s=float(speed), friction=float(friction), physics_dt_s=dt,
        success=success, stop_reason=reason, initial_base_qpos=initial,
        simulated_s=float(data.time), final_x_m=float(data.qpos[0]),
        final_z_m=float(data.qpos[2]), final_y_m=float(data.qpos[1]),
        maximum_x_m=float(trace[:, 1].max()),
        maximum_terrain_height_under_base_m=float(surface_height(trace[:, 1].max(), angle_deg)),
        maximum_abs_lateral_m=float(np.max(np.abs(trace[:, 2]))),
        minimum_base_clearance_m=float(trace[:, 24].min()),
        final_pitch_deg=float(trace[-1, 25]),
        final_second_vx_m_s=float(trace[trace[:, 0] >= data.time-1, 20].mean()),
        wall_s=time.perf_counter()-started,
        trace=trace, actions=np.asarray(actions), qpos=np.asarray(states),
    )


def summarize(records):
    groups = {}
    for r in records:
        key = (r['checkpoint_iteration'], r['command_speed_m_s'], r['friction'],
               r['physics_dt_s'], r['angle_deg'])
        groups.setdefault(key, []).append(r)
    return [dict(checkpoint_iteration=k[0], speed_m_s=k[1], friction=k[2], physics_dt_s=k[3],
                 angle_deg=k[4], rise_m=v[0]['ramp_rise_m'], successes=sum(r['success'] for r in v),
                 trials=len(v), all_passed=all(r['success'] for r in v),
                 stop_reasons={reason: sum(r['stop_reason'] == reason for r in v)
                               for reason in sorted({r['stop_reason'] for r in v})},
                 mean_maximum_x_m=float(np.mean([r['maximum_x_m'] for r in v])))
            for k, v in sorted(groups.items())]


def run(args):
    if args.speed <= 0 or args.seconds <= .5:
        raise ValueError('Positive speed and duration > .5 s required')
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    records = json.loads(report_path.read_text())['trials'] if report_path.exists() else []
    checkpoints = []
    for checkpoint in args.checkpoints:
        actor = NumpyActor(checkpoint)
        digest = sha(checkpoint)
        metadata = dict(path=str(checkpoint), sha256=digest, iteration=actor.iteration)
        checkpoints.append(metadata)
        for angle in args.angles:
            for seed in args.seeds:
                # Keep existing measurements and refuse to silently replace them.
                if any(r['checkpoint_sha256'] == digest and r['angle_deg'] == angle
                       and r['seed'] == seed and r['command_speed_m_s'] == args.speed
                       and r['friction'] == args.friction and r['physics_dt_s'] == args.dt
                       and r['timeout_s'] == args.seconds for r in records):
                    continue
                trial = rollout(actor, angle, seed, args.speed, args.seconds, args.friction, args.dt)
                arrays = {name: trial.pop(name) for name in ['trace', 'actions', 'qpos']}
                label = f"i{actor.iteration}-a{angle:g}-v{args.speed:g}-mu{args.friction:g}-dt{args.dt:g}-s{seed}-t{args.seconds:g}"
                np.savez_compressed(args.output/f'{label}.npz', **arrays)
                trial.update(checkpoint_iteration=actor.iteration, checkpoint_sha256=digest,
                             checkpoint_path=str(checkpoint), trace_file=f'{label}.npz', timeout_s=args.seconds)
                records.append(trial)
                report = dict(
                    status='sim2sim_measured', updated_at=datetime.now().astimezone().isoformat(),
                    simulator='MuJoCo', simulator_version=mujoco.__version__, numpy_version=np.__version__,
                    python_version=platform.python_version(), model_xml=str(DEFAULT_XML),
                    model_xml_sha256=sha(DEFAULT_XML), evaluator_sha256=sha(__file__),
                    actor_adapter_sha256=sha(Path(__file__).with_name('m20_policy.py')),
                    course=dict(ramp_start_x_m=RAMP_START, ramp_horizontal_run_m=RAMP_RUN,
                                ramp_width_m=2*HALF_WIDTH, lane_half_width_m=LANE_HALF_WIDTH,
                                start_base_x_m=-.65, entry_step_m=0., plateau_end_x_m=6.8),
                    policy=dict(observations=57, actions=16, terrain_observation=False,
                                actor='deterministic ELU, 512/256/128, no empirical normalization',
                                physics_hz_tested=sorted({1/r['physics_dt_s'] for r in records}), control_hz=50.,
                                leg_kp=80., leg_kd=2., wheel_kd=.6,
                                actuator_delay_steps=0, observation_noise=False,
                                domain_randomization=False, robot_self_collisions=False, wheel_radius_m=.09),
                    success_definition='All four wheels beyond crest + 0.10 m, base beyond crest + 0.55 m, '
                                       'world vertical alignment > 0.85 for 0.5 s; no fall, sustained body contact, or lane exit.',
                    perturbation_definition='Seed 0 nominal; other seeds: yaw +/-0.025 rad, '
                                            'y +/-0.015 m, x +/-0.05 m; no stochastic policy/noise.',
                    trace_columns=['time_s', 'base_x', 'base_y', 'base_z', 'qw', 'qx', 'qy', 'qz']
                                  + [f'{leg}_{axis}' for leg in ['fl_wheel', 'fr_wheel', 'hl_wheel', 'hr_wheel']
                                     for axis in ['x', 'y', 'z']]
                                  + ['world_vx', 'world_vy', 'world_vz', 'base_up_dot_world_up',
                                     'base_surface_clearance_m', 'nose_up_pitch_deg'],
                    limitations=['MuJoCo transfer; not the original Isaac/PhysX evaluation or a physical robot limit.',
                                 'Nominal mass, geometry and deterministic control; no sensor noise or randomized material.',
                                 'Zero actuator delay; training configured random 0-1 physics-step delay.',
                                 'A finite 2 m horizontal ramp; rise depends on length, and is not a step-height limit.',
                                 'Success ratios cover only recorded initial-pose perturbations, not statistical field reliability.'],
                    trial_count=len(records), summaries=summarize(records), trials=records,
                )
                temporary = args.output/'report.json.tmp'
                temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
                temporary.replace(report_path)
                print(json.dumps({k: trial[k] for k in [
                    'checkpoint_iteration', 'angle_deg', 'command_speed_m_s', 'friction', 'seed',
                    'success', 'stop_reason', 'maximum_x_m', 'simulated_s']}, ensure_ascii=False), flush=True)
    shutil.copy2(__file__, args.output/'evaluator_source.py')
    shutil.copy2(Path(__file__).with_name('m20_policy.py'), args.output/'actor_adapter_source.py')
    return records


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoints', nargs='+', type=Path, default=[DEFAULT_CHECKPOINT, LATEST_CHECKPOINT])
    p.add_argument('--output', type=Path, default=OUTPUT)
    p.add_argument('--angles', nargs='+', type=float, default=list(range(0, 51, 5)))
    p.add_argument('--seeds', nargs='+', type=int, default=[0, 1, 2])
    p.add_argument('--speed', type=float, default=.4)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--friction', type=float, default=1.)
    p.add_argument('--dt', type=float, default=DT)
    run(p.parse_args())


if __name__ == '__main__':
    main()
