#!/usr/bin/env python3
"""Render an exported M20 trajectory with PyBullet's CPU TinyRenderer.

The robot poses originate from an Isaac Sim policy rollout.  PyBullet is used
only to render M20's supplied URDF/STL geometry, so this utility neither needs
Vulkan nor the server's RTX scene renderer.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import pybullet as p


parser = argparse.ArgumentParser(description="CPU-render an exported M20 trajectory.")
parser.add_argument("--trajectory", type=Path, required=True)
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--label", type=str, default="M20 policy rollout")
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=540)
parser.add_argument("--frame_stride", type=int, default=2, help="Keep every N trajectory samples.")
parser.add_argument(
    "--camera",
    choices=("follow", "fixed"),
    default="follow",
    help="Use a robot-following camera or a fixed camera centered on the whole rollout.",
)
args = parser.parse_args()

if args.frame_stride < 1:
    raise ValueError("--frame_stride must be positive")

data = np.load(args.trajectory, allow_pickle=False)
root_pos = data["root_pos_w"].astype(np.float64)
root_quat = data["root_quat_w"].astype(np.float64)
joint_pos = data["joint_pos"].astype(np.float64)
joint_names = [str(name) for name in data["joint_names"]]
dt = float(data["step_dt"])
command_velocity = data["command_velocity"].astype(np.float64) if "command_velocity" in data.files else None
root_lin_vel_b = data["root_lin_vel_b"].astype(np.float64) if "root_lin_vel_b" in data.files else None
terrain_hits_w = data["terrain_hits_w"].astype(np.float64) if "terrain_hits_w" in data.files else None
if not (len(root_pos) == len(root_quat) == len(joint_pos)):
    raise ValueError("Trajectory arrays have mismatched lengths.")

args.output.parent.mkdir(parents=True, exist_ok=True)
fps = 1.0 / (dt * args.frame_stride)
writer = cv2.VideoWriter(
    str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (args.width, args.height)
)
if not writer.isOpened():
    raise RuntimeError("OpenCV could not open the requested MP4 output.")

client = p.connect(p.DIRECT)
try:
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setAdditionalSearchPath(str(args.urdf.parent))
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    plane = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=plane_shape)
    p.changeVisualShape(plane, -1, rgbaColor=[0.32, 0.34, 0.37, 1.0])
    terrain_source = "flat placeholder"
    if terrain_hits_w is not None:
        # Reconstruct a static local mesh from the exact height-scanner hits
        # observed during the Isaac Sim rollout.  This is visual-only: robot
        # physics was already simulated before these poses were exported.
        hits = terrain_hits_w.reshape(-1, 3)
        hits = hits[np.isfinite(hits).all(axis=1)]
        hits[:, :2] -= root_pos[0, :2]
        resolution = 0.1
        grid_ids = np.rint(hits[:, :2] / resolution).astype(np.int32)
        heights = {}
        for (gx, gy), z in zip(grid_ids, hits[:, 2]):
            heights.setdefault((int(gx), int(gy)), []).append(float(z))
        heights = {key: sum(values) / len(values) for key, values in heights.items()}
        vertices = [(gx * resolution, gy * resolution, z) for (gx, gy), z in heights.items()]
        vertex_ids = {key: index for index, key in enumerate(heights)}
        indices = []
        for gx, gy in heights:
            quad = ((gx, gy), (gx + 1, gy), (gx, gy + 1), (gx + 1, gy + 1))
            if all(key in vertex_ids for key in quad):
                a, b, c, d = (vertex_ids[key] for key in quad)
                indices.extend((a, b, c, b, d, c))
        if indices:
            terrain_shape = p.createVisualShape(
                p.GEOM_MESH,
                vertices=vertices,
                indices=indices,
                rgbaColor=[0.38, 0.40, 0.30, 1.0],
            )
            p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=terrain_shape)
            terrain_source = "Isaac Sim height-scan mesh"
    robot = p.loadURDF(str(args.urdf), useFixedBase=False)

    pybullet_joint_ids = {
        p.getJointInfo(robot, i)[1].decode("utf-8"): i for i in range(p.getNumJoints(robot))
    }
    trajectory_joint_ids = [pybullet_joint_ids.get(name) for name in joint_names]
    absent = [name for name, joint_id in zip(joint_names, trajectory_joint_ids) if joint_id is None]
    if absent:
        raise RuntimeError(f"URDF does not contain trajectory joints: {absent}")

    projection = p.computeProjectionMatrixFOV(
        fov=58.0, aspect=args.width / args.height, nearVal=0.05, farVal=20.0
    )
    start_xy = root_pos[0, :2].copy()
    trajectory_xy = root_pos[:, :2] - start_xy
    fixed_target_xy = (trajectory_xy.min(axis=0) + trajectory_xy.max(axis=0)) / 2.0
    for frame_index in range(0, len(root_pos), args.frame_stride):
        base_pos = root_pos[frame_index].copy()
        base_pos[:2] -= start_xy
        # Isaac Sim uses w,x,y,z; PyBullet uses x,y,z,w.
        quat_wxyz = root_quat[frame_index]
        base_orn = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
        p.resetBasePositionAndOrientation(robot, base_pos, base_orn)
        for joint_id, value in zip(trajectory_joint_ids, joint_pos[frame_index]):
            p.resetJointState(robot, joint_id, float(value))

        if args.camera == "follow":
            target = [float(base_pos[0]), float(base_pos[1]), float(max(0.25, base_pos[2] * 0.7))]
        else:
            target = [float(fixed_target_xy[0]), float(fixed_target_xy[1]), 0.40]
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target,
            distance=2.8,
            yaw=45.0,
            pitch=-20.0,
            roll=0.0,
            upAxisIndex=2,
        )
        _, _, rgba, _, _ = p.getCameraImage(
            args.width, args.height, viewMatrix=view, projectionMatrix=projection, renderer=p.ER_TINY_RENDERER
        )
        image = np.asarray(rgba, dtype=np.uint8)[..., :3]
        frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elapsed = frame_index * dt
        displacement = float(np.linalg.norm(base_pos[:2]))
        if frame_index == 0:
            planar_speed = 0.0
        else:
            previous = root_pos[frame_index - 1, :2] - start_xy
            planar_speed = float(np.linalg.norm(base_pos[:2] - previous) / dt)
        cv2.putText(frame, args.label, (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, args.label, (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (245, 245, 245), 1, cv2.LINE_AA)
        status = f"Isaac Sim trajectory | t={elapsed:.2f}s | distance={displacement:.2f} m | speed={planar_speed:.2f} m/s"
        cv2.putText(frame, status, (24, args.height - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, status, (24, args.height - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        if command_velocity is not None:
            cmd = command_velocity[frame_index]
            actual = root_lin_vel_b[frame_index] if root_lin_vel_b is not None else (0.0, 0.0, 0.0)
            command_text = f"command body vx={cmd[0]:+.2f}, vy={cmd[1]:+.2f}, yaw={cmd[2]:+.2f} | actual vx={actual[0]:+.2f}, vy={actual[1]:+.2f}"
            cv2.putText(frame, command_text, (24, args.height - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 3, cv2.LINE_AA)
            cv2.putText(frame, command_text, (24, args.height - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (245, 245, 245), 1, cv2.LINE_AA)
        cv2.putText(frame, terrain_source, (24, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, terrain_source, (24, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (245, 245, 245), 1, cv2.LINE_AA)
        writer.write(frame)
finally:
    writer.release()
    p.disconnect(client)

print(f"[VIDEO] frames={len(range(0, len(root_pos), args.frame_stride))} fps={fps:.2f} output={args.output}")
