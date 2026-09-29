# M20 fixed stair evaluation — 2026-09-29

This is a post-training evaluation of the completed `model_4999.pt` policy,
not additional training.  It uses the same `Rough-Deeprobotics-M20-v0` task
and exports robot state from Isaac Sim without creating a camera, which avoids
the host's RTX-renderer failure.

## Reproduce

On an instance that contains the saved M20 environment and checkpoints:

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate m20
cd /root/m20-repro
export OMNI_KIT_ALLOW_ROOT=1
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json

bash scripts/run_m20_stair_evaluation_suite.sh
bash scripts/render_m20_stair_evaluation_suite.sh
```

The first script runs physics and writes `.npz` trajectories.  The second
uses PyBullet's CPU TinyRenderer and the supplied M20 URDF only to display the
already exported state.  It does not re-simulate the robot.

Each rollout has one environment, fixed seed 42, 350 control steps at 0.02 s
(7 seconds), a fixed body-frame `vx=+0.60 m/s` command, no terrain or physical
domain randomization, and no external pushes.  The terrain is a single 8 m
Isaac Lab pyramid-stair field, with 30 cm tread width, 1 m border, and the
listed fixed height.  `stairs_up` approaches the centre platform; `stairs_down`
uses the inverted field.

## Measured result

`forward` is final world-x displacement. `peak rise` is the largest base-z
increase from the first valid state. A zero `jumps` count means the recorded
trajectory had no reset discontinuity.

| Scenario | Forward | Peak rise | Mean vx | Last 2 s vx | Jumps | Interpretation |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Untrained `model_0`, up 8 cm | -0.50 m | 0.00 m | -0.08 m/s | 0.00 m/s | 0 | Does not climb. |
| Final `model_4999`, up 5 cm | +3.16 m | +0.29 m | +0.45 m/s | +0.60 m/s | 0 | Completes the climbed section. |
| Final `model_4999`, up 8 cm | +3.33 m | +0.47 m | +0.49 m/s | +0.59 m/s | 0 | Completes the climbed section. |
| Final `model_4999`, up 12 cm | -0.32 m | +0.01 m | -0.05 m/s | +0.01 m/s | 0 | Does not climb under this command. |
| Final `model_4999`, up 16 cm | -0.53 m | +0.01 m | -0.08 m/s | 0.00 m/s | 0 | Does not climb under this command. |
| Final `model_4999`, down 12 cm | +3.68 m | 0.00 m | +0.54 m/s | +0.01 m/s | 0 | Descends about 0.73 m, then slows near the end of the clip. |

The policy was trained with stair-step heights sampled from 5–23 cm, but this
single fixed speed/direction evaluation demonstrates reliable climbing only at
5–8 cm.  It does **not** establish 12 cm or 16 cm climbing capability.

## Delivered videos

The valid H.264 files are in `server-record/videos/stair-suite/` and in
`~/Videos/M20/stairs/`:

- `m20_stair_suite_overview.mp4` — six scenarios in one 1920×720 view.
- `model_0_stairs_up_08.mp4`
- `model_4999_stairs_up_05.mp4`
- `model_4999_stairs_up_08.mp4`
- `model_4999_stairs_up_12.mp4`
- `model_4999_stairs_up_16.mp4`
- `model_4999_stairs_down_12.mp4`

Every video overlays the requested and measured body velocity, base-height
rise, elapsed time, and travelled distance.  The stair meshes in those videos
are reconstructed from the exact fixed Isaac Lab procedural configuration;
the pose, joint state, and motion are from the corresponding Isaac Sim export.
