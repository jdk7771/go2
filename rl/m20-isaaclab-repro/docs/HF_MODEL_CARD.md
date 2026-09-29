---
license: other
tags:
  - isaac-lab
  - isaac-sim
  - rsl-rl
  - quadruped-robot
  - m20
---

# DeepRobotics M20 Isaac Lab training checkpoints

These files are reproducibility snapshots for the `Rough-Deeprobotics-M20-v0`
task from DeepRobotics `rl_training`, trained with Isaac Sim 5.1.0, Isaac Lab
2.3.2, RSL-RL 5.0.1, Python 3.11, and PyTorch 2.7.0+cu128.

The current private repository retains earlier partial checkpoints and stores
new baseline checkpoints at every 500 iterations under unique run paths. Files
are append-only: an existing checkpoint is never replaced.

## Checkpoints

The previous run's `model_400.pt` is from a 2,048-environment run that
reached iteration 459 of 5,000 before the rented instance stopped. The last
logged mean reward was 43.27. This is a partial training checkpoint, not a
completed or validated policy.

`runs/2026-09-28_08-58-15/model_0.pt` is the initial, untrained baseline
checkpoint for before/after playback. The same path contains independent
snapshots at 500, 1,000, 1,500, 2,000, 2,500, 3,000, 3,500, 4,000, and 4,500
updates from a fresh 2,048-environment, 5,000-iteration run started on
2026-09-28, plus the final `model_4999.pt`. RSL-RL counts its 5,000 updates
from zero, so `model_4999.pt` is the completed baseline policy.

At the final logged update, mean reward was 60.38, mean episode length 996.39,
time-out termination 98.28%, bad-orientation termination 1.61%, and
terrain-out-of-bounds termination 0.11%. This is a completed training result;
it still needs held-out terrain and command evaluation before making a
generalization claim.

The exact source revisions and setup notes remain in the local reproduction
package.

## Reproduction notes

- AutoDL headless Vulkan: create `/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json`
  pointing to `/lib/x86_64-linux-gnu/libEGL_nvidia.so.0`, then set
  `VK_ICD_FILENAMES` to that file. AutoDL's default GLX ICD exposed only
  llvmpipe; the EGL ICD made the RTX 4090 visible to `vulkaninfo` and Isaac Sim.
- Restore the M20 model archive to `/root/m20-repro/deep_robotics_model`, which
  is the path expected by the patched asset configuration.
- The test and training commands use `--headless`; remove the unsupported
  `--/rtx/verifyDriverVersion/enabled=false` argument.
- Run from the data disk so checkpoints survive independently of the system
  image. The training script starts a new timestamped run unless resume options
  are explicitly supplied.

See `repro/SETUP_AND_RUN.md` for the complete record and commands.
