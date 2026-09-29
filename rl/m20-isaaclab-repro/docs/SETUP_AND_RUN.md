# M20 Isaac Lab training record

This directory is the reproducible record for the DeepRobotics M20 task
`Rough-Deeprobotics-M20-v0`.  It pins the source code and keeps the commands
used for installation, validation, training, and transfer.

For the final result, storage locations, and hand-off checklist, see
[`FINAL_DELIVERY.md`](FINAL_DELIVERY.md).

## Tested target

| Component | Required version |
| --- | --- |
| Python | 3.11 |
| PyTorch | 2.7.0 + cu128 |
| torchvision | 0.22.0 |
| Isaac Sim | 5.1.0 (`all,extscache`) |
| Isaac Lab | v2.3.2 |
| RSL-RL | 5.0.1 (installed by `rl_training`) |
| Task | `Rough-Deeprobotics-M20-v0` |

The `src/IsaacLab` and `src/rl_training` directories are the local source
copies.  Exact commits and complete package manifests are captured in
`$M20_ARTIFACT_DIR/manifests/` after bootstrap.

`patches/apply_m20_asset_fix.sh` is applied during bootstrap. The current
upstream M20 config resolves the model submodule from the wrong relative
directory and asks for `M20/M20_usd/M20.usd`, while the pinned model contains
`M20/usd/M20.usd`. Without this patch the M20 task cannot find its USD asset.

## Layout on the training server

* `/root/m20-repro` holds code, scripts, and this record on the system disk;
  this is the directory to preserve when saving an AutoDL image.
* `/root/miniconda3/envs/m20` is the Conda environment and is also on the
  system disk.
* `/root/autodl-tmp/m20-artifacts` holds logs, checkpoints, manifests, and
  source-transfer bundles on the faster data disk. Copy this directory off the
  instance before destroying it. It is not included in an AutoDL saved image.
* `/root/autodl-tmp/m20-cache/pip` caches wheels to accelerate reinstalls on
  this particular server.

## Commands

Start a new shell with:

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate m20
```

Before installing or running Isaac Sim on any new provider, run the graphics
preflight. CUDA alone is insufficient: Isaac Sim requires an NVIDIA GPU that
is also visible through Vulkan.

```bash
bash /root/m20-repro/scripts/preflight_gpu.sh
```

It must print `Vulkan NVIDIA GPU check passed.` If it does not, switch to an
instance that exposes NVIDIA graphics/Vulkan to the container. Reinstalling
Python packages cannot correct this provider-side GPU pass-through issue.

### AutoDL headless Vulkan configuration

AutoDL documents a headless Vulkan configuration that uses NVIDIA's EGL ICD.
Run it once as root, then repeat the preflight:

```bash
bash /root/m20-repro/scripts/configure_autodl_vulkan.sh
bash /root/m20-repro/scripts/preflight_gpu.sh
```

The script writes `/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json` and the
preflight, smoke-test, and training scripts automatically select it. This is
distinct from the default GLX ICD used by a desktop session. If the EGL ICD
still cannot enumerate an NVIDIA GPU, the instance must be replaced or repaired
by AutoDL.

Create or repair the full stack:

```bash
bash /root/m20-repro/scripts/bootstrap_m20.sh
```

Run the required validation chain: Isaac Lab Cartpole followed by five M20
learning iterations. The training launch itself checks that M20 is registered
and that its model and simulation assets load:

```bash
bash /root/m20-repro/scripts/verify_m20.sh
```

Run the requested baseline after validation. The script runs from the data disk,
so RSL-RL stores checkpoints and parameter snapshots there. Use `nohup` so
training continues if SSH disconnects:

```bash
nohup bash /root/m20-repro/scripts/train_m20_baseline.sh \
  > /root/autodl-tmp/m20-artifacts/logs/m20-baseline-2048.console.log 2>&1 \
  < /dev/null &
echo $! > /root/autodl-tmp/m20-artifacts/logs/m20-baseline-2048.pid
```

Training output and checkpoints are written below
`/root/autodl-tmp/m20-artifacts/logs/rsl_rl/deeprobotics_m20_rough/`. Preserve
that directory before destroying the machine. A restart uses the run and
checkpoint names with the upstream `--resume --load_run <run> --checkpoint
<model>.pt` options.

Create a compact transfer package containing pinned source archives, exact
Python package lists, and all scripts:

```bash
bash /root/m20-repro/scripts/create_portable_bundle.sh
```

This package does not include Isaac Sim wheels or model checkpoints. It is a
small, auditable source package; use it together with the checkpoint directory
and the manifest on the next service. A Docker image was not built because
this instance does not provide Docker. Saving an AutoDL image preserves the
system-disk code and Conda environment; the artifact directory still needs a
separate copy. After a successful smoke test, use AutoDL's **Save as image**
on this instance. The next instance created from that image skips the Python
and Isaac Sim installation; run the preflight again because Vulkan access is
created by the new instance, not stored in the image.

## Change record

| Time | Action | Result / issue |
| --- | --- | --- |
| 2026-09-27 | Server inspection | Ubuntu 22.04.5; RTX 4090 D (24 GiB); 16 vCPU, 62 GiB RAM; 30 GB system disk and 100 GB data disk; GPU idle. |
| 2026-09-27 | Source retrieval | Local complete clones saved. Remote `rl_training` is shallow and its asset repository is sparse-checked to M20 to avoid downloading unrelated robot assets over the slow GitHub connection. |
| 2026-09-27 | Upstream M20 asset fix | Added `apply_m20_asset_fix.sh`: corrects the asset root and `M20/usd/M20.usd` path. |
| 2026-09-27 | Background Isaac Lab launch | Set `TERM=xterm` in scripts when the session provides `TERM=dumb`; otherwise `isaaclab.sh` exits before starting simulation. |
| 2026-09-27 | Environment bootstrap | Python 3.11.16, PyTorch 2.7.0+cu128 and torchvision 0.22.0+cu128 installed; CUDA reports the RTX 4090 D. Isaac Sim 5.1.0, Isaac Lab v2.3.2 source and `rl_training` with RSL-RL 5.0.1 installed. NVIDIA EULA accepted. |
| 2026-09-27 | Dependency recovery | The first Isaac Lab all-framework install stopped on optional `rl-games` GitHub retrieval. Replaced it with `-i rsl_rl`; repaired the omitted core `isaaclab` package and installed `hidapi`. `cusrl[all]` changed Isaac Sim pins, so `click==8.1.7`, `psutil==5.9.8`, `typing_extensions==4.12.2`, and `wrapt==1.16.0` were restored. |
| 2026-09-27 | Vulkan fix on new AutoDL instance | Default GLX ICD only exposed llvmpipe. AutoDL's documented headless EGL ICD (`libEGL_nvidia.so.0`) made the RTX 4090 visible to Vulkan; `preflight_gpu.sh` now passes. The fix is scripted and selected by validation/training scripts. |
| 2026-09-27 | Smoke tests | Finite Cartpole completed. M20 RSL-RL smoke completed five iterations at 64 environments; GPU use confirmed and checkpoints saved. |
| 2026-09-27 | Asset recovery | New instance had source on its system disk but an empty data disk. Restored the M20 USD/MJCF archive to `/root/m20-repro/deep_robotics_model`, the path expected by the upstream extension after the local asset-path fix. The restore script now uses this location. |
| 2026-09-27 | First 2048-environment baseline | Reached iteration 459/5000 and saved checkpoints through `model_400.pt`; the process was absent at the next check, with no traceback in the captured console log. The partial run, configs, checkpoints, and log were backed up locally. |
| 2026-09-27 | Local result backup | Smoke checkpoints and parameters copied to `server-record/results/smoke/2026-09-27_23-03-04/`. |
| 2026-09-28 | Local result backup | The stopped partial run and current fresh run checkpoints/configs/logs are under `server-record/results/`. |
| 2026-09-28 | Completed 2048-environment baseline | Finished all 5,000 updates in 03:00:39. RSL-RL counts from zero, so `model_4999.pt` is the final model. Its logged mean reward was 60.38, mean episode length 996.39, terrain level 4.0068, XY velocity error 0.9029, yaw error 0.3826, time-out termination 98.28%, bad-orientation termination 1.61%, and terrain-out-of-bounds 0.11%. |
| 2026-09-28 | Local result archive | Copied the complete 311 MB finished run locally: all 51 checkpoints, TensorBoard events, environment and agent YAML snapshots, upstream source diff, and the 11 MB console log. The final `model_4999.pt` SHA-256 is `5308793f73e2e39be5ea4bd72a58211652a01cde078520367e24940fa3c7accc`. |
| 2026-09-28 | Hugging Face backup | Created private repo `JIANGdk0303/m20-isaaclab-rslrl-checkpoints`. Earlier partial checkpoints are retained. The completed run has its initial `model_0.pt`, distinct 500-step snapshots (`500` through `4500`), and final `4999`, all below its unique run path and never overwritten. |
| 2026-09-28 | Policy video comparison | Direct headless `--video` playback crashed in Isaac Sim 5.1's native RTX scene renderer both during training and after the GPU was idle, despite isolated Kit caches. No MP4 was produced on this AutoDL image. The deterministic recording script is retained for a renderer-capable image or a different server. |

## Notes

The currently running instance has one RTX 4090 with 24 GiB VRAM, 16 vCPU, and
120 GiB RAM. The 2,048-environment run started successfully; if it later runs
out of memory, reduce `--num_envs` and record the working setting here.

AutoDL headless instances may have a default GLX ICD that does not enumerate
the GPU. `configure_autodl_vulkan.sh` writes the EGL ICD and `preflight_gpu.sh`
checks that Vulkan sees an NVIDIA device before any Isaac Sim launch.

## Moving to another server

The verified transfer bundle is also kept locally in
`server-record/transfer/`. Copy that directory to the new server, then run:

```bash
bash /path/to/transfer/scripts/restore_from_bundle.sh /path/to/transfer /root/m20-repro
bash /root/m20-repro/scripts/preflight_gpu.sh
bash /root/m20-repro/scripts/bootstrap_m20.sh
bash /root/m20-repro/scripts/verify_m20.sh
```

The restore script verifies hashes, restores the pinned source and M20 asset
archives, and reapplies the M20 asset fix. It deliberately
does not contain any credential. On a server created from a saved AutoDL image,
skip restore/bootstrap when `/root/m20-repro` and the `m20` environment are
already present; run the preflight and smoke test instead.

## Hugging Face checkpoint backup

The current private repository is
<https://huggingface.co/JIANGdk0303/m20-isaaclab-rslrl-checkpoints>. The uploader
creates a private model repository under the account that owns `HF_TOKEN` and
uploads every file in the supplied folder. Keep that folder limited to the
checkpoint files intended for upload. The token is read from the environment;
never save it in source, shell history, logs, or the repo.

For the active baseline, create a separate staging folder for each multiple of
500 and upload it below a run-specific path such as
`runs/2026-09-28_08-58-15/model_500.pt`. Do not reuse an existing path.

## Policy video comparison

`scripts/record_policy_comparison.sh` takes an initial checkpoint, a current
checkpoint, and an empty output directory. It uses one environment and a fixed
seed (`42`) for both replays, with 600 recorded steps each. The current AutoDL
image crashes in Isaac Sim's RTX scene renderer even after training finishes,
so run the script only on a renderer-capable image or a different server.

Upstream references: [rl_training README](https://github.com/DeepRoboticsLab/rl_training/blob/main/README.md)
and [Isaac Lab v2.3.2 installation guide](https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/index.html).
