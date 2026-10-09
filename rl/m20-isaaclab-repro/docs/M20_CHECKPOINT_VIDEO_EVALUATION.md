# M20 checkpoint video evaluation

The watcher records eight deterministic 30-second test rollouts for every saved PPO checkpoint. The purpose is visual, checkpoint-to-checkpoint comparison rather than reward collection.

Fixed protocol:

- seed: `20261009`
- one environment, zero reset perturbation, and a versioned route geometry
- the robot starts at a fixed point; the XY goal is always 6 m forward
- T01: flat route
- T02–T04: three-step routes at 0.05 / 0.10 / 0.15 m per step
- T05–T08: single raised-platform routes at 0.05 / 0.10 / 0.15 / 0.20 m
- every obstacle lies on the direct start-to-goal route; the goal is on a valid support surface and may be on a platform or stair route
- a fixed side-front camera and orange visual goal post are recorded in every video
- 1,500 policy steps (30 seconds at 50 Hz)

Run it beside training:

```bash
bash scripts/evaluate_m20_position_goal_checkpoints.sh \
  /root/autodl-tmp/m20-artifacts/logs/rsl_rl/m20_position_goal_blind_6m_v1/<run-directory>
```

The exact geometry is pinned in `scripts/m20_position_goal_evaluation_manifest.json`. Videos are stored at `<run-directory>/videos/play/` and named `model_<iteration>_T0x-step-0.mp4`. Marker files prevent a completed checkpoint from being rendered twice; evaluator logs are in `<run-directory>/videos/checkpoint-evaluation-logs/`.
