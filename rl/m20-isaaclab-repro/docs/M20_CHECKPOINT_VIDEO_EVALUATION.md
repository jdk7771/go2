# M20 checkpoint video evaluation

The watcher records three deterministic 30-second test rollouts for every saved PPO checkpoint.  The purpose is visual, checkpoint-to-checkpoint comparison rather than reward collection.

Fixed protocol:

- seed: `20261009`
- one environment and identical reset conditions
- flat plane
- stairs with the training range of 3–8 cm
- ledge with the training range of 5–8 cm
- 1,500 policy steps (30 seconds at 50 Hz)

Run it beside training:

```bash
bash scripts/evaluate_m20_position_goal_checkpoints.sh \
  /root/autodl-tmp/m20-artifacts/logs/rsl_rl/m20_position_goal_blind_6m_v1/<run-directory>
```

Videos are stored at `<run-directory>/videos/play/` and named `model_<iteration>_<scenario>-episode-0.mp4`.  Marker files prevent a completed checkpoint from being rendered twice; evaluator logs are in `<run-directory>/videos/checkpoint-evaluation-logs/`.
