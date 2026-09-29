# 固定上游版本

| 项目 | 上游 | 固定提交 | 用途 |
| --- | --- | --- | --- |
| Isaac Lab | `https://github.com/isaac-sim/IsaacLab.git` | `37ddf626871758333d6ed89cf64ad702aef127d0` | v2.3.2 仿真与任务框架。 |
| DeepRobotics RL Training | `https://github.com/DeepRoboticsLab/rl_training.git` | `331ce0f09299d94b8c68f0987c6c8bffbd91eb0a` | M20 任务和 RSL-RL 训练入口。 |
| DeepRobotics 模型资源 | `rl_training` 的嵌套子模块 | `bc574ae6b6c7ae19ab06d110f47704988c508cd8` | M20 USD/MJCF 资源来源。 |

本次针对上游的必要本地修复：

1. 将 M20 资源根目录解析到复现目录的 `deep_robotics_model/`。
2. 将旧的 `M20/M20_usd/M20.usd` 路径改为实际存在的
   `M20/usd/M20.usd`。

两个改动由 `patches/apply_m20_asset_fix.sh` 重放，原始 diff 也随完成训练
保存在本地结果归档中。
