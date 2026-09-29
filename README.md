# Go2 控制与强化学习实验

这个仓库将传统控制和强化学习实验分到独立分支，避免两套依赖与运行方式互相干扰。

| 分支 | 内容 |
| --- | --- |
| `traditional-control` | Go2 的 MuJoCo C++ 仿真与传统控制代码。 |
| `rl-learning-control` | DeepRobotics M20 / Isaac Lab 的可复现强化学习训练包。 |

`main` 保留原始项目历史。需要传统控制时使用 `traditional-control`；需要复现
强化学习训练时使用 `rl-learning-control`。

## 传统控制

传统控制代码位于 `src/` 和 `test/`，使用 CMake 构建。构建输出位于本地
`build/`，不会提交到仓库。

## 阅读资料

完整 Obsidian 阅读资料保留在 `docs/Work_mark/`，包括四足机器人、运动控制和
强化学习论文，以及个人笔记和学习计划。

## 强化学习结果

大型训练结果不提交到 Git：最终 M20 checkpoint、完整日志和可迁移包保存在
本地归档；每 500 步 checkpoint 与最终模型保存在私有 Hugging Face 仓库。RL
分支中的文档记录了准确路径、版本、哈希和复现命令。
