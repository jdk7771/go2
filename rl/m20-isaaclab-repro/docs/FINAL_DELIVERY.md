# M20 训练最终交付清单

完成日期：2026-09-28。任务为 `Rough-Deeprobotics-M20-v0`，使用 Python
3.11、Isaac Sim 5.1.0、Isaac Lab 2.3.2、RSL-RL 5.0.1 与 PyTorch
2.7.0+cu128。

## 最终结果

- 训练配置：2,048 个并行环境，5,000 次更新。
- 最终文件：`model_4999.pt`。RSL-RL 从零计数，因此它就是第 5,000 次
  更新的最终模型。
- 训练耗时：03:00:39。
- 最后日志指标：mean reward 60.38、mean episode length 996.39、自然超时
  终止 98.28%、坏姿态终止 1.61%、地形越界 0.11%。
- 最终模型 SHA-256：
  `5308793f73e2e39be5ea4bd72a58211652a01cde078520367e24940fa3c7accc`。

这些是训练分布上的指标；正式比较不同策略或宣称泛化前，应在固定命令和
未参与训练的地形上再做评估。

## 本地保存内容

本地根目录：`/home/jiang/m20-repro`。

| 内容 | 位置 | 说明 |
| --- | --- | --- |
| 可复现代码 | `src/IsaacLab`、`src/rl_training` | 已保留完整源码与本地修复。 |
| M20 资源和迁移包 | `server-record/transfer/` | 约 100 MB；含源码、M20 资源、安装/验证/训练脚本、文档及 `SHA256SUMS.txt`。 |
| 完整训练结果 | `server-record/results/complete-run/2026-09-28_08-58-15/` | 约 311 MB；含全部 51 个 checkpoint、TensorBoard events、环境与 agent YAML、源码 diff、最终控制台日志。 |
| 本地 checkpoint 快照 | `server-record/results/baseline/2026-09-28_08-58-15/` | 每 500 步和最终模型的独立副本。 |
| 操作与故障记录 | `docs/SETUP_AND_RUN.md` | 安装过程、Vulkan 修复、训练和视频问题。 |

迁移包校验命令：

```bash
cd /home/jiang/m20-repro/server-record/transfer
sha256sum -c SHA256SUMS.txt
```

## 云端保存内容

当前 AutoDL 服务器保留：

| 内容 | 位置 |
| --- | --- |
| 代码、脚本与文档 | `/root/m20-repro/` |
| 原始完整训练输出 | `/root/autodl-tmp/m20-artifacts/logs/rsl_rl/deeprobotics_m20_rough/2026-09-28_08-58-15/` |
| 训练控制台日志 | `/root/autodl-tmp/m20-artifacts/logs/m20-baseline-fresh-20260928.console.log` |

最终模型的本地与云端 SHA-256 已核对一致。

## Hugging Face 保存内容

私有仓库：<https://huggingface.co/JIANGdk0303/m20-isaaclab-rslrl-checkpoints>

完成训练的 checkpoint 置于独立路径
`runs/2026-09-28_08-58-15/`。其中保留 `model_500.pt`、`model_1000.pt`、
…、`model_4500.pt`，以及最终 `model_4999.pt`。每个文件单独上传，未覆盖
已有版本。此前的 partial-run checkpoint 也仍保留在该私有仓库。

源代码、文档、资源包和完整日志保留在本地及云端；Hugging Face 只保存模型
checkpoint，避免将主机路径、日志和环境内部信息公开或混入模型仓库。

## 下一台服务器的复现

把 `server-record/transfer/` 复制到新服务器后执行：

```bash
bash /path/to/transfer/scripts/restore_from_bundle.sh /path/to/transfer /root/m20-repro
bash /root/m20-repro/scripts/preflight_gpu.sh
bash /root/m20-repro/scripts/bootstrap_m20.sh
bash /root/m20-repro/scripts/verify_m20.sh
```

再用 `model_4999.pt` 进行回放或评估。AutoDL 上需要由
`configure_autodl_vulkan.sh` 写入 EGL Vulkan ICD；该步骤已由预检和训练
脚本处理。

## 已知限制

当前 AutoDL 镜像能稳定完成 headless 训练，但 Isaac Sim 5.1 的 RTX 场景
渲染器在 `--video` 回放时原生崩溃，训练前后均可复现。因此未生成伪造视频。
`scripts/record_policy_comparison.sh` 已保留；在图形渲染正常的 NVIDIA
容器或其他服务器上，可用它对 `model_0.pt` 与最终模型生成固定 seed 的视频
对比。
