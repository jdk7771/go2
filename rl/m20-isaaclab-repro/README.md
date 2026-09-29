# M20 Isaac Lab 强化学习复现包

本目录是 `Rough-Deeprobotics-M20-v0` 的训练交付物。它保存本次验证过的
安装、预检、训练、checkpoint 归档、迁移和问题记录；不把大型训练日志、模型
权重或 Python/Isaac Sim 二进制文件提交到 Git。

## 固定上游代码

`upstream/IsaacLab` 与 `upstream/rl_training` 是固定提交的 Git submodule。
克隆时使用递归子模块：

```bash
git clone --branch rl-learning-control --recurse-submodules \
  git@github.com:jdk7771/go2.git
cd go2
git submodule update --init --recursive
```

准确提交和来源见 [`SOURCE_REVISIONS.md`](SOURCE_REVISIONS.md)。本项目的
M20 资源路径修复保留在 `patches/apply_m20_asset_fix.sh`。

## 训练结果与模型

最终训练结果不提交到 Git：完整 5,000 次更新的本地归档包含全部 checkpoint、
TensorBoard、配置和控制台日志。私有 Hugging Face 仓库保存每 500 次更新的
checkpoint 与最终 `model_4999.pt`。准确路径、哈希和恢复命令见
[`docs/FINAL_DELIVERY.md`](docs/FINAL_DELIVERY.md)。

## 运行

先阅读 [`docs/SETUP_AND_RUN.md`](docs/SETUP_AND_RUN.md)。其中说明了
AutoDL 的 Vulkan EGL ICD 修复、M20 资源恢复、烟雾测试和正式训练命令。
`scripts/` 目录包含实际使用的脚本；所有凭据仅从环境变量读取，未保存在仓库。

## 视频

`scripts/record_policy_comparison.sh` 可以在渲染正常的 NVIDIA 容器中，以固定
seed 回放两个 checkpoint。当前 AutoDL 镜像的 Isaac Sim 5.1 RTX 渲染器会原生
崩溃，因此该机器未生成伪造视频；详见交付文档。
