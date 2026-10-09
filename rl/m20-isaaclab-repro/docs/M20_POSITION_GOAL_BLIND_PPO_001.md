# M20-PG-001：6 m Blind Position-Goal PPO Baseline

状态：**已完成代码准备；尚未在服务器启动训练。** 这是新的第一次实验，不复用
`Rough-Deeprobotics-M20-v0` 的权重、日志或结果。

## 实验定义

- Task ID：`M20-PositionGoal-Blind-v0`；每回合以实际 reset 后的位置和 yaw 为基准，在正前方固定 6 m 放置目标。
- Actor：56 维输入（角速度、重力投影、归一化 2D 相对目标、16 维关节位置、16 维关节速度、16 维上一动作）；不输入速度命令、地形扫描、相机或障碍类别。
- Critic：59 维，即 Actor 状态加仿真真值 3D 机身线速度；不使用高度扫描。
- 动作和 PPO：保留 12 个腿关节位置目标 + 4 个轮速目标、50 Hz 策略、200 Hz 物理和原 PPO 网络/超参数；从随机初始化开始。

## 首轮范围

本次代码将方案中的课程设计拆成可审计的第一阶段：12 m × 8 m 地形块中平地/规则楼梯/方形台阶的比例为 50%/35%/15%，楼梯高度 0.03–0.08 m、方形台阶高度 0.05–0.08 m。旧的“按行驶距离升级地形”课程被关闭；0.10 m 以上挑战和基于分场景成功率的自适应课程必须在 PG-001 结果出来后另建实验，不能在本次结果中冒充已验证能力。

## 安全实现与可审计性

- 物理循环每 5 ms 记录世界系三维根部线速度峰值；任一物理步 `v_3d > 0.80 m/s` 立即作为非 timeout 的失败终止。
- `0.65 m/s` 起二次速度惩罚，`0.75 m/s` 起更强惩罚；安全监督在每个物理步把轮速目标置零并记录介入步数。它不会直接写回机器人速度，也不被宣称为严格硬安全保证。
- 成功需要 `d_xy < 0.35 m` 且 `v_3d < 0.15 m/s` 连续 0.5 s；最大回合时间为 30 s。
- TensorBoard 每个完成回合记录 `Episode/max_speed_3d`、违规率、监督介入次数和介入时间。固定 T01–T08 的视频与 20 个扰动重复评测是训练完成后的验收工作，尚未执行，不能预先声称结果。

## 运行顺序

先恢复子模块，再应用补丁和做小规模链路检查：

```bash
cd /root/m20-repro
bash patches/apply_m20_position_goal_blind_patches.sh \
  src/rl_training src/IsaacLab
cd src/rl_training
python scripts/tools/list_envs.py | grep M20-PositionGoal-Blind-v0
bash ../../scripts/train_m20_position_goal_blind.sh --num_envs=64 --max_iterations=5
```

正式训练：

```bash
bash /root/m20-repro/scripts/train_m20_position_goal_blind.sh
```

完整设计原文归档于
[`experiments/2026-10-09_M20_PositionGoal_Blind_PPO_Baseline_实验方案_限速0.8ms.md`](experiments/2026-10-09_M20_PositionGoal_Blind_PPO_Baseline_实验方案_限速0.8ms.md)。
