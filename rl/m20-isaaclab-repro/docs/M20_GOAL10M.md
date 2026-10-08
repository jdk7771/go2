# M20 新任务：初始位置正前方 10 m 目标

这是在已完成的速度跟踪 baseline 之外新增的训练任务，任务 ID 为
`Rough-Deeprobotics-M20-Goal10m-v0`。旧任务 `Rough-Deeprobotics-M20-v0`、旧配置和
`model_4999.pt` 保持原样。新任务需要重新训练；旧 checkpoint 的评测数字不能当作新任务结果。

## 目标定义与数据流

每次子环境重置后，读取机器人**重置完成后的**世界坐标 `(x0, y0)` 和初始朝向 `yaw0`：

```text
goal_xy = (x0, y0) + 10 m × (cos(yaw0), sin(yaw0))
```

这样即使机器人初始位置和朝向被随机化，目标仍在它当次起点的正前方 10 m。
`goal_xy` 在本回合保持固定。机器人偏离路线时目标不会跟着移动；只有下次重置才重新计算。
目标距离机身水平位置不超过 `0.6 m` 时判定到达，给一次到达奖励并结束该子环境的回合。

actor 的结构和输入长度保持 `57 -> 16`。目标坐标不直接送进 actor；上层目标控制器每个
20 ms 控制周期计算当前位置相对目标的误差，把它旋转到当前机身坐标系，并转换为
`(vx_cmd, vy_cmd, wz_cmd)`。这三个速度指令仍占 actor 观测的第 `6:9` 维：

```text
delta_world = goal_xy - robot_xy
delta_body  = rotate_world_to_body(delta_world, current_yaw)
vx_cmd = clip(delta_body.x, -1.5, 1.5) m/s
vy_cmd = clip(delta_body.y, -1.0, 1.0) m/s
wz_cmd = clip(1.5 × heading_error_to_goal, -1.5, 1.5) rad/s
```

其中 `heading_error_to_goal` 是机身当前朝向和指向目标的方位角之差。进入 0.6 m
到达半径后，三个速度指令归零。原有 57 维传感器观测、16 维动作映射和 PPO
网络无需改变。命令的重采样时间设为 60 s，长于 20 s 回合，因此回合中不会重新选择目标。

奖励继续使用原来的速度跟踪、稳定、接触与平滑项，并新增：

| 项目 | 权重 | 意义 |
| --- | ---: | --- |
| 向目标的有符号速度 `goal_progress` | `+2` | 沿目标方向前进得分，倒退扣分。 |
| 进入 0.6 m 到达半径 `goal_reached` | `+250` | 到达时触发一次；实际还会乘 `step_dt=0.02`。 |

到达目标也作为终止条件，因此不会在终点反复领取奖励。原有地形课程仍按行进距离
调整难度。新任务使用独立的实验目录 `deeprobotics_m20_goal10m_rough`，每 500 次
迭代保存一个 checkpoint，不覆盖已完成的 baseline。

## 更高的楼梯

| 参数 | 已完成的 baseline | 新 Goal10m 任务 |
| --- | ---: | ---: |
| 正向楼梯最高台阶 | 23 cm | 32 cm |
| 反向楼梯最高台阶 | 23 cm | 32 cm |
| 正向楼梯比例 | 20% | 25% |
| 反向楼梯比例 | 20% | 25% |
| 方格台阶最高高度 | 20 cm | 25 cm |
| 两种坡面比例 | 各 10% | 各 5% |

楼梯最低高度仍为 5 cm，地形课程仍从较低难度起步。较高台阶只在较难等级
出现，避免所有环境一开始就面对 32 cm 障碍。每个地形块仍为 8 m × 8 m；
10 m 目标可能跨越相邻地形块，因此该任务训练的是连续地形上的目标移动。

## 本地准备与服务器验证

本地源码位于 `m20-repro/src/rl_training`；GitHub 复现分支中的
`patches/m20_goal10m.patch` 可把同样改动应用到固定上游提交。恢复迁移包或运行
`bootstrap_m20.sh` 时会自动应用。服务器准备好后，先做小规模验证：

```bash
bash /root/m20-repro/patches/apply_m20_goal10m_patch.sh /root/m20-repro/src/rl_training
cd /root/m20-repro/src/rl_training
python scripts/tools/list_envs.py | grep Rough-Deeprobotics-M20-Goal10m-v0
bash /root/m20-repro/scripts/train_m20_goal10m.sh --num_envs=64 --max_iterations=5
```

确认任务注册、目标距离、奖励和终止逻辑正常，再启动正式训练：

```bash
bash /root/m20-repro/scripts/train_m20_goal10m.sh
```

小规模验证只能证明代码链路能跑，不能证明能上 32 cm 楼梯。正式训练后应按台阶高度、
方向和随机种子分别统计 10 m 到达率、耗时、翻倒率、速度误差与碰撞。服务器验证尚未进行，
本文记录的是已完成的本地代码改动和待执行的验证命令。
