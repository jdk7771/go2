# Go2 WBC/QP 实现路线

当前工程状态：

- `src/interactive_mujoco.cpp` 只负责 MuJoCo 加载、GLFW 可视化、鼠标选中和扰动。
- 控制循环里目前只是 `mj_step(model, data)`，还没有控制器。
- Go2 模型是 floating base + 12 个关节电机。
- `data->ctrl[0..11]` 可以直接写关节力矩，适合做力矩级控制和 inverse dynamics WBC。

建议不要一开始就写完整 WBC。完整 WBC 失败时很难判断是动力学、Jacobian、坐标系、接触状态还是 QP 设置的问题。更稳的路线是先做最小可验证模块，再逐步叠加。

## 总体路线

推荐顺序：

```text
PD 站立
腿部 FK/Jacobian
VMC 站立
接触检测
接触力 QP
swing leg 轨迹
trot 步态
inverse dynamics WBC
MPC 或 Raibert 落脚点
```

## Step 0: 拆控制框架

目标：先把代码结构拆出来，避免所有逻辑都塞进 `interactive_mujoco.cpp`。

建议模块：

- `MujocoSim`: 负责模型加载、仿真步进、渲染、扰动。
- `RobotState`: 保存 base 状态、关节状态、足端状态。
- `Go2Model`: 负责关节索引、actuator 索引、body/site ID、机器人参数。
- `Controller`: 控制器基类，输入 `RobotState`，输出 12 维关节力矩。
- `PdStandController`: 第一个最简单控制器。
- `WbcController`: 后面再实现。

需要完成：

1. 固定 actuator 顺序。
2. 固定 12 个关节的名称和索引。
3. 每个控制周期从 MuJoCo 读状态。
4. 每个控制周期写 `data->ctrl[i]`。
5. 所有 torque 都经过 limit。

验收标准：

- 程序还能正常打开 MuJoCo 窗口。
- 控制器能稳定输出 12 个 torque。
- 打印出来的关节顺序和 XML actuator 顺序一致。

## Step 1: 关节力矩 PD 站立

目标：先让狗能靠关节 PD 保持一个固定姿态。

需要完成：

1. 读取 12 个关节角 `qj`。
2. 读取 12 个关节速度 `dqj`。
3. 设置一个固定站姿 `qj_des`。
4. 使用关节 PD：

```text
tau = kp * (qj_des - qj) + kd * (dqj_des - dqj)
```

5. `dqj_des` 先全设为 0。
6. 对每个电机做 torque limit。

Go2 torque limit 可先按 XML：

- abduction/hip yaw 类：约 `23.7 Nm`
- thigh/calf 类：约 `45.43 Nm`

注意事项：

- 不要一开始用太大 `kp`。
- 先让狗从接近站姿的位置开始。
- 如果从默认姿态直接掉下来，不代表 PD 错，可能是初始姿态太差。

验收标准：

- 狗不会瞬间爆炸或疯狂抖动。
- 狗至少能在固定姿态附近维持几秒。

## Step 2: 状态估计和坐标系

目标：统一所有控制器使用的状态表达。

需要完成：

1. base world position。
2. base world orientation。
3. base linear velocity。
4. base angular velocity。
5. joint position。
6. joint velocity。
7. 每只脚的位置和速度。

坐标系建议：

- world frame: MuJoCo 世界坐标。
- base frame: 机身坐标。
- leg frame: 每条腿髋关节附近的局部坐标，可选。

关键原则：

- 控制 base 姿态、高度、速度时，最好明确变量在哪个 frame。
- 足端力 `f` 和 Jacobian `J` 必须在同一个 frame 中使用。
- `tau = J^T f` 里的 `J` 和 `f` 坐标系不一致时，结果会完全错误。

验收标准：

- 静止时 base 速度接近 0。
- 静止时 foot velocity 接近 0。
- 姿态角没有跳变。

## Step 3: 腿部 FK 和 Jacobian

目标：能正确计算每条腿足端位置和足端 Jacobian。

需要完成：

1. 找到四只脚的 body 或 site ID。
2. 使用 MuJoCo 自带 `mj_jacBody` 或 `mj_jacSite` 获取足端 Jacobian。
3. 分离出每条腿对应的 3 个关节列。
4. 得到每条腿的 `3x3` 线速度 Jacobian。
5. 验证：

```text
v_foot ≈ J_leg * dq_leg
```

6. 用数值差分验证 FK/Jacobian：

```text
J[:, i] ≈ (p(q + eps_i) - p(q)) / eps
```

注意事项：

- MuJoCo floating base 有 6 个 `qvel`，所以关节速度通常从 `qvel[6]` 开始。
- `qpos` 里 floating base 是 7 维，因为四元数，所以关节位置通常从 `qpos[7]` 开始。
- `qpos` 和 `qvel` 的维度不一样，不要直接混用索引。

验收标准：

- 每只脚的位置合理。
- `J*dq` 和 MuJoCo 足端速度一致。
- 数值差分误差在可接受范围内。

## Step 4: VMC 站立控制

目标：在没有 QP 的情况下，让机身具备基本抗扰能力。

VMC 是 Virtual Model Control。思路是：先设计期望机身 wrench，再把 wrench 分配成足端力，最后用 `J^T f` 转成关节力矩。

需要完成：

1. 设置 base 目标高度。
2. 设置 base 目标 roll/pitch/yaw。
3. 用 PD 生成期望机身 wrench：

```text
W_des = [Fx, Fy, Fz, Mx, My, Mz]
```

4. 先假设四只脚都接触地面。
5. 手动分配足端力。
6. 每条腿计算：

```text
tau_leg = J_leg^T * f_leg
```

7. 加 joint damping：

```text
tau += -kd_joint * dqj
```

验收标准：

- 狗在站立状态下能抵抗轻微鼠标扰动。
- 四只脚法向力大致总和接近 `mass * 9.81`。

## Step 5: 接触检测和状态机

目标：知道哪只脚是 stance，哪只脚是 swing。

需要完成：

1. 遍历 MuJoCo contact。
2. 判断 contact 是否发生在 foot 和 ground 之间。
3. 得到每只脚的 contact boolean。
4. 加 hysteresis，避免接触状态一帧一跳。
5. 每只脚维护：

```text
STANCE
SWING
```

6. touchdown 时记录落脚点。
7. liftoff 时记录摆腿起点。

验收标准：

- 静止站立时四只脚状态稳定为接触。
- 手动拖拽或抬腿时，对应脚能变为非接触。
- 接触状态不会高频抖动。

## Step 6: 接触力 QP

目标：第一个真正的 QP。先只优化足端接触力，不直接优化 `ddq` 和 `tau`。

变量：

```text
x = [f_FR, f_FL, f_RR, f_RL]
```

每只脚 3 维，总共 12 维。

目标：

```text
min || A * x - W_des ||^2 + regularization
```

其中：

```text
A_i = [I; r_i_cross]
```

`r_i` 是 foot 相对 base 或 CoM 的位置。

约束：

```text
fz >= 0
fz <= fz_max
|fx| <= mu * fz
|fy| <= mu * fz
swing leg force = 0
```

需要完成：

1. 构建每只脚的 wrench map。
2. 构建总矩阵 `A`。
3. 构建期望机身 wrench `W_des`。
4. 构建摩擦锥约束。
5. 构建摆动腿零力约束。
6. 用 QP solver 求解。
7. 把足端力转为关节力矩：

```text
tau = J^T * f
```

推荐 solver：

- 优先：OSQP。
- 如果想手搓：先写 active-set 或 projected gradient，但不建议一开始这么做。

验收标准：

- 四脚站立时，`sum(fz)` 接近 `mass * 9.81`。
- `fz` 不为负。
- 水平力不超过摩擦约束。
- 鼠标扰动时，QP 输出的力有合理变化。

## Step 7: Swing Leg 轨迹

目标：让非接触腿能按轨迹抬起和落下。

需要完成：

1. 设计 swing duration。
2. 记录 swing 起点。
3. 计算 swing 终点。
4. 用三次或五次多项式生成足端轨迹。
5. z 方向中间加抬脚高度。
6. 计算期望 foot position、velocity、acceleration。
7. 用足端 PD 生成期望 foot force：

```text
f_swing = kp * (p_des - p) + kd * (v_des - v)
```

8. 转成关节力矩：

```text
tau_swing = J^T * f_swing
```

验收标准：

- 单腿摆动时足端轨迹平滑。
- 脚不会砸地或拖地。
- touchdown 时速度不要太大。

## Step 8: Trot 步态

目标：实现最简单的对角小跑。

Trot 分组：

```text
Group A: FL + RR
Group B: FR + RL
```

需要完成：

1. 建立 gait clock。
2. 设置 duty factor。
3. 根据相位切换 stance/swing。
4. stance leg 使用接触力 QP。
5. swing leg 使用足端轨迹跟踪。
6. 合成最终 torque：

```text
tau = tau_stance_qp + tau_swing + tau_damping
```

7. 加安全保护：姿态角过大时停止步态，回到 PD 或关闭 torque。

验收标准：

- 原地 trot 能跑几秒。
- 低速向前命令下能缓慢前进。
- 不需要一开始追求高速和漂亮步态。

## Step 9: Inverse Dynamics WBC

目标：实现真正的 floating-base inverse dynamics WBC。

变量：

```text
ddq: 广义加速度，18 维
tau: 关节力矩，12 维
lambda: 接触力，3 * 接触脚数量
```

动力学等式：

```text
M(q) * ddq + h(q, dq) = S^T * tau + Jc^T * lambda
```

任务项：

- base 高度任务。
- base 姿态任务。
- base 线速度任务。
- base 角速度任务。
- stance foot 不滑动约束。
- swing foot 轨迹任务。
- joint posture regularization。

约束项：

- 动力学等式。
- stance foot 加速度约束：

```text
Jc * ddq + Jdot * dq = 0
```

- 摩擦锥。
- `fz >= 0`。
- torque limit。
- joint limit，可后加。

需要完成：

1. 从 MuJoCo 获取质量矩阵 `M`。
2. 从 MuJoCo 获取 bias force `h`。
3. 构建 floating-base selector `S`。
4. 构建接触 Jacobian `Jc`。
5. 估计或计算 `Jdot * dq`。
6. 构建 QP。
7. 解出 `tau`。
8. 写入 `data->ctrl`。

验收标准：

- 四脚站立时比接触力 QP 更稳。
- 接触脚切换时不会明显爆力矩。
- `tau` 长期不贴着 limit。

## Step 10: 上层速度控制和落脚点

目标：让机器人根据速度命令走起来。

需要完成：

1. 输入命令：

```text
vx_cmd
vy_cmd
yaw_rate_cmd
```

2. base velocity tracking。
3. Raibert foot placement：

```text
p_foot_des = p_hip + 0.5 * stance_time * v_body + k * (v_body - v_cmd)
```

4. yaw 转向时修正落脚点。
5. 限制落脚点范围。

验收标准：

- 给 `vx_cmd` 能前进。
- 给 `yaw_rate_cmd` 能转向。
- 速度不要先追求大，先追求不摔。

## Step 11: MPC

目标：替换简单 Raibert 或 VMC，生成更合理的 base wrench 或 contact force。

建议放到最后做。

常见形式：

- Convex MPC：优化未来若干步的接触力。
- 状态：base position、orientation、linear velocity、angular velocity。
- 输入：每条 stance leg 的 contact force。
- 约束：摩擦锥、接触时序、力限制。

需要完成：

1. 离散化 centroidal dynamics。
2. 给定 gait schedule。
3. 预测 horizon 内优化 contact force。
4. 当前时刻第一组 force 交给下层 WBC。

验收标准：

- 高速或扰动下比单步 QP 稳。
- 对速度命令的响应更平滑。

## 推荐调试顺序

每个阶段都要有明确 debug 输出：

- `qj`, `dqj`
- base position
- base orientation
- foot position
- foot velocity
- contact state
- desired wrench
- QP force
- final torque
- torque saturation ratio

每次只改一个东西：

1. 先不开扰动。
2. 再开轻微扰动。
3. 再加接触切换。
4. 再加步态。
5. 最后加速度命令。

## 最容易踩的坑

1. `qpos` 和 `qvel` 维度不同：floating base `qpos` 是 7 维，`qvel` 是 6 维。
2. actuator 顺序和 joint 顺序不一定一样，必须确认。
3. `J^T f` 的 `J` 和 `f` 坐标系必须一致。
4. 四元数误差不能直接相减。
5. 没有 torque limit 会很容易炸。
6. QP 权重过大等价于制造超大力矩。
7. 接触状态抖动会导致控制器发散。
8. swing leg touchdown 速度太大，会直接把机身打翻。
9. 先写完整 WBC，很难 debug。

## 当前工程下一步

最建议马上做的不是 QP，而是这三件事：

1. 把 `interactive_mujoco.cpp` 拆出控制器接口。
2. 实现 `PdStandController`。
3. 实现足端 Jacobian 读取和验证。

这三件事稳定后，再进入 VMC 和接触力 QP。
