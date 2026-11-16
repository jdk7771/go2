# M20 点云机身规划

当前入口是 [交互式查看器](outputs/ray_newton_euler_v1/index.html)：左键旋转、滚轮缩放、右键平移。
当前采用 **从起点开始的局部滚动规划**，包含 12 个任务（4 个原场景＋8 个开源世界局部任务）。
网页按几何指令顺序显示逐渐增长的已走路径，以及每一轮的局部初值和局部计划。

## 从起点开始逐段规划（当前方案）

实现：[rolling_planner.py](rolling_planner.py)。每个任务仅指定起点和终点，不输入全程参考路线。

1. 在原任务端点附近筛选机身净空 ≥35 mm 的位姿：前后 ±0.20 m、左右 ±0.30 m、高度 ±0.04 m。
   使用规划点云筛选最近的安全候选，不使用求解结果挑选端点。
2. 从实际起点出发，每轮只读取当前位置 **1.4 m** 范围内的点云。
3. 在当前位置 **0.65 m** 范围内，以位置和姿态的有界局部网格搜索选择可达短期目标。
   全局终点只提供搜索方向和距离评分；进入局部范围后才连接终点。搜索不生成全程路线。
4. 当前位姿和短期目标固定为这一小段的两端，生成 9 节点局部直连初值。
   节点仍为 `(T, v_world, omega_body)`：射线选点 → 表面虚拟力/力矩 → 恢复/弹性/阻尼 →
   牛顿—欧拉加速度 → 半隐式积分和 SO(3) 指数更新。本次测试最多松弛 60 步。
5. 只对准备执行的前段做完整局部点云扫掠检查。若前段不安全，把短期目标缩短到局部搜索的第一条可达边，重新松弛。
   两次都不通过则停车；失败保留，不把未走路径记为成功。
6. 每轮最多推进 **0.10 m**，拆成平移 ≤10 mm、旋转 ≤2° 的几何指令。
   把实际推进位置作为下一轮起点。已走路径保持原样，只追加新的推进段。
7. 到达、停滞、无法找到局部前沿或达到轮数上限后停止；再用独立密集表面检验已走路径。
   只有 **到达终点且连续无碰撞** 才记为规划成功。

这里新增了局部网格目标选择与受阻恢复，因此绕行能力包含这一前端的贡献。
局部虚拟速度在每次短段求解时从零开始，求解内连续积分；发出的位姿序列是几何推进指令。
目前使用已有离线多次扫描点云的局部截取，尚未模拟移动传感器的新回波和遮挡，也没有实体机器人执行控制器。
虚拟积分的 `deltaT` 按 0.015→0.003 s 由粗到细变化，和几何指令的实际执行时间分开记录。

```bash
python -m m20_3d_planner rolling \
  --source m20_3d_planner/outputs/ray_open_world_collision_v1 \
  --output m20_3d_planner/outputs/my_rolling_run \
  --max-replans 80 --relaxation-steps 60 --workers 4
python -m m20_3d_planner view \
  --run m20_3d_planner/outputs/ray_newton_euler_v1 \
  --only-run m20_3d_planner/outputs/my_rolling_run --port 42481
```

归档包含每轮短期目标、局部搜索路径、局部初值/最终轨迹、虚拟状态与加速度抽样、接受/拒绝原因、推进段索引及耗时。

本轮结果见 [12 场景报告](outputs/rolling_local_v4/REPORT.md)：**7/12 到达且连续无碰撞**；
入口窄通道、餐椅、沙发、桶组、柜体 5 个任务未到达，保留了通过连续碰撞检查的已走部分。
单轮耗时中位数约 **3.25 s**，本版本尚未达到实时规划速度。原网页入口只展示这一轮逐步规划。

## 历史实验说明

非平行截面实验原始记录保存在 `outputs/nonparallel_30deg/`，已加入同一查看器的场景列表：
下方水平、上方 30° 斜面；上方水平、下方水平段接左右 30° 斜段（内角 150°）。
默认正视且放大显示，可开关紫色上表面/棕色下表面的截面参考，仍可任意旋转和平移。
这些轮廓仅用于显示，优化仍只接收点云。下方折面建成封闭平台，水平部分 z=0.29 m，顶板下表面 z=0.51 m；
这是一组合成夹具尺寸，没有真实地形测量数据或足端接触模型。详见[新实验报告](outputs/nonparallel_30deg/report/REPORT.md)。

“空旷区进入 · 水平地面＋30° 倾斜顶板窄通道”的原始记录位于 `outputs/open_entry_channel_30deg/`。
通道 x∈[-0.35,0.62] m、净宽 0.44 m，地面 z=0，顶板下表面 z=0.38+tan(30°)·y；
入口前后为空旷区，初始轨迹仍为 `[x,0,0.4,0,0,0]`。同一查看器可播放完整进入过程，
无需另开入口。6D 同时降至 0.28 m 并侧倾 29.91°，密集复检间隙 29.25 mm；
XYZ/XYZ+yaw 分别穿透 9.92/12.37 mm。6D 达到 300 次迭代上限，保守连续下界 11.86 mm。

## 使用

在仓库根目录执行：

```bash
conda env create -f m20_3d_planner/environment.yml  # 环境未创建时执行
conda activate m20-planner
python -m m20_3d_planner view --run m20_3d_planner/outputs/ray_newton_euler_v1 --port 42481
```

查看器自动导出并打开本地页面；指定端口被占用时选择空闲端口，地址见终端；Ctrl+C 停止服务。
`view --export-only` 只导出离线 `index.html`；该文件可直接打开，依赖已内嵌，运行时不访问 CDN。

```bash
# 新实验使用新目录；目标目录已有 protocol.json 时拒绝覆盖。
python -m m20_3d_planner run --output m20_3d_planner/outputs/my_experiment --workers 4
python -m m20_3d_planner view --include-run m20_3d_planner/outputs/my_experiment --export-only
# 刷新原查看器即可，无需再启动服务。

# 可选：导出该实验代表场景的 MP4，集中保存于该实验的 videos/。
python -m m20_3d_planner video --run m20_3d_planner/outputs/my_experiment

# 只复现实验中的两个非平行场景（请另选未使用的输出目录）。
python -m m20_3d_planner run --output m20_3d_planner/outputs/nonparallel_reproduction \
  --scenes flat_floor_sloped_ceiling_30deg flat_ceiling_bent_floor_30deg
python -m m20_3d_planner view --include-run m20_3d_planner/outputs/nonparallel_reproduction --export-only
```

不传 `--scenes` 会运行全部 21 个场景；筛选场景不改变各场景的原始点云种子。
`--modes full` 可只运行六维方法；默认仍运行三个自由度对照。
新实验直接保存每次接受迭代的位姿，无需重跑即可播放过程。
`replay --run <目录>` 仅用于给早期实验补录中间位姿：逐次代价和最终位姿必须与原记录匹配，
失败时拒绝保留新缓存。当前默认实验的 51 组过程已全部补齐并核验。

## 高障碍物绕行实验

入口：[绕行实验网页](outputs/tall_obstacle_wide/index.html)，[实验报告](outputs/tall_obstacle_wide/report/REPORT.md)。
本轮只包含新增场景，历史 20 个场景的输出未随仓库提供。网页默认俯视，可切换三维，
对比初始/最终路径、播放真实接受迭代、逐点检查机身姿态和真值间隙。
“实验 / 点云筛选”切换宽横移边界与原始边界；“优化自由度”切换三个对照。
半透明立柱外形只是显示参考，可在“优化结果”区关闭；优化器只接收采样点云。

障碍物为落地长方体，中心 `[0,0.08,0.70]` m、尺寸 `[0.34,0.44,1.40]` m。
高度超过所有允许机身位姿的上缘，必须从侧面绕行。初值仍为 z=0.4 m、零姿态直线，
起终点固定。小幅 y 偏置来自场景几何，初始轨迹没有绕行方向提示。
原始 ±0.16 m 横移范围无法绕过此立柱，因此新增 `--lateral-limit` 参数，默认仍为 0.16，
新实验显式使用 0.60；高度、姿态边界、权重及停止条件均保持原值。
此参数写入协议及每条结果，历史重放会读取它，配对比较会检查它。

```bash
# 在具有 requirements.txt 所列依赖的环境中执行；输出目录须尚未使用。
python -m m20_3d_planner run --output m20_3d_planner/outputs/tall_obstacle_wide \
  --scenes tall_obstacle_detour --lateral-limit 0.60 --workers 1
python -m m20_3d_planner run --output m20_3d_planner/outputs/tall_obstacle_original_bounds \
  --scenes tall_obstacle_detour --lateral-limit 0.16 --workers 1
python -m m20_3d_planner view --run m20_3d_planner/outputs/tall_obstacle_wide \
  --include-run m20_3d_planner/outputs/tall_obstacle_original_bounds --port 42481
# 如需加入已有的统一查看器（该历史实验存在时）：
python -m m20_3d_planner view --include-run m20_3d_planner/outputs/tall_obstacle_wide \
  m20_3d_planner/outputs/tall_obstacle_original_bounds --export-only
```

本机复跑使用 NumPy 1.26.4、SciPy 1.15.2、Matplotlib 3.10.9；点云种子固定为 1020。

| 横移边界 | XYZ 最小间隙 | XYZ+yaw 最小间隙 | 6D 最小间隙 |
|---|---:|---:|---:|
| 原始 ±0.16 m | -69.99 mm | -69.98 mm | -70.00 mm |
| 放宽 ±0.60 m | -2.09 mm | 24.41 mm | 29.89 mm |

宽边界 6D 选择从 y 正方向绕行，最大横移 0.466 m，最大 yaw 达到 18° 边界，
目标值从 2738.97 降至约 22.78。它没有选择几何上更短的 y 负侧，这体现了局部优化的路径选择局限。
XYZ 虽正式收敛，密集复检仍有 2.09 mm 穿透，说明软碰撞代价及求解成功标志不等于安全保证。
6D 达到 300 次迭代上限，连续间隙保守下界 12.59 mm，未取得连续 ≥20 mm 证书。
这些结果仅验证离线合成点云的机身几何，不验证腿部、动力学或实机执行。

## 虚拟刚体动力学版本

`--solver virtual_wrench` 将每个中间节点作为一个与 body OBB 同形的虚拟刚体，
保存位置、旋转矩阵、世界系线速度和机身系角速度。点云排斥在机身表面产生合力和力臂力矩，
路径恢复弹簧与相邻节点的拉伸/弯曲弹簧提供耦合。运动由以下方程驱动：

```text
m v_dot = F_total - Dv v
I omega_dot = R.T tau_total - omega × (I omega) - Dw omega
p_next = p + dt v_next
R_next = R exp(dt [omega_next]_cross)
```

虚拟质量默认 1，惯量由均匀箱体公式 `m/12 * (边长平方之和)` 计算；线阻尼默认 40，
角阻尼默认 2，最大积分步长 0.01。数值采用阻尼精确衰减、其余项半隐式积分。
若势能加动能的总机械能增长超过数值容差，则减半步长重试，稳定后逐步恢复。
这不是按势能作线搜索；惯性允许势能上升、动能下降。

此版本保留原碰撞势能及全部弹簧权重，以隔离运动更新机制。
为了与原势能完全一致，旋转恢复及耦合弹簧仍定义在欧拉角上，它们的广义作用量通过
`A(rpy).T tau = angular_effort` 映射成空间力矩；不是简单将欧拉角梯度当作机身角加速度。
陀螺项使用机身坐标系，姿态通过旋转矩阵指数映射更新。
端点固定；位姿盒约束使用位置投影、移除向外速度和惯量度量下不增加动能的角速度约束冲量。
仅当约束力平衡残差及线/角速度均达阈值时标记收敛。相同残差也作为旧优化器的诊断，
但新分支不调用 `cost_grad()` 或 `scipy.optimize.minimize()` 来决定更新。

```bash
# 初次运行可使用下面的目录；已有 protocol.json 时须另选新目录。
python -m m20_3d_planner run --output m20_3d_planner/outputs/tall_obstacle_virtual_wrench \
  --solver virtual_wrench --scenes tall_obstacle_detour --lateral-limit 0.60 \
  --point-selection radius --max-length 0.6 --workers 1
python -m m20_3d_planner view --run m20_3d_planner/outputs/tall_obstacle_wide \
  --include-run m20_3d_planner/outputs/tall_obstacle_virtual_wrench --port 42481
# 可选：显式设置虚拟参数。记录到新实验目录，不覆盖已有实验。
# --maxiter 4000 --wrench-dt 0.01 --virtual-mass 1 --linear-damping 40 --angular-damping 2
```

原默认求解器仍为 `--solver lbfgsb`，默认 300 次迭代；虚拟动力学默认 4000 步。
所有配置、虚拟箱体惯量和停止原因均写入实验记录及源代码快照；历史重放读取相同配置，
配对点云筛选比较拒绝混用不同求解器或动力学配置。
本轮使用 0.6 m 半径查询，超过机身半对角线与激活距离之和，因此保留完整碰撞目标；
点云、种子、初值、权重和位姿边界与宽边界基线相同。

本轮三种自由度均达到 4000 步上限，密集复检均仍有约 70 mm 穿透。6D 最大横移约 0.148 m、
最大 roll 23.44°，势能从 2738.97 降到 1713.56，虚拟时间 11.582，最终动能约 3.00e-8。
这表明直接换成虚拟动力学没有解决直线穿透初值的局部陷阱；没有按真值结果修改初值或筛选成功记录。
原 L-BFGS-B 结果仍独立保留，不能将其成功轨迹当作新求解器结果。

查看器默认选择新版本，“优化过程”提供当前观察节点的合力、合力矩、速度、整条轨迹动能和虚拟时间。
红色箭头表示世界系合力，紫色表示世界系力矩轴，箭头长度经过显示缩放；合力包含弹簧作用，
不含固定端点及盒约束反力。所有数值使用虚拟单位，不是传给真实机器人电机的命令。
`iteration_wrenches` 保存世界系 `[F,tau]`，`iteration_twists` 保存 `[v_world,omega_body]`，
`iteration_times` 和 `iteration_kinetic_energy` 与接受积分步逐项对应。
虚拟松弛时间不是执行时间；这个模型没有足端接触、关节、重力平衡或实机动力学可行性保证。

## 射线牛顿—欧拉版本（ray_dynamics）

本版本按节点状态 `X_k=(T_k,v_world,k,omega_body,k)` 保存和演化轨迹。
`T` 是实际存储的 4×4 SE(3) 矩阵；欧拉角仅用于已有位姿边界、记录和显示。
这是独立的新实验版本，显式使用 `--solver ray_dynamics`；不调用 L-BFGS-B 或 `cost_grad()` 更新节点。

每轮按以下顺序执行：

1. 从各节点当前机身中心出发，KD-tree 查询半径内点云，按全空间方位角/仰角格保留首命中点。
   默认 0.6 m、4°。这是有限角宽的离散射线近似，不是对不存在的网格表面做精确求交；
   首命中可能舍弃相关障碍，因此完整表面验收仍不可省略。
2. 对命中点计算到机身 OBB 的最近表面点、带符号距离和法向。
   计算碰撞排斥力及 `r_surface × F` 力矩，保留非中心作用位置。
3. 加入路径/高度恢复、姿态恢复和节点间弹性；旋转项采用 SO(3) 对数。
   姿态恢复使用 `Log(R_ref.T R)`，旋转弹性使用 `phi_k=Log(R_k.T R_(k+1))`
   及相邻 `phi` 的差分；通过 SO(3) 对数雅可比将旋转弹性映射成力矩。
4. 显式计算牛顿—欧拉加速度（对角箱体惯量在机身系）：

   ```text
   F_total_world = F_elastic_world - Dv * v_world
   tau_total_body = R.T * tau_elastic_world - Dw * omega_body
   a_world = F_total_world / m_v
   alpha_body = (tau_total_body - omega_body × (I_v * omega_body)) / I_v
   ```

5. 半隐式欧拉先更新速度，再更新 SE(3) 位姿：

   ```text
   v_next = v + dt * a
   omega_next = omega + dt * alpha
   p_next = p + dt * v_next
   R_next = R * Exp(dt * [omega_next]_cross)
   ```

6. 起终点的 T 固定，速度和加速度记录为零。沿用原位姿范围，通过位置/姿态投影和
   速度约束冲量处理边界。约束作用量残差与速度连续 5 步达到容差才标记收敛。
   求解结束后，无论是否收敛，均对线性平移 + SO(3) 测地插值进行完整留出表面密集复检，
   并扣除插值区间运动界和合成表面覆盖误差，报告保守连续间隙下界。

时间步调度默认 `0.01 → 0.001`，前 1000 步按几何序列减小，之后保持细步长。
每次尝试都使用同轮固定的射线命中集合检查机械能稳定性，必要时再减半。
当前版本接受后允许以 `--wrench-dt-growth 1.05` 逐步恢复实际时间步，始终不超过由粗到细的调度上限，
且每次重新检查单步运动限制；设为 1 可复现只减不增行为。实际步长也可能小于最终调度值。
接受后重新选点；命中切换会改变势能，因此没有要求跨命中集合全局能量单调。
这里的加速度不含位姿边界约束冲量，虚拟时间不等于机器人执行时间。

```bash
# 另选未使用的输出目录，保留已有实验。
python -m m20_3d_planner run --solver ray_dynamics --initialization straight \
  --output m20_3d_planner/outputs/my_ray_dynamics \
  --modes full --scenes beam_h430_yaw00_tilt+0 tilted_corridor_30deg tall_obstacle_detour \
  --lateral-limit 0.60 --workers 1 --maxiter 4000 \
  --wrench-dt 0.01 --wrench-final-dt 0.001 --wrench-schedule-steps 1000
python -m m20_3d_planner view --run m20_3d_planner/outputs/my_ray_dynamics
```

本轮原始记录位于 `outputs/ray_newton_euler_v1/`。
三个场景均从相同零姿态直线开始，完整 6D、4000 步，尚未达到正式收敛条件：

| 场景 | 规划墙钟时间 | 密集采样最小间隙 | 连续间隙保守下界 | 连续 ≥20 mm 证书 |
|---|---:|---:|---:|---|
| 低梁 h430 | 63.21 s | 44.38 mm | 27.37 mm | 是 |
| 30° 倾斜通道 | 84.27 s | 16.41 mm | -0.73 mm | 否 |
| 高障碍物 | 72.71 s | -70.00 mm | -88.47 mm | 否 |

这轮实现验证了新积分流程和状态记录，但仍有局部陷阱，也未达到实时要求。
射线筛选会改变碰撞势能，不能仅凭删点后目标值较低判断避障效果。
[离线查看器](outputs/ray_newton_euler_v1/index.html) · [原始报告](outputs/ray_newton_euler_v1/report/REPORT.md)。

查看器“优化过程”可检查当前节点的合力/力矩、速度、加速度和实际/调度步长。
可复查的 `trials/*.npz` 字段：

| 字段 | 约定 |
|---|---|
| `iteration_transforms` | 每步每节点的 SE(3) 4×4 矩阵 |
| `iteration_twists` | `[v_world, omega_body]` |
| `iteration_accelerations` | `[a_world, alpha_body]`，不含边界冲量 |
| `iteration_wrenches` | 含阻尼的世界系 `[F_total,tau_total]`，不含边界反力 |
| `iteration_torques_body` | 含阻尼的机身系力矩，用于牛顿—欧拉方程 |
| `iteration_elastic_wrenches` | 不含阻尼的世界系 `[F_elastic,tau_elastic]` |
| `iteration_time_steps` / `iteration_scheduled_time_steps` | 实际 / 调度时间步 |

旋转弹性定义和射线输入均不同于旧版本，不能将二者耗时差异全部归因于积分器。
当前版本是离线机身规划实验，未验证实时频率、足端接触和实机跟踪。

### 单步运动限制（当前版本）

默认每个节点每步最多平移 **10 mm**、旋转 **2°**。可用
`--max-node-translation-step 0.01 --max-node-rotation-step-deg 2` 显式配置。
先根据牛顿—欧拉加速度预测速度，在位姿边界裁剪前检查 `dt*||v_next||` 和 `dt*||omega_next||`；
后者使用未折返转角，避免整圈旋转后矩阵差很小而漏检。超限就保持原状态、减半 `dt` 并重新积分。
裁剪后还会检查实际平移量和 SO(3) 相对转角，然后进行原有机械能检查。
单步限制通过缩小时间步实现；已有的位姿边界约束仍按原方式投影并处理约束冲量。

`history.csv` 增加拟议/实际单步平移、转角；`*.npz` 增加 `iteration_motion_steps`
与 `iteration_proposed_motion_steps`，单位分别为米、弧度。
结果记录区分运动超限重试 `motion_limited_retries` 和能量重试 `energy_rejected_steps`。
查看器显示当前积分步的全轨迹最大单步位移与转角，历史无此限制的版本标注为“历史未限步”。

四个场景的首轮固定参数实验保存在 `outputs/ray_newton_euler_limited/`：低横梁、30° 倾斜通道、
入口窄通道、高立柱。均沿用下方立柱调参权重、70 mm 激活距离、3000 步上限和相同初值，
只新增上述单步限制；没有按场景重新调参或筛掉失败记录。
首轮沿用旧的只减不增时间步，3000 步只推进约 0.24～0.32 个虚拟秒。
随后加入接受步后的步长恢复，在完全相同的场景/权重/初值上复跑，保存为
`outputs/ray_newton_euler_limited_adaptive/`；全部迭代仍受 10 mm / 2° 限制。

```bash
python -m m20_3d_planner run --solver ray_dynamics --initialization straight \
  --output m20_3d_planner/outputs/my_ray_limited \
  --scenes beam_h430_yaw00_tilt+0 tilted_corridor_30deg open_entry_flat_floor_channel_30deg tall_obstacle_detour \
  --modes full --lateral-limit 0.60 --workers 2 --maxiter 3000 \
  --weights-json m20_3d_planner/configs/ray_tall_weights.json --activation-distance 0.07 \
  --linear-damping 4 --angular-damping 2 --virtual-mass 1 \
  --wrench-dt 0.02 --wrench-final-dt 0.002 --wrench-schedule-steps 1500 \
  --max-node-translation-step 0.01 --max-node-rotation-step-deg 2 --wrench-dt-growth 1.05 --label '单步受限版'
```

### 单步受限、自适应步长的实测结果

四个场景使用完全相同的参数，原始报告在 `outputs/ray_newton_euler_limited_adaptive/report/REPORT.md`。

| 场景 | 最小采样间隙 [mm] | 连续间隙保守下界 [mm] | 正式收敛 | 耗时 [s] |
|---|---:|---:|---|---:|
| beam_h430_yaw00_tilt+0 | 52.03 | 35.18 | 是 | 32.19 |
| tilted_corridor_30deg | -56.48 | -75.97 | 否 | 58.37 |
| open_entry_flat_floor_channel_30deg | 23.93 | 6.43 | 否 | 103.48 |
| tall_obstacle_detour | -70.00 | -89.88 | 否 | 49.96 |

逐帧 SE(3) 状态独立核验保存于 `motion_validation.json`：全部接受步均满足 10 mm / 2°。
立柱首步最大平移由历史未限步版的约 608 mm 降至 8.75 mm。低横梁通过连续 ≥20 mm 证书；
入口窄通道下界为正但不足 20 mm；倾斜通道和立柱仍碰撞。单步限制修复了数值跳跃，没有解决局部避障失败。
这些是离线几何试验，不代表实时性或实机可执行性。

### 穿障初值与开源多场景测试（历史全程变形方案）

当前 `ray_dynamics` 默认用 `--initialization small_bend`：统一轻微 S 弯，横移不超过 25 mm，
零姿态、零初速，不搜索避障初值。保留牛顿—欧拉动力学和 10 mm / 2° 单步限制。

新实验 `outputs/ray_open_world_collision_v1/` 包含 12 个任务：
低横梁、倾斜通道、入口通道、立柱，以及原始 AWS 仓库/住宅世界的货架支脚、堆箱、桶组、
搬运车、餐椅、茶几、沙发、柜体周边。源世界只做刚体坐标转换与安全范围外裁剪，不缩放或重新摆放几何。
准备任务时先检查端点安全及中段碰撞，固定任务后再运行规划器，不按求解结果筛掉失败。
每个任务保存初始穿入间隙、最终间隙、连续下界、耗时及完整迭代。

本轮结果：12 个初值全部穿障；最终 3/12 采样无碰撞，只有低横梁与倾斜通道 2/12 得到连续无碰撞正下界。
搬运车的采样间隙仅 0.96 mm，连续下界 −13.82 mm，尚不能确认连续无碰撞；其余 9 个仍碰撞。
连续 ≥20 mm 及严格动力学收敛均为 0/12。全部 36000 个接受步满足 10 mm / 2°。
完整表格、失败结果与三维总览见 `outputs/ray_open_world_collision_v1/BENCHMARK.md`。

源仓库地址、固定提交、原始许可和逐文件校验清单见 [数据说明](datasets/open_worlds/README.md)。
需要准备数据时先执行 `python m20_3d_planner/datasets/open_worlds/fetch_sources.py`，
再执行 `python -m m20_3d_planner.open_worlds`。`--suite mixed` 合并合成与开源任务；
`--suite open_worlds` 只运行 8 个导入任务，`--suite synthetic` 保留原 21 场景。

```bash
MPLCONFIGDIR=/tmp/go2-matplotlib XDG_CACHE_HOME=/tmp/go2-cache \
  /home/jiang/miniconda3/envs/nero/bin/python -m m20_3d_planner run \
  --output m20_3d_planner/outputs/ray_open_world_reproduce \
  --suite mixed --solver ray_dynamics --initialization small_bend --modes full --workers 4 --maxiter 3000 \
  --scenes beam_h430_yaw00_tilt+0 tilted_corridor_30deg open_entry_flat_floor_channel_30deg tall_obstacle_detour \
    open_warehouse_shelf open_warehouse_clutter open_warehouse_bucket open_warehouse_pallet_jack \
    open_house_chairs open_house_coffee_table open_house_sofa open_house_cabinet \
  --weights-json m20_3d_planner/configs/ray_corridor_weights.json \
  --activation-distance 0.035 --point-selection rays --ray-angle-deg 2 --lateral-limit 0.6 \
  --virtual-mass 1 --linear-damping 20 --angular-damping 1 \
  --wrench-dt 0.02 --wrench-final-dt 0.002 --wrench-schedule-steps 1500 \
  --max-node-translation-step 0.01 --max-node-rotation-step-deg 2 --wrench-dt-growth 1.05 \
  --label '穿障初值·开源多场景动力学'
```

开源网格采用三角形确定性覆盖采样，连续下界使用各任务自己的覆盖半径。
距离复检的 KD-tree 查询使用严格距离界剪枝，仍计算全真值表面的最小距离，不使用规划器的射线删点结果。
额外的离线参考网格搜索仅用于可达性说明，参考路线不传给规划器，且不保证可用固定 23 个锚定节点表示。

### 点云选择的平滑弯曲初值（历史对照）

该轮使用 `--initialization cloud_bow`；当前默认已改为未经避障的 `small_bend`。
显式指定 `--initialization straight` 可保留历史直线初始化；历史实验归档不变。

初值由新增的 `initialization.py` 生成：

1. 固定起终点，生成左右对称的多幅度横向弯曲和升降候选。端点使用五次平滑过渡，
   中段留出宽平台，给有限长度的机身预留进入/退出障碍区域的距离。
2. 对较好的位置候选和小幅居中候选，试探正负侧倾。候选只读取点云、机身尺寸和运动边界，
   不读取场景名、障碍几何参数或真值表面。
3. 用 KD-tree 搜索附近全部原始点，在节点间也采样，按点到机身的间隙不足惩罚、偏移、
   路长及侧倾幅度评分。初值评分间隙为 `min(激活距离, 20 mm)`，避免要求窄通道达到不可能的激活间隙。
4. 选中曲线作为初始位姿及 XY/姿态恢复参考；高度恢复仍指向名义高度。初速度为零，
   随后执行原来的射线—虚拟力/力矩—牛顿欧拉—半隐式积分流程，仍限制单步 10 mm / 2°。

这是新增的有限候选初值搜索，不能把绕行方向归功于纯局部动力学自行发现。
初值可存在碰撞或不满足 20 mm 间隙；最终仍需整条路径的真值与连续间隙验证。
单步运动限制从选中的第 0 帧开始计量；初值候选之间的差异不属于积分运动。
候选及评分、选中编号、耗时全部保存在 `protocol.json` 的 `initialization_details` 中。
每个 `scenes/*.npz` 同时保存实际 `initial_trajectory` 和原 `straight_reference_trajectory`，
该轮归档包含四个场景；橙色路径显示实际初值。

本轮四场景记录在 `outputs/ray_cloud_bow_v1/`；动力学和势场参数与上一轮
`ray_limited_corridor_tuned` 完全相同，改变的是初值及其恢复参考。

| 场景 | 最终采样间隙 mm | 连续下界 mm | 连续 ≥20 mm | 动力学收敛 |
|---|---:|---:|---|---|
| 低横梁 | 28.45 | 11.18 | 否 | 否 |
| 30° 倾斜通道 | 25.26 | 8.13 | 否 | 否 |
| 入口窄通道 | 25.14 | 7.81 | 否 | 否 |
| 立柱绕行 | 84.60 | 67.86 | 是 | 是，600 步 |

四场景最终连续间隙下界均为正；两种通道均从内部穿过，没有从侧墙外绕过。
立柱初值横向弯曲 −0.34 m，初始间隙已经达到 85 mm，动力学随后稳定到 84.60 mm，
总耗时约 14.12 s（含初值筛选 4.12 s）。这说明初值前端解决了本场景的绕行方向选择；
不代表原直线初值下的纯局部动力学解决了全局绕行。另三场景达到 3000 步上限，尚未正式收敛。
全部 9600 个接受步通过独立限幅核验；完整对比与穿越检查见
`outputs/ray_cloud_bow_v1/INITIALIZATION_COMPARISON.md`。

```bash
MPLCONFIGDIR=/tmp/go2-matplotlib XDG_CACHE_HOME=/tmp/go2-cache \
  /home/jiang/miniconda3/envs/nero/bin/python -m m20_3d_planner run \
  --output m20_3d_planner/outputs/ray_cloud_bow_reproduce \
  --solver ray_dynamics --initialization cloud_bow --modes full --workers 2 --maxiter 3000 \
  --scenes beam_h430_yaw00_tilt+0 tilted_corridor_30deg open_entry_flat_floor_channel_30deg tall_obstacle_detour \
  --weights-json m20_3d_planner/configs/ray_corridor_weights.json \
  --activation-distance 0.035 --point-selection rays --ray-angle-deg 2 --lateral-limit 0.6 \
  --virtual-mass 1 --linear-damping 20 --angular-damping 1 \
  --wrench-dt 0.02 --wrench-final-dt 0.002 --wrench-schedule-steps 1500 \
  --max-node-translation-step 0.01 --max-node-rotation-step-deg 2 --wrench-dt-growth 1.05 \
  --label '点云弯曲初值·受限动力学'
```

### 单步受限后的参数调试

`outputs/ray_limited_parameter_pilots/REPORT.md` 保留 8 组参数试验及全部失败结果，
每组 2000 步，仍用原始直线零速度初值、固定端点和 10 mm / 2° 单步限制。
四组倾斜通道试验均消除了采样碰撞；四组立柱试验仍碰撞，没有把这些结果筛掉。

选择通道试验中间隙最好的一组，用相同参数统一复跑四个场景、每场景最多 3000 步，
结果和完整迭代历史在 `outputs/ray_limited_corridor_tuned/`。这是已知场景调参及参数迁移检查，
不是独立泛化验证。新权重保存在 `configs/ray_corridor_weights.json`。

| 参数 | 前一轮统一参数 | 通道调参参数 |
|---|---:|---:|
| 碰撞激活距离 [mm] | 70 | 35 |
| 射线角分辨率 [°] | 4 | 2 |
| 横向路径恢复权重 | 2 | 100 |
| 高度恢复权重 | 32 | 300 |
| 姿态恢复权重 | 30 | 0.3 |
| 平移二阶弹性权重 | 300 | 110 |
| 线阻尼 | 4 | 20 |
| 角阻尼 | 2 | 1 |

较弱的姿态恢复允许机身侧倾，较强的位置恢复抑制侧向/向下漂移；更密的射线改善表面采样。
通道理想对齐时单侧几何间隙约 32 mm，70 mm 激活距离会使两侧持续产生较强排斥。
碰撞激活距离是产生虚拟力的范围，正式连续间隙检查阈值仍为 20 mm。
采样无碰撞、连续间隙证书、动力学平衡收敛分别报告，不能相互代替。

四场景正式复跑：倾斜通道采样间隙由 −56.48 mm 改善至 +25.21 mm，连续下界 +8.07 mm；
入口通道为 +24.46 mm / +6.91 mm（采样 / 连续下界）；立柱仍碰撞。
低横梁采样间隙反而降至 +28.45 mm，连续下界 +11.17 mm，丢失前一版的连续 ≥20 mm 证书。
因此这组参数不能统一替代所有场景的原参数。新四场景都未达到严格动力学平衡收敛；
全部 12000 个接受步经独立核验满足运动限制。
完整新旧对比见 `outputs/ray_limited_corridor_tuned/TUNING_COMPARISON.md`。

```bash
MPLCONFIGDIR=/tmp/go2-matplotlib XDG_CACHE_HOME=/tmp/go2-cache \
  /home/jiang/miniconda3/envs/nero/bin/python -m m20_3d_planner run \
  --output m20_3d_planner/outputs/ray_limited_corridor_reproduce \
  --reference-results m20_3d_planner/robot.json \
  --solver ray_dynamics --initialization straight --modes full --workers 2 --maxiter 3000 \
  --scenes beam_h430_yaw00_tilt+0 tilted_corridor_30deg open_entry_flat_floor_channel_30deg tall_obstacle_detour \
  --weights-json m20_3d_planner/configs/ray_corridor_weights.json \
  --activation-distance 0.035 --point-selection rays --ray-angle-deg 2 --lateral-limit 0.6 \
  --virtual-mass 1 --linear-damping 20 --angular-damping 1 \
  --wrench-dt 0.02 --wrench-final-dt 0.002 --wrench-schedule-steps 1500 \
  --max-node-translation-step 0.01 --max-node-rotation-step-deg 2 --wrench-dt-growth 1.05 \
  --label '受限积分·通道调参' --tuning-note '已知倾斜通道选参，四场景统一参数复跑'
```

### 高障碍物参数调试（历史未限步版本）

以下是增加单步限制之前的记录。最终立柱路径曾通过连续间隙检查，但首步部分节点跳动约 0.61 m，
依赖大步长和位姿边界裁剪，不能作为稳定虚拟动力学变形的证据。新的单步受限实验独立保存，
不把这里的成功轨迹当成新版本结果。

针对 `tall_obstacle_detour` 进行了 13 组参数试验，原始记录保存在
`outputs/ray_tuning_pilots/`、`outputs/ray_tuning_round2/`；各目录的 `source/parameter_sweep.py`
保存对应试验脚本。使用同一噪声点云、相同 23 节点直线初值、固定端点及 ±0.60 m 横移边界。
这是已知场景调参，不能作为独立场景泛化实验；没有给节点预设绕行方向或改动障碍物。

| 参数 | 原版本 | 立柱调参版 |
|---|---:|---:|
| 线阻尼 | 40 | 4 |
| 路径恢复 `path_xy` | 18 | 2 |
| 姿态恢复 `attitude` | 0.30 | 30 |
| 平移弯曲 `acceleration_translation` | 110 | 300 |
| 碰撞激活距离 | 47 mm | 70 mm |
| 初始 / 最终调度步长 | 0.01 / 0.001 | 0.02 / 0.002 |
| 步长调度步数 / 总步数上限 | 1000 / 4000 | 1500 / 3000 |

较低线阻尼和路径恢复允许横移；较强姿态恢复抑制不必要的翻转；较强弯曲弹性与更大的激活距离
改善节点间回拐切角。70 mm 是软碰撞作用范围，最终验收目标仍为连续 ≥20 mm，没有放宽验收。
虚拟质量仍为 1，角阻尼仍为 2，其余权重沿用原值。

权重覆写文件为 `configs/ray_tall_weights.json`，未列字段继承 `settings.planner_weights()`。
完整配置写入协议和逐条结果；历史重放使用保存的权重和激活距离。

```bash
python -m m20_3d_planner run --solver ray_dynamics --initialization straight \
  --output m20_3d_planner/outputs/my_ray_tuned \
  --scenes tall_obstacle_detour --modes full --lateral-limit 0.60 --workers 1 --maxiter 3000 \
  --weights-json m20_3d_planner/configs/ray_tall_weights.json --activation-distance 0.07 \
  --linear-damping 4 --angular-damping 2 --virtual-mass 1 \
  --wrench-dt 0.02 --wrench-final-dt 0.002 --wrench-schedule-steps 1500 \
  --label '立柱调参版' --tuning-note '已知立柱场景调参，非独立泛化验证'
```

正式复跑记录保存在 `outputs/ray_newton_euler_tuned/`；网页可在“实验 / 点云筛选”中对比原版本与立柱调参版。
正式复跑耗时 38.22 s，最小采样间隙 46.54 mm，
连续间隙保守下界 29.56 mm，获得连续 ≥20 mm 证书。
3000 步仍未达到严格的力平衡/速度收敛条件，不能据此宣称实时或普遍收敛。

## 代码位置

| 文件 | 职责 / 扩展位置 |
|---|---|
| `core.py` | 机身模型、距离、碰撞能量与 wrench、轨迹目标和解析梯度 |
| `virtual_wrench.py` | 虚拟 body 质量/惯量、合力/力矩、阻尼动力学、SO(3) 更新与约束冲量 |
| `ray_dynamics.py` | SE(3) 节点状态、SO(3) 弹性、牛顿—欧拉加速度、半隐式积分与由粗到细时间步 |
| `point_selection.py` | 从当前 body 中心做最大距离查询、球面分格首命中；只返回原始点索引 |
| `settings.py` | 自由度、节点数、安全目标、势能激活距离、权重与停止条件 |
| `scenes.py` | 合成场景、表面采样、双雷达与噪声点云流程；新增场景从 `build_scenes()` 加入 |
| `experiment.py` | 优化、收敛诊断、独立验收和运行记录；不接收场景提示生成初值 |
| `reporting.py` | 实验报告与静态图 |
| `comparison.py` | 核对输入及协议后，汇总不同筛选实验的效果和实际规划耗时 |
| `visualization/data.py` | 浏览器与视频共用的数据读取和历史重放核验 |
| `visualization/web.py` | 导出离线查看器 / 启动本地服务 |
| `visualization/assets/` | `index.html`、`viewer.css`、`viewer.js`；修改交互与画面从这里入手 |
| `visualization/video.py` | 唯一的 MP4 导出器 |
| `tests/` | 数值回归与真实浏览器交互检查 |

导出显示的位姿约定为 `[x,y,z,roll,pitch,yaw]`，单位为米和弧度；新版本内部直接保存 SE(3)，
旋转为 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`。浏览器按同一约定显示，未夸大尺寸或角度。
可视化只消费运行记录，不修改规划数据。改算法请输出到新实验目录，以保留对照。
`view --include-run` 将实验登记在主入口的 `viewer_sources.json`，重启或重新导出会保留。
`viewer_runs/` 使用相对目录链接提供原实验的视频与报告；不复制原始结果，也不混算不同实验的成功率。
同名场景自动保留独立 ID，报告按钮与视频按当前场景定位其来源。

## 输出位置

每次实验只有一个目录 `outputs/<实验名>/`：

```text
index.html          交互查看入口
viewer_data/        自动导出的浏览器数据，按场景加载
viewer_sources.json 主入口登记的附加实验；只有统一查看入口使用
viewer_runs/        附加实验的相对目录链接；只有统一查看入口使用
videos/             特写 MP4 与一个索引
report/             报告、图表、诊断与环境记录
scenes/             规划点云、初值、独立验收表面
trials/             各方法轨迹、逐次记录与验收
source/             运行时源代码快照
replay/             早期实验的补录缓存；新实验通常无需此目录
protocol.json       实验协议
results.json/csv    汇总指标
```

`outputs/` 为生成内容，未加入版本管理。旧版松散文件保存在 `archive/previous_layout.zip`，
后续只维护上面的代码和输出结构。

## 验证与当前边界

```bash
python -m unittest discover -s m20_3d_planner/tests -v
```

数值测试仅需 conda 环境；浏览器测试另需 `pip install playwright` 和系统 Chrome，
检查离线打开、统一查看器全部 60 条轨迹的旋转矩阵、导入来源与报告/视频、拖拽/缩放/平移、图层和播放。
没有这些可选依赖或尚未导出结果时，浏览器测试跳过。

统一直线初值下，XYZ/XYZ+yaw/6D 达到严格 20 mm 间隙的场景数为 12/17、13/17、14/17。
30° 通道会主动侧倾，但节点间仍有 1.27 mm 穿透。详见 [实验报告](outputs/unified_init/report/REPORT.md)。
这仍是离线合成点云与机身几何验证，未验证腿部、接触、动力学和实机跟踪。
两个非平行新场景中三种方法的密集复检间隙均达到 20 mm，6D 最大 roll 分别为 29.64°、1.60°。
下方折面场景主要向水平一侧横移 0.16 m，并不能证明侧倾不可或缺；6D 两组均达到 300 次迭代上限。
6D 的保守连续间隙下界分别为 14.84 mm、5.26 mm，因此密集复检达标不等于连续 ≥20 mm 已获保证。
默认机器人常量来自 `robot.json` 中明确记录的历史模型参数；也可给 `run` 同时传入
`--urdf` 与 `--m20-config` 从原模型读取，不宣称本机已重新核验缺失的原始模型。

## Body 中心射线筛选

```bash
python -m m20_3d_planner run --output m20_3d_planner/outputs/my_ray_experiment \
  --modes full --point-selection rays --max-length 0.6 --ray-angle-deg 4 --workers 1
python -m m20_3d_planner view --include-run m20_3d_planner/outputs/my_ray_experiment --export-only
```

每次目标评估按当前各节点的 body 中心重新筛选，覆盖整个球面，包含上下方向。
`max-length` 单位为米，从中心算起；先用一次构建的 KD-tree 截取半径内点，
再把相对方向按方位角/仰角分格，每格取距离中心最近的原始点，精确同距取原索引较小者。
分格在世界坐标系中固定；射线起点跟随平移，body 旋转仍通过后续 OBB 距离参与势能与力矩。
这是离散点云的有限角宽首命中近似；没有求点云与零宽度射线的精确相交。

筛选之后沿用原始点坐标、面积权重、47 mm 距离势能及表面力臂。
力不会被改成穿过中心的径向力，力矩仍能引起侧倾。距离越近，原有软惩罚及作用力越强。
不对删掉的点补权，因此首命中删点可能降低碰撞惩罚；命中点切换也会引入目标不连续，
使 L-BFGS-B 提前停止或线搜索困难，效果需要用完整表面验收。

可选 `--point-selection all`（默认，完整点云）、`radius`（仅截取距离）、`rays`（距离＋首命中）。
本机身半对角线约 0.398 m，加 0.047 m 激活距离约为 0.445 m；
`max-length=0.6` 足以包含任意朝向下所有可能产生惩罚的点。
因此 `radius` 在此半径下保留原有目标；`rays` 会额外近似该目标。

本轮对照保存在 `outputs/center_ray_filter/`：完整点云、仅距离、4° 射线、2° 射线。
每组全部 20 个场景、相同点云/初值/权重/边界、六维优化、最多 300 次迭代，单进程依次执行。
`wall_seconds` 包含求解时的筛选开销，`total_planning_seconds` 还包括 KD-tree 构建与初始 wrench；
均不包含点云生成、独立验收和导出。筛选耗时、候选数和保留数另存于每条结果。
验收仍为完整留出表面、每段 50 等分，不能用筛选后的点证明安全。

生成配对对照报告（只读已有结果，不重跑优化）：

```bash
python -m m20_3d_planner compare \
  --runs m20_3d_planner/outputs/center_ray_filter/{baseline_full_cloud,radius_060,rays_4deg,rays_2deg} \
  --output m20_3d_planner/outputs/center_ray_filter
```

统一查看器仍按 20 个实际场景选择，“实验 / 点云筛选”切换原结果及本轮四组方法，
不会把同一场景重复铺满场景菜单。本轮各组只求解 6D，因此相应结果的 XYZ 选项禁用。
本轮汇总见 [筛选效果与时间](outputs/center_ray_filter/REPORT.md)。

实测 20 场景的总规划时间：完整点云 180.97 s，距离截取 48.23 s，4° 射线 69.94 s，2° 射线 94.59 s。
仅距离截取加速 3.75 倍，全部接受迭代位姿及代价值逐元素一致；平均保留 17.8% 的点。
4°/2° 射线平均保留 6.0%/11.0% 的点，但分格首命中增加开销，20 mm 达标数均由 17/20 降到 15/20。
空旷区进入通道的间隙由 29.25 mm 降到 0.95/5.43 mm，虽然两组仍能下降并侧倾。
因此优先推荐 `radius` 作为本轮无损加速方式；`rays` 保留为可选实验，不默认替换旧结果。
上述通过率来自完整表面密集复检，不代表连续 20 mm 保证；每组仅一个固定点云种子、一次计时。

### 只显示当前方案

当前网页只加载 `ray_open_world_collision_v1` 的 12 个任务，只有一个实验时隐藏实验切换控件。
选择保存在查看器根目录的 `viewer_sources.json`，重新导出仍保持该方案。

```bash
python -m m20_3d_planner view --run m20_3d_planner/outputs/ray_newton_euler_v1 \
  --only-run m20_3d_planner/outputs/ray_open_world_collision_v1 --export-only
```
