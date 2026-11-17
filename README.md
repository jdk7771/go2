# 3DPlanner

M20 点云驱动的六维机身轨迹规划。原有 Go2 C++ 工程位于 `src/`、`test/`。

新增论文复现：[FSGP-BGK CPU 核心方法、运行命令与边界说明](fsgp_bgk_repro/README.md)，
[已运行的五场景对照报告](fsgp_bgk_repro/outputs/cpu/REPORT.md)。

新增论文复现：[Quadric LiDAR 二次曲面里程计、增量建图与定位](quadric_lidar_repro/README.md)，
[KITTI 04 实测报告](quadric_lidar_repro/outputs/kitti04/REPORT.md)。

新增点云实验：[严格 20 cm 局部二次拟合与机身几何评估](step.md)、
[Q20 与 FSGP 的 81 组配对测评](fsgp_bgk_repro/outputs/visualization/fsgp-q20/report.md)，
[源码检出后生成数据与启动 42482 的方法](fsgp_bgk_repro/README.md#当前点云实验与-fsgp-对比2026-11-17)。

原远端 `step.md` 的机身—轮端联合规划方案完整保存在
[联合规划扩展方案](docs/body-wheel-joint-planning.md)，仍为待实现的设计文档。

## 当前版本：从起点逐步规划

先筛选安全起终点；从起点开始，每轮读取周围 1.4 m 点云，在 0.65 m 局部范围选短期目标，
使用射线虚拟力和牛顿—欧拉积分调整 9 节点短轨迹。检查通过后最多前进 0.10 m，再从新位置规划。
推进指令限制为 ≤10 mm / ≤2°，已走路径只追加、不再变形。

[逐步规划网页](m20_3d_planner/outputs/ray_newton_euler_v1/index.html) ·
[流程与运行命令](m20_3d_planner/README.md#从起点开始逐段规划当前方案)

下面保留此前全程轨迹变形方案的记录。

新增 **`--solver ray_dynamics` 射线牛顿—欧拉版本**：节点直接保存 `(T_SE3,v_world,omega_body)`，
中心射线筛选后计算表面力/力矩、阻尼和线/角加速度，半隐式更新速度与位置，SO(3) 指数更新旋转。
支持 `--wrench-dt 0.01 --wrench-final-dt 0.001 --wrench-schedule-steps 1000` 由粗到细调度，
固定端点并进行 SO(3) 插值的完整表面连续间隙复检；查看器可检查加速度和时间步。
[运行方法与字段说明](m20_3d_planner/README.md)。

当前版本默认限制每节点单步平移 **10 mm**、旋转 **2°**，超限缩小时间步重算。
首轮四场景对照保存在 `outputs/ray_newton_euler_limited/`；加入上限内步长恢复的复跑保存在
`outputs/ray_newton_euler_limited_adaptive/`，网页默认展示后者并可检查实际单步运动量。
复跑中低横梁通过连续 ≥20 mm 检查；入口窄通道下界约 6.43 mm；倾斜通道和立柱仍碰撞。

历史未限步立柱调参版的最终路径曾获得连续 ≥20 mm 证书（下界 29.56 mm），
但首步跳动约 0.61 m，不能作为稳定动力学积分的证据。其 38.22 s 耗时及成功路径独立保留，
不作为加入单步限制后的新结果。
[对比网页](m20_3d_planner/outputs/ray_newton_euler_v1/index.html) ·
[调参报告](m20_3d_planner/outputs/ray_newton_euler_tuned/report/REPORT.md)。

新增实验：[1.40 m 高障碍物绕行网页](m20_3d_planner/outputs/tall_obstacle_wide/index.html)。
可对比虚拟刚体动力学与原 L-BFGS-B、横移 ±0.60 m 与原始 ±0.16 m 边界、三种自由度，以及真实演化过程。
网页默认选择新增虚拟刚体版本；“实验 / 点云筛选”可切回原优化器。
启动本轮网页：

```bash
python -m m20_3d_planner view --run m20_3d_planner/outputs/tall_obstacle_wide --port 42481
```

新增 `--solver virtual_wrench` 直接积分 body 合力/力矩、线速度和角速度，采用箱体惯量及阻尼，
不调用 L-BFGS-B 更新轨迹。4000 步高障碍物实验的三种自由度均未绕开，密集复检仍有约 70 mm 穿透。
网页可检查逐步合力、力矩、动能及虚拟松弛时间。[实现与复现](m20_3d_planner/README.md#虚拟刚体动力学版本)。

原 L-BFGS-B 6D 向 y 正方向绕行，最大横移 0.466 m，密集复检最小间隙 29.89 mm；
300 次迭代尚未正式收敛，连续间隙保守下界 12.59 mm，未取得连续 ≥20 mm 证书。
复现步骤与对照说明见 [开发说明](m20_3d_planner/README.md#高障碍物绕行实验)。

**日常使用：[交互式点云查看器](m20_3d_planner/outputs/unified_init/index.html)。**
支持任意旋转、缩放和平移；初始/最终轨迹叠加；真实优化过程和最终轨迹播放。
六个姿态特写视频也集中在查看器的“查看视频”按钮中。

原查看器现已统一包含 20 个场景：原 17 个对照、两个非平行截面，
以及“空旷区进入、水平地面＋30° 倾斜顶板窄通道”。在同一场景列表中切换，报告随场景切换。
“实验 / 点云筛选”用于在同一场景内切换完整点云、距离截取和 body 中心射线对照。

```bash
conda activate m20-planner
python -m m20_3d_planner view --port 42481
```

已运行的查看器直接刷新即可。重新启动时上面命令优先使用 42481，端口占用时选择空闲端口，实际地址见终端。
导出的 `index.html` 也可直接用浏览器打开，无需联网。

代码结构、运行新实验与扩展方法见 [开发说明](m20_3d_planner/README.md)。
当前结果与局限见 [实验报告](m20_3d_planner/outputs/unified_init/report/REPORT.md)。
筛选的配对耗时与几何效果见 [筛选对照](m20_3d_planner/outputs/center_ray_filter/REPORT.md)。
整理前的代码和旧版视频已集中归档于 `m20_3d_planner/archive/previous_layout.zip`。
