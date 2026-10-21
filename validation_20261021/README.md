# 本地验证说明

日期：2026-10-21。主报告：[三维规划方法调研与验证](../three_d_planning_methods_and_validation.md)。

本目录是简化规划反例、几何筛选及 Isaac Sim 碰撞交叉检查，**不包含完整机器人行走复现，也没有训练或评估 RL**。

## 文件

| 文件 | 内容 |
|---|---|
| `run_checks.py` | 38 个场景、两种搜索方法；摆轮搜索；膝/姿态/摩擦/滚动约束与图连通性反例 |
| `results.json` | 固定随机种子、场景参数、全部返回路径、逐项诊断及计时 |
| `numerical.log` | 数值程序输出的汇总 |
| `checks.png` | 顶棚叠加缺口及摆轮路径图 |
| `isaac_collision_checks.py` | 将轮、膝和旋转机身的几何检查交给独立 PhysX overlap 查询 |
| `isaac_results.json` | PhysX 检查计数与交叉核对结果 |
| `isaac.log` | Isaac 启动与查询日志 |
| `diagnostic_scene.usda` | PhysX 检查使用的静态障碍；不含机器人及参考轨迹可视化 |
| `environment.json` | 本次运行的 Python 和相关依赖版本 |
| `transition_checks.py` / `transition_results.json` | 审计已有 38 条机身路径的轮端参考变化、固定轮向滚动兼容性和逐边锁定支撑 |
| `contact_graph_demo.py` / `contact_graph_results.json` | 四点足玩具模型的完整接触图搜索；单腿换足、锁定支撑机身前移 |
| `contact_graph.png` | 平地接触图解的机身/四足世界坐标轨迹及支撑裕度 |

## 数值实验运行

在本目录运行。需要 Python、NumPy、SciPy 和 Matplotlib；原始运行环境的版本见 `environment.json`。

```bash
OPENBLAS_NUM_THREADS=1 python run_checks.py > numerical.log 2>&1
```

依赖为 NumPy、SciPy、Matplotlib。随机种子为 `20261021`；结果路径应可复现，计时会随机器负载变化。仅一组种子和手工设计的场景分布，不构成泛化统计。

接触连续性检查（先运行 `run_checks.py` 生成 `results.json`）：

```bash
OPENBLAS_NUM_THREADS=1 python transition_checks.py
OPENBLAS_NUM_THREADS=1 python contact_graph_demo.py
```

## Isaac Sim 检查运行

先运行上面的程序生成 `results.json`。需要主机可用的 NVIDIA GPU；当前沙箱不提供 GPU 访问。

```bash
SIM_PYTHON=/path/to/isaacsim-python
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=YES OPENBLAS_NUM_THREADS=1 \
timeout --kill-after=15s 240s "$SIM_PYTHON" isaac_collision_checks.py > isaac.log 2>&1
```

本机原始实验使用 Isaac Sim 6.0.1 的 Conda Python；若该环境遇到 C++ 库链接错误，需按其安装方式将对应环境的 `lib` 加入 `LD_LIBRARY_PATH`。实测结果见 `isaac_results.json` 的 `agrees_with_reduced_geometry`。

## 解读限制

1. 机身搜索只包含平移：A* 使用 `x/y` 10 cm、`z` 5 cm 网格，机身朝向固定。独立的窄通道检查才涉及旋转。
2. 轮近似球体，腿可达性近似距离区间。没有真实关节限位、轮轴姿态、连杆碰撞或连续换脚。逐节点四接触能变化甚至跳变，不能作为行走轨迹。
3. 每腿只选有限候选中的第一个可用点，并检查四接触的水平凸包。这个筛选可能误拒绝其他可行组合，也可能接受真实机器人不可执行的组合。
4. 38 个场景包括 36 个参数扰动场景，以及平地和单台阶两个固定检查；并非 38 种环境类型。台阶是一个 15 cm 高台阶，没有连续楼梯或梅花桩行走。
5. 所谓通过支撑筛选是返回路径的每个节点通过代码中的简化条件；它不是独立机器人执行成功率。完整扫掠检查也仅针对该固定朝向盒体。
6. 摆轮搜索以球和盒障碍建模。除起落点的竖直升降段外，轮心高度至少 12 cm，对 6 cm 半径相当于底部离地至少 6 cm。未施加真实腿可达性、关节构型或机身支撑条件。
7. 膝反例只证明一组 IK 分支可碰撞，不证明其他分支也碰撞。连通性反例是手工定义的抽象图，未构造对应的机器人场景。
8. 摩擦实验是静态线性规划，使用保守摩擦棱锥；不是动态走坡测试。轮侧滑检查是固定轮向/腿相对位置的瞬时约束，不否定抬腿侧移等其他动作。
9. Isaac Sim 仅建立静态障碍、步进初始化物理场景并执行碰撞查询，没有驱动机器人、接触力验证或相机重建。对路径采样点不碰撞不应外推一般连续轨迹安全；本例几何连续性论证另依赖精确 SDF 与网格步长。
10. `transition_checks.py` 中的固定轮向滚动只允许固定偏航、未转向的轮沿 x 运动；相邻路径段可保持原四支撑点，不等于同一四支撑点能保持到终点。它只审计一条已有 body 路线及一种贪心落足，没有自动搜索换足动作。
11. `contact_graph_demo.py` 使用平地两条固定 y 支撑线上的点足，锁轮静止时允许机身前移；单腿换足时机身保持原位，其余三足保持 1 cm 静态支撑裕度。只检查摆腿起终点距离及最大步长，没有腿段或摆动路径碰撞、力学、滚动及目标机器人的关节运动。横贯缺口不能走通仅相对于该动作集合，不能推断实际轮足机器人无法通过。

这些实验主要用于排除架构中的错误假设。主报告的完整规划器、控制器条件化验证和 RL 训练方案尚未实现。
