# M20 3D Point-Cloud Virtual-Wrench Planner

这是一个面向 DeepRobotics M20 的离线几何验证套件：从三维点云直接计算机身虚拟力与
虚拟力矩，并优化连续的 6D `base_link` 参考轨迹。

## 当前验证结果

固定参数盲测包含 17 个场景：

- 11 个不同高度、yaw、横向倾角和纵向坡度的低梁；
- 4 个 10–35° 倾斜通道；
- 2 个斜梁、不同高度悬障与错位立柱组成的非对称三维场景。

独立密集表面复检结果：17/17 无穿透，14/17 达到 20 mm 安全间隙。三个未达到
安全目标的场景是 20°、30°、35° 倾斜通道，最小间隙分别为 12.1、5.6、16.4 mm。
因此核心 virtual-wrench 方法能产生无碰撞轨迹，但当前非光滑优化器的安全余量还不够稳定。

规划点云使用 M20 MJCF 中的前后雷达位置，并模拟角度栅格遮挡、距离噪声、7.5% 丢点、
离群点、半径滤波、20 mm 体素化以及沿路径 14 帧融合。密集真值表面只在规划完成后
评分，不参与调参、重试或轨迹选择。

关键结果和可视化位于 [`artifacts/`](artifacts/)。

## 运行

```bash
python -m pip install -r m20_3d_planner/requirements.txt

python m20_3d_planner/run_3d_suite.py \
  --urdf /path/to/deep_robotics_model/M20/urdf/M20.urdf \
  --m20-config /path/to/deeprobotics_m20/rough_env_cfg.py \
  --output m20_3d_planner/results
```

也可以设置 `M20_URDF` 和 `M20_CONFIG` 环境变量后省略对应参数。生成结果包括每个场景
的规划点云、独立真值表面、6D 轨迹、三维图、批量 JSON/CSV 和代表性 MP4 动画。

## 文件

- `m20_core.py`：M20 URDF 机身包络、点到 OBB 的带符号距离、virtual wrench、轨迹优化与独立复检。
- `run_3d_suite.py`：场景生成、M20 前后雷达点云模拟、17 场景批量实验及可视化。
- `artifacts/REPORT.md`：本次固定参数盲测报告。
- `artifacts/suite_results.json`：完整机器可读指标。

## 边界

这里的点云是具有传感器误差模型的**仿真点云**，不是 M20 实机 rosbag/PCD。当前仅验证
机身几何参考轨迹，尚未加入四轮接触、腿长与关节限位、动力学和控制跟踪；不得直接将
轨迹下发到实机。
