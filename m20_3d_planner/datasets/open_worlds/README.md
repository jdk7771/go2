# 开源场景输入

使用 AWS RoboMaker Small Warehouse World 和 Small House World 的原始 SDF 碰撞模型：

| 世界 | 原仓库 | 固定提交 | 许可原文 |
|---|---|---|---|
| 仓库 | https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2 | `ee0af733315e78432408c3cd98d378ecee5f767c` | warehouse/LICENSE |
| 住宅 | https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros2 | `ff9631ca6d1db9c1ba656498151464b5ab74aafe` | house/LICENSE |

两份许可均为 MIT-0 文本，原文件随资源保留。来源文件和网格逐文件 SHA-256 保存在
`sources.lock.json`；最初下载归档的哈希保存在各世界的 `download_manifest.json`。
这里只需要碰撞模型，不依赖视觉纹理、Gazebo 或 ROS 的安装。

## 复现

```bash
python m20_3d_planner/datasets/open_worlds/fetch_sources.py
python -m m20_3d_planner.open_worlds
```

第一步补齐并校验固定提交的源文件，拒绝覆盖哈希不一致的本地文件；第二步生成
`prepared/*.npz` 和 `tasks.json`。已有源文件不需要重新下载。

8 个任务分别截取原世界中的货架、堆箱、桶组、搬运车、餐椅、低茶几、沙发及柜体附近。
只刚体变换坐标系并裁剪，不缩放、不重新摆放模型，附近原有对象仍然保留。
局部裁剪范围为 x±1.95 m、y±1.15 m、z∈[-0.15,2.05] m，包含当前位姿约束下
机身能到达的完整区域；裁剪不产生封口墙面。

任务在求解前按固定顺序选择：原始网格与中间机身存在至少 5 mm 穿入，起终点的机身
外扩 40 mm 后不触碰网格，且中心不在模型实体内部。不依据求解结果挑选或删除任务。
这不构成对所有任务在固定 23 节点表示下可解的证明。

COLLADA 导入保留原单位（厘米、毫米转换为米）、场景矩阵、polylist/triangles、
世界/模型/链接/碰撞的位姿。曲面球体离散误差单独加入覆盖界。
真值用最长边细分至 12 mm 的三角形顶点采样，整个面到采样点的距离不超过 12 mm。
因此连续间隙验证使用每个任务自己的覆盖半径，不套用旧合成场景的 22 mm 网格假设。

`check_reference.py` 是额外的离线可达性检查：固定姿态的网格路线通过沿坐标轴的扫掠盒验证。
找到的路线仅证明该局部空间存在一条几何参考路线，可能超过 23 个节点；从不交给规划器当初值。
搜索失败不证明无解。其输出保存在 `grid_references/`。
