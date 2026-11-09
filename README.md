# 3DPlanner

M20 点云驱动的六维机身轨迹规划。原有 Go2 C++ 工程位于 `src/`、`test/`。

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
