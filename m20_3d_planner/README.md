# M20 点云机身规划

入口是 [交互式查看器](outputs/unified_init/index.html)：左键旋转、滚轮缩放、右键平移。
可切换 20 个场景、XYZ/XYZ+yaw/完整 6D，显示初始与优化后路径、检查任意位置，
并播放真实优化迭代。正视看 roll，侧视看 pitch，俯视看 yaw；“聚焦机身”放大观察。
橙色虚线与线框表示初始路径/机身，蓝色表示优化结果；金色顶面和红色机头区分朝向。
“查看视频”集中提供六个姿态特写，其他旧视频已归档。

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
python -m m20_3d_planner view --port 42481
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

不传 `--scenes` 会运行全部 20 个场景；筛选场景不改变各场景的原始点云种子。
`--modes full` 可只运行六维方法；默认仍运行三个自由度对照。
新实验直接保存每次接受迭代的位姿，无需重跑即可播放过程。
`replay --run <目录>` 仅用于给早期实验补录中间位姿：逐次代价和最终位姿必须与原记录匹配，
失败时拒绝保留新缓存。当前默认实验的 51 组过程已全部补齐并核验。

## 代码位置

| 文件 | 职责 / 扩展位置 |
|---|---|
| `core.py` | 机身模型、距离、碰撞能量与 wrench、轨迹目标和解析梯度 |
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

位姿约定始终为 `[x,y,z,roll,pitch,yaw]`，单位为米和弧度，
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
