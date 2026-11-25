# FSGP-BGK：CPU 核心方法复现

## 当前首页：十八场景、凸包显示、六项指标

统一入口 **http://127.0.0.1:42482/**。保留丘陵、森林、平地、沙漠、山地、雪地、火山和高低组合多级台阶，新增“平地 · 通道与高低台”和“A* 初版 · 矮台、立墙与顶板”，恢复大桌子，并增加净空 40／42／60 cm 的独立低桌及封闭通道降身／上桥、第二台下方净空 70／42 cm 的延伸场景及五平台通行链，共十八场景。方法为局部二次曲面，移除组合曲面及 FSGP 选择；指标仅为上方占用、平均法向坡度、拟合 RMSE、不确定性、法向变化平滑性、最大拟合残差。

拟合后直接以合格点的精确 XY 凸包作为显示域，全部有效凸包完整显示，不再按附近观测删除显示面。显示细分与指标采样分开；上方占用仅用原始点，法向与不确定性独立从函数计算。默认自适应扩展至全部十八场景；固定 R7 和两种方窗保留作对照。72 组基础数据统一规则、单位与白到红色标。

[新增场景直接查看](http://127.0.0.1:42482/?scene=flat-corridor-platforms&profile=qcircle-adaptive&coverage=1&points=1)：8×8 m 平地；通道长 4 m、净宽 2 m、净高 0.60 m；矮台高 0.15 m，高台高 0.80 m，均为实心平台，各 1.6×1.4 m。约 2.5 cm 点间距，156,743 个构造表面点；四种设置均从 XYZ 重新运行，未使用部件标签。[记录与耗时](../result.md#12-平地新增通道与高低台2026-11-21)。

[森林直接查看](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive&coverage=1) · [当前公式与结果](../result.md#11-当前首页八场景凸多边形显示与六项指标) · [详细报告](outputs/visualization/convex-assessment/report.md)。原有历史结果和独立页面保留，以下旧章节不再全部对应当前首页。

生成当前首页：`/usr/bin/python3 -m fsgp_bgk_repro.convex_surface_assessment`，然后运行 `convex_assessment_report` 和 `publish_metric_viewer`。本机需 `PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`。服务入口仍是 `serve_visualization.py`，端口 42482。

2026-11-22 森林另提供 [取消中心预测偏移上限的对照](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive-no-offset&coverage=1)。保留邻域高度筛选、至少 15 点、秩与 RMSE≤8 mm 检查；有效函数 4,666→4,760，固定种子 5,000 个已观测点支持率 71.06%→71.40%，不是独立精度或面积覆盖率。[完整说明](../result.md#4-森林取消中心预测偏移上限2026-11-22)。用 `/usr/bin/python3 -m fsgp_bgk_repro.forest_center_offset_ablation` 生成，再发布查看器。只在森林增加此选项，原四基础设置与结果保留。

所有当前场景均已同步 [补齐小缝（<9 cm²）](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive&coverage=1&gaps=1) 显示开关，共十八场景×四基础设置，加森林门限对照，共 73 组。只补原凸包投影并集内部面积严格小于 9 cm² 的完整封闭空档，较大空档及外侧敞口保留，不继续拟合。统一检查边界预测高度兼容（≤4 cm），找不到兼容父函数时整块小缝保留；连续陡坡不按总高差否决。原始点、有效函数与六指标不变。补缝是原函数的显示外推，不是新增实测覆盖；各阶段的数据保全范围见实验记录，本轮规划核对原 61 组的 915 个文件不变。生成：`/usr/bin/python3 -m fsgp_bgk_repro.small_gaps`，然后 `publish_metric_viewer`；使用系统 GEOS C 库，不依赖 Shapely。原丘陵独立结果保留。[计算与验证](../result.md#all-small-gaps)。

新增 [邻层边界高度差图](http://127.0.0.1:42482/?scene=flat-corridor-platforms&profile=qcircle-adaptive&coverage=1&boundary=1)：高低台通道与多级台阶的四设置，图中勾选“边界高度差”显示白→红色线，点选显示两侧函数编号与 Δz。仅比较共同边界 XY 的预测，不是中心高差；默认矮台／地面约 15 cm、高台／地面约 80 cm。没有新增拟合或预设层数，六指标和小缝补偿保留。[方法与剖面图](../result.md#boundary-heights) · [生成命令](../step.md#boundary-heights-reproduce)。


## 当前分层投影 A*（更新 2026-11-24）

[高低台对比](http://127.0.0.1:42482/surface-planner/?scene=flat-corridor-platforms) · [丘陵滚动](http://127.0.0.1:42482/surface-planner/?scene=hill) · [简明流程](../planning-workflow.md) · [当前结果](../result.md#layer-projection-planning)。全部沿用 42482。

先评价落足点，单足代价为坡度／平滑性／RMSE／最大误差／不确定性，权重 0.30／0.20／0.15／0.10／0.10。按拟合域邻接及同 XY 预测一致性分成连续层；同层只滚动，不用跨步高差触发抬腿，也不以摆腿兜底滚动失败。

先在起始层运行 A*，再把交接边界高差严格小于 40 cm 的可达层投影叠加，重新运行 A*。保留真实 Z、函数和层编号，只有观测交接处允许换层。40 cm 是用户设定；跨层连接仅为规划候选，尚未生成或验证跨层动作。机身／足点的观测支持、软 IK、上方通行、碰撞和准静态 ZMP 检查保留。

在路径选项切换本层／叠加层，“投影代价图（XY）”显示搜索图；紫色虚线表示跨层候选。复现见 [step.md 第 27 节](../step.md#layer-projection-reproduce)，[逐场景耗时](../surface-planning-v1.md)。旧查看器与完整数据独立保存在 `outputs/visualization/surface-planner/history/before-layer-projection-20261123/`。

新增第二台空心延伸对照：[70 cm 净空](http://127.0.0.1:42482/surface-planner/?scene=platform-underpass-070)、[42 cm 净空降身](http://127.0.0.1:42482/surface-planner/?scene=platform-underpass-042)。按机器人初始前方 +X、右侧 −Y，保持前后 X=[1,2.6] m，平台右沿延伸至 Y=−4 m；目标在前方出口外；前者平台下中心离地 40 cm，后者降至 32 cm、驶出后回到 40 cm。顶板已有拟合，默认蓝色显示，勾选“透视上层”查看下方。0/1 按同 XY 最近上层的净空≥41.5 cm 初筛，再检查实际机身碰撞与 IK/ZMP。原场景及 915 个拟合数据文件保留，新增 8 组拟合与六指标。[流程及范围](../planning-workflow.md#7-第二个平台下方通行测试)。

[五平台全程规划](http://127.0.0.1:42482/surface-planner/?scene=four-platforms)：20×8 m场地，顶高／净空依次80 cm实心、80/70、52/42、30/20、78/38 cm。2、3、4横跨全宽，5留1.2 m小路；只保留全程本层／叠加查询，结果由A*自由搜索。其他69组拟合不变，替换本场景四组，旧四平台实验归档。[流程](../planning-workflow.md#9-五平台通行链只保留全程规划)。

## 源码同步与复现（2026-11-25）

本轮同步包括圆形／方窗拟合、凸包裁剪与六指标、补缝和边界高度差、分层 A*、五平台场景、前端、测试及实测报告。[完整流程](../planning-workflow.md)、[结果](../result.md)、[五平台及分项耗时](../surface-planning-v1.md)为当前说明；下方历史章节保留旧实验。

仓库保留选定的汇总、核查记录、截图和分项计时数据。新生成的点云、拟合二进制、逐帧路线及历史网页数据不整体上传；首次检出需先生成数据。[step.md 第29节](../step.md#29-五平台通行链2026-11-24)给出五平台生成、拟合、规划、扩大预算、核查和发布的顺序，其他场景的生成入口见其前面章节。页面统一使用 **42482**。

拟合依赖 `requirements.txt`；补缝另需系统 GEOS C 库。M20 姿态／重心检查还需 MuJoCo 及机器人 XML 和 meshes，默认读取 `/home/jiang/m20-repro/src/rl_training/deep_robotics_model/M20/mjcf/M20.xml`；异机目录不同需调整 `m20_policy.py` 中的 `EXTERNAL`。盲走能力复测另外需要该文件指定的策略检查点，规划本身不加载策略权重。外部模型与检查点未随本轮源码同步。2026-11-25 上传前重新运行全部 **128 项 Python 测试，通过**。

## 历史圆形与完整指标实验（2026-11-21）

统一首页默认提供半径 7 cm 圆形拟合：中心约 11.5 cm、至少 15 点全部参与、有限重叠，新增补中心最多 5%。原有 17 场景可选，20 cm、15 cm 方窗及 FSGP 历史选项保留。按用户要求已移除路径规划、相关结果与两个新增规划场景。

- [统一首页](http://127.0.0.1:42482/) · [多级台阶](http://127.0.0.1:42482/?scene=steps&profile=qcircle-r07&coverage=1) · [丘陵](http://127.0.0.1:42482/?scene=hill&profile=qcircle-r07&coverage=1) · [原有大桌子](http://127.0.0.1:42482/?scene=table&profile=qcircle-r07&coverage=1)。
- [圆形拟合方法与指标](../result.md#8-圆形-r7-cm-拟合2026-11-21) · [实验说明](../step.md#14-圆形-r7-cm-拟合2026-11-21)。
- 圆形与两种方窗均计算 11 项指标，17 场景×3 设置：[三组指标、已知率与计时](http://127.0.0.1:42482/assessment/circle-square-comparison/)。圆形补齐 A/B/C、占用、落足和机身评价；方窗重新计算并核对历史结果。选择指标自动显示白到红，未知保持灰色。

沙漠／山地／雪地／火山在前端自动模式下默认使用“自适应圆形”（手动旧设置和历史链接保留）：按密度与局部残差在 R=7–15 cm 中选择，至少 15 点、目标 24 点，保留独立函数与真实观测支持约束。原始观测点支持率分别提高至 99.96%／95.00%／94.50%／99.06%（固定种子抽样，不是独立精度或面积覆盖率）。[直接看沙漠](http://127.0.0.1:42482/?scene=desert&profile=qcircle-adaptive&coverage=1) · [四场景对照](http://127.0.0.1:42482/assessment/adaptive-circle/) · [详细方法](../result.md#9-四个稀疏地形的自适应圆形拟合2026-11-21)。

论文：[Real-time Spatial-temporal Traversability Assessment via Feature-based Sparse Gaussian Process](https://arxiv.org/abs/2503.04134v2)，IROS 2025。
官方实现：[ZJU-FAST-Lab/FSGP_BGK](https://github.com/ZJU-FAST-Lab/FSGP_BGK)。

已打通 **点云 → 曲率/高差特征 → 特征点采样 → 稀疏地形 GP → 坡度和不确定性 → 时空 BGK → 代价地图**。
这是按论文 v2 公式构建、可独立运行的 CPU 参考实现；没有直接执行原版 PyTorch/CuPy GPU 链路。
现有 M20 规划代码不依赖本模块。

已运行结果：[完整报告](outputs/cpu/REPORT.md) · [丘陵地图](outputs/hill_map/map.png) ·
[丘陵方法对照](outputs/cpu/hill_m125.png) · [逐帧指标](outputs/cpu/per_frame.csv) ·
[结果复核记录](outputs/cpu/audit.json)。

## 当前点云实验与 FSGP 对比（2026-11-17）

新增严格 20×20 cm 局部二次拟合、三种边界跳变指标、10–60 cm 净空、
50×80 cm 机身几何评估，以及 Q20 与本地 FSGP CPU 参考的 81 组配对测评。

- [方法、计算公式和 17 场景结果](../step.md)
- [Q20 / FSGP 效果测评与限制](outputs/visualization/fsgp-q20/report.md)
- [共同支持误差、覆盖率、危险排序与计时总览](outputs/visualization/fsgp-q20/comparison.png)
- [逐场景／逐图耗时记录](outputs/visualization/assessment/timing/report.md)

当前仓库保存源码、测试和选定的实测报告；新增的大型点云、NPZ、渲染二进制和逐例网页数据
由脚本生成，不随本次源码提交。已有历史数据继续保留。**首次检出此版本，需要先生成新场景和查看数据，
再启动下方的 42482 服务**；只有报告文件不足以运行所有交互窗口。所有新旧查看器均沿用 42482。

```bash
cd /path/to/go2
# Python 需要 requirements.txt 中的 NumPy、SciPy、Matplotlib。
# 本机可用 /usr/bin/python3，其他机器应使用已装齐依赖的 Python。
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib

# 单个大场景、三个独立通道、多级台阶、新增平地障碍，随后生成拟合与评估。
/usr/bin/python3 -m fsgp_bgk_repro.composite_scene
/usr/bin/python3 -m fsgp_bgk_repro.corridor_scenes
/usr/bin/python3 -m fsgp_bgk_repro.mixed_steps_scene
/usr/bin/python3 -m fsgp_bgk_repro.flat_obstacles_scene
/usr/bin/python3 -m fsgp_bgk_repro.scene_fit_catalog
/usr/bin/python3 -m fsgp_bgk_repro.assessment_export
/usr/bin/python3 -m fsgp_bgk_repro.assessment_profiles
/usr/bin/python3 -m fsgp_bgk_repro.circular_surfaces
/usr/bin/python3 -m fsgp_bgk_repro.convex_surface_assessment
/usr/bin/python3 -m fsgp_bgk_repro.convex_assessment_report
/usr/bin/python3 -m fsgp_bgk_repro.publish_metric_viewer

# 9 场景 × 3 噪声／密度 × 3 种子，含 8 帧时序小实验。
/usr/bin/python3 -m fsgp_bgk_repro.compare_fsgp_q20
/usr/bin/python3 -m fsgp_bgk_repro.compare_fsgp_q20_report
/usr/bin/python3 -m fsgp_bgk_repro.plot_fsgp_q20
/usr/bin/python3 -m fsgp_bgk_repro.audit_fsgp_q20

# 当前完整 Python 单元测试：128 项。
/usr/bin/python3 -m unittest discover -s fsgp_bgk_repro/tests -v

# 服务若已在运行，直接刷新原页面，不再启动第二个服务。
/usr/bin/python3 fsgp_bgk_repro/serve_visualization.py
```

访问 `http://127.0.0.1:42482/`、`http://127.0.0.1:42482/assessment/` 或
`http://127.0.0.1:42482/fsgp-q20/`。仓库中的耗时与截图是本机历史实测，
重新生成后的时间会因机器和运行条件变化；训练残差与独立测试误差分别记录。

## 立即运行

所有场景的统一拟合与可视化入口：**http://127.0.0.1:42482/**。今后沿用此端口。
首页为简洁的单图评价界面：选场景、选指标，直接查看白→红曲面及全场景平均值、最大值和指标已知比例；点击曲面可查看实际局部函数数值。低值白色、高值红色、未知灰色，色标明确显示单位和上限。

首页“评价方法”现可选择 **局部二次曲面 / FSGP · M=125 / FSGP · M=500**，全部 17 场景使用相同的保存 XYZ。FSGP 显示独立计算的 GP 查询曲面，提供可通行性代价、未平滑代价、坡度、曲率、局部高差和 GP 标准差；切换方法保留视角，并自动关闭蓝色覆盖模式以显示白红代价。直接查看 [原单级台阶 FSGP-500](http://127.0.0.1:42482/?scene=steps-single&method=fsgp-500&metric=gp_cost&coverage=0) 或 [多级台阶 FSGP-125](http://127.0.0.1:42482/?scene=steps&method=fsgp-125&metric=gp_cost&coverage=0)。

本页使用已有论文公式 CPU 参考，在 5×5 m 局部窗口内计算 20 cm 栅格；当前是单帧静态点云，未执行 BGK 历史融合，也不等同于原版 GPU 优化训练。代价越大越难通过，保留原始加权值，不限制为 0–1；单层 GP 代价不包含二次方案的机身净空和落足模型。详见 [方法、参数与限制](visualization/metrics/fsgp-method.md)。生成 FSGP 与发布新选项：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.export_fsgp_viewer
PYTHONNOUSERSITE=1 /usr/bin/python3 -m fsgp_bgk_repro.publish_metric_viewer
```

“拟合设置”提供 **20×20 cm／至少 30 点** 与 **15×15 cm／至少 15 点** 两组，窗口内全部合格点参与。两组 17 场景使用相同原始 XYZ、中心体素与编号，高度带保持 ±4 cm，机身保持 80×50 cm；切换加载各组真实模型与面片，CSV 和分享参数同步。直接查看 [森林 15 cm](http://127.0.0.1:42482/?scene=forest&profile=q15-n15&metric=jump_b_m&coverage=1) 或 [两组设置对照](http://127.0.0.1:42482/assessment/profile-comparison/)。森林有效函数 3,958→11,845（16.2%→48.3%，按中心数），但外推极值和不确定性也增加，不能据此声称精度提高。

支持突变 A/B/C、坡度、粗糙度、拟合残差、不确定性、上方占用、落足与机身评价，共 11 项；切换指标使用同一批已保存结果，不重新拟合。默认展示严格 20 cm 独立面片，可直接勾选组合曲面或原始点。勾选“拟合覆盖（蓝色）”可直接区分实际有效面（蓝色）、未拟合中心（橙色）和有效但当前未成面的中心（紫色）；白红模式中的白色实体面表示低指标，背景色空隙才表示没有显示曲面。图下显示各类中心数量；点击面片描出该函数实际三角形。

通道和桌子默认显示全部高度，手动选择“仅看底部 10 cm”才裁剪显示；统计仍包含全场景有效函数。默认 20 cm 森林的 24,500 个中心中 19,995 个因点数不足未求解、547 个因 RMSE 超过 2 cm 被拒绝；3,958 个有效函数中 28 个裁剪后无独立三角面。详细归因见 [实验记录](../experiment-results.md) 与 `outputs/visualization/metrics/fit-coverage-audit.json`；[指标与计算方法](../result.md) 单独保留，15 cm 的独立统计在 `assessment/profile-comparison/`。

原八窗口工作台保留于 `http://127.0.0.1:42482/assessment/details.html`，历史 30 近邻查看器保留于 `http://127.0.0.1:42482/scene-fits/`。历史查看器仍使用 0.12 m 体素、30 近邻与 0.75 m 搜索半径，其数据与新首页的严格 20 cm 方案分开。旧首页带 `color`、`surface` 等参数的深链接自动进入原查看器。

```bash
# 启动 / 重启统一页面（安装好的用户服务，用户登录后自动启动）
systemctl --user start go2-pointcloud-viewer.service
systemctl --user restart go2-pointcloud-viewer.service

# 也可在终端直接运行，使用相同固定端口（服务已运行时无须重复启动）
/usr/bin/python3 fsgp_bgk_repro/serve_visualization.py

# 从完整点云重新测试全部 17 个场景
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.scene_fit_catalog
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_scene_fit_catalog
```

[全景汇总 CSV](outputs/visualization/scene-fits/summary.csv) ·
[全部场景计算校验](outputs/visualization/scene-fits/catalog-audit.json) ·
[全部场景浏览器验证](outputs/visualization/scene-fits/browser-audit.json)。
原首页完整保留于 [历史页面](outputs/visualization/legacy-results.html)。

[拟合失败与组合缺口统计](http://127.0.0.1:42482/scene-fits/fit-assembly-stats/) 按局部函数统一计数：
未得到有效拟合、有效但完全未进入最终三角面、至少参与一个三角面的函数。
最后一类另外统计“仍有邻接三角形被删除”的数量，避免漏掉部分拼接失败形成的白块。
统计从当前保存结果读取，不更改拟合或连接规则，可用以下命令重新生成：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.summarize_fit_assembly
```

在仓库根目录执行，无需下载模型、安装 PyTorch 或启动 ROS：

```bash
# 校验 9 份官方源码/配置和 2 份官方点云
bash fsgp_bgk_repro/run_cpu.sh verify-sources

# 官方 200000 点 hill.pcd，截取局部扫描生成地图
bash fsgp_bgk_repro/run_cpu.sh map \
  --input fsgp_bgk_repro/data/hill.pcd \
  --center 0 0 --inducing 125 \
  --output fsgp_bgk_repro/outputs/hill_map

# 完整诊断：5 场景 × 3 个 M × 8 帧，3 种方法，共 360 行指标
bash fsgp_bgk_repro/run_cpu.sh benchmark \
  --config fsgp_bgk_repro/configs/paper_cpu.json \
  --output fsgp_bgk_repro/outputs/cpu

# 只跑一组
bash fsgp_bgk_repro/run_cpu.sh benchmark \
  --scenes hill --inducing 125 --frames 8 \
  --output fsgp_bgk_repro/outputs/hill_only

# 几何、导数、坐标对齐、历史过期、文件格式等检查
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest discover -s fsgp_bgk_repro/tests -v

# 从保存的地图重新计算末帧指标、检查输入公平性及汇总数值
PYTHONNOUSERSITE=1 /usr/bin/python3 -m fsgp_bgk_repro.audit_results \
  fsgp_bgk_repro/outputs/cpu
```

脚本使用本机已有 `/usr/bin/python3`，隔离用户 site-packages，并固定 BLAS 为单线程。
实测依赖为 Python 3.10.12、NumPy 1.21.5、SciPy 1.8.0、Matplotlib 3.5.1。
默认 `python3` 是缺少 SciPy 的 Conda Python 3.13，因此提供上述启动脚本。
其他机器可在独立虚拟环境安装 `requirements.txt`，设置 `FSGP_PYTHON=/path/to/venv/bin/python`。

## 已完成与未完成

| 内容 | 状态 |
|---|---|
| 特征提取、IDW、诱导点预测、解析 GP 导数、BGK、空间平滑 | 已实现、已测试 |
| 官方 hill/quick 点云 | 已随目录提供、SHA-256 固定 |
| SGP-XY / FSGP / FSGP-BGK 对照，M=50/125/500 | 已运行 |
| 平面、斜坡、台阶的独立几何诊断 | 已运行 |
| 官方完整 2500 场景与论文 Table I–III 指标 | 尚未复现；原始场景/标签/批测程序未完整发布 |
| 原版 GPU 链路与论文实时性能 | 尚未验证；本机 NVIDIA 驱动不可用 |
| A* + MINCO + MPC 导航及实车实验 | 尚未复现；官方仓库也将规划模块标为待发布 |

本次诊断中 BGK 在 15 个场景/M 配置中有 9 个降低了相对 FSGP 的 MAE。
所有结果均保留，不能把这些本地数据解释为论文精度提升幅度的验证。

## 交互可视化

浏览器入口为 `outputs/visualization/index.html`。在仓库根目录启动查看服务：

```bash
python3 -m http.server 42482 --bind 127.0.0.1 \
  --directory fsgp_bgk_repro/outputs/visualization
```

然后访问 **http://127.0.0.1:42482/**。顶部选择场景与诱导点数量，拖动点云旋转；
向下滚动可查看四张代价地图和误差曲线。误差曲线需要加载图表组件。
场景下拉框还提供「大桌子（桌下空旷）」：展示按地面、桌面和四条腿分层抽样的 6000 个表面点，
可点击「桌底视角」检查净空。已生成桌子评测后，同一场景可查看三种方法的地图和逐帧代价偏差；
首次仅生成场景、尚无评测输出时，界面只展示点云。

对话中提供了五场景交互视图：可旋转三维点云、切换局部扫描/完整地形、显示诱导点，
并对照几何参考、SGP-XY、FSGP 和 FSGP-BGK 的代价地图与八帧误差曲线。
地图对应第 8 帧，全部 25×25 栅格值保持原始精度；各方法及 M 在同一场景内共用色标。
三维显示最多取 900 个扫描点或 1200 个全景点，仅用于显示。

已有实验结果变化后，可重新生成对话视图源：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.visualization.export
```

生成器仅读取已有 NPZ/CSV，不重新执行 GP 或修改评测结果。

丘陵场景的坡度与平均绝对高差相关性分析：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.analyze_feature_correlation
```

结果位于 `outputs/visualization/hill-feature-analysis/`，包含空间分布/散点图、
逐格数据及统计与来源记录。分析以 M=125 的第8帧为主，并在共同有效区域比较不同 M。

### 大桌子悬空场景

浏览器访问 **http://127.0.0.1:42482/large-table/**，可旋转、切换桌底/侧面/俯视、
切换点云或实体、分别隐藏桌面/桌腿/地面，并下载 PCD 和 NPZ。
桌面 3×2 m、总高 1.2 m、厚 0.1 m，桌下净高 1.1 m；四条桌腿截面 0.12×0.12 m。
地面 6×6 m，桌下无横梁、围挡或其他物体。沿 X/Y 的两腿内侧净距分别为 2.52/1.52 m。

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m fsgp_bgk_repro.table_scene
```

输出位于 `outputs/visualization/large-table/`。`large_table.pcd` 为带 label 字段的二进制点云，
`large_table.npz` 含 `points` 与 `labels`，0=地面、1=桌面、2–5=四条桌腿。
表面采样间隔不大于 4 cm；`scene.json` 保存尺寸、点数、PCD 哈希和桌下净空校验结果。
这是完整、无噪声的多层表面模型，没有模拟扫描遮挡。
它保留同一 XY 下的桌面、桌底和地面，不能直接用单层高程图表示其全部几何。

### 桌子上的论文方法实测

```bash
# 保留所有高度层，M=50/125/500，各 8 帧，并运行无噪声的完整几何/仅地面对照
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.evaluate_table_scene

# 独立重算 72 张逐帧地图及桌下统计，验证多层歧义和无噪声地面对照
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_table_scene

# 将桌子结果与已有五场景合并导出；不重新执行 GP
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.visualization.export
```

[桌子实测报告](outputs/table_cpu/REPORT.md) · [高程/坡度/高差/代价图](outputs/table_cpu/table_m125.png) ·
[无噪声剖面对照](outputs/table_cpu/controls.png) · [完整复核](outputs/table_cpu/table_audit.json)。
本实验不修改 `core.py` 和原五场景协议。参考是解析的平坦地面零代价，排除与桌腿相交的栅格，
不是论文官方真值，也不把连续代价阈值化为碰撞成功率。
M=125 的桌下内部地面（真实 z=0）预测高程中位数约 0.881 m、坡度比均值约 1.492。
完整无噪声输入仍有混淆，仅地面无噪声对照为零；中心三层的 `(x,y,kappa,g)` 相同而目标 z 不同，
详细数值保存在 `multilayer_ambiguity.json`。BGK 融合二维代价，没有新增桌下净空表达。

## 方法与约定

对应论文 arXiv v2 的公式编号：

| 公式 | 实现 | 约定 |
|---|---|---|
| (1) | `core.extract_features` | 3D KNN 含自身；最小特征值/总特征值为曲率；平均绝对高差为 g |
| (2) | `core.select_inducing` | 特征点与非特征体素代表点取并集，超过 M 后随机截取 |
| (3)–(7) | `core.SparseTerrainGP` | 4D 输入 `(x,y,kappa,g)`；XY-IDW；保留 M 点条件化预测 |
| (8) | `TraversabilityMapper.assess` | 原始曲率、高差、GP 的 XY 偏导范数组合；cost 越大越难通过 |
| (9),(10) | `core.bgk_update` | 同一世界栅格内融合，使用旧方差置信度和指数时间衰减 |
| (11) | `core.smooth_known` | 真正二维高斯滤波，按有效邻居归一化；未知区域保持 NaN |

模型选择和论文未完全指定的参数都写在 `configs/paper_cpu.json`：

- 核采用官方源码使用的 Rational Quadratic，固定长度尺度 0.6 m、alpha=10；不做边际似然优化。
  常数均值取保留点高程均值，观测噪声方差 0.0004 m²，Cholesky jitter 为 1e-8。
- 公式 (6),(7) 本质上是对 M 个保留观测进行 GP 条件化。本实现遵循这些公式，
  **没有实现完整变分 SGP，也不等同于原版 ExactGP + InducingPointKernel 的训练过程**。
  建模 O(M³)，查询 O(QM²)，按批查询以控制内存。
- PCA 只在训练输入拟合一次，预测复用同一变换，导数转回原始 XY。
  保留全部主成分；对当前各向同性核，旋转不改变核距离，因此不会凭空提高拟合精度。
- 曲率和高差由当前完整局部扫描插值，未从完整场景参考数据获取。
  GP 坡度使用固定 kappa/g 时的 XY 偏导，与原版自动求导输入含义一致；未加入 IDW 特征的空间导数链。
- 三个评分权重各 1/3，不做逐帧 min-max 缩放，也不将 cost 冒充校准概率。
  `slope` 是坡度比 `|dz/dxy|`，不是角度；如需要角度可取 `arctan(slope)`。
- 当前有效观测需同时满足 XY 最近点距离 ≤0.45 m 和潜在 GP 方差 ≤0.8。
  这是额外覆盖约定，避免把没有证据的插值当作自由空间。
- 历史按绝对世界栅格 ID 保存；移动窗口外或超过 60 s 的观测淘汰。
  仅有历史的单元保留原观测时间，缺少新数据不会刷新其时间戳。
- 公式 (10) 的方差用于后续启发式置信度，不是经过代价非线性传播的严格贝叶斯后验。

## 输入、输出与接入

`map` 支持二进制/ASCII PCD、N×3 NPY、带 `points` 键的 NPZ、无表头 CSV 或 XYZ。
PCD `binary_compressed` 需先转换为 ASCII 或 binary。忽略附加字段及非有限 XYZ。
输入单位为米，必须已对齐重力方向、处在同一固定世界坐标系；本模块不做 SLAM 或 TF 变换。

`map` 输出：

- `map.png`：高程、潜在方差和通行代价。
- `map.npz`：`xy`、`cell_ids`、高程/方差/坡度/曲率/高差、原始代价、融合代价及覆盖掩码。
- `metadata.json`：参数、输入哈希、实际点数、覆盖率和运行时间。

单帧命令只有一次观测，BGK 不会产生时间融合收益。连续帧可直接调用：

```python
from fsgp_bgk_repro import Config, TraversabilityMapper

mapper = TraversabilityMapper(Config(inducing_points=125))
for points_world, robot_xy, stamp_seconds in frames:
    current = mapper.assess(points_world, center=robot_xy, timestamp=stamp_seconds)
    result = mapper.fuse(current)
    cost_yx = result["cost"]        # (ny,nx)，NaN = 未知
    known = result["known"]        # 扁平，含未过期历史
    observed = result["observed"]  # 扁平，仅当前帧
```

除二维 `cost` 外，各层按 `xy` 的顺序展平；`shape` 给出 `(ny,nx)`。
`height/variance/slope` 属于当前 GP，`fused_variance` 属于 BGK 历史融合，不能混用。
空扫描或局部不足 3 点时 `assess` 抛出 `ValueError`。

## 评测方式

`flat/ramp/steps` 为本地确定性合成地形，`hill/quick` 来自固定官方仓库快照。
每帧从移动局部窗口最多取 2500 点，随机丢弃 15%，加入高程标准差 0.015 m 的噪声。
这是点随机丢失模型，没有模拟真实 LiDAR 遮挡射线。每帧时间间隔为 0.1 s。
对照方法读取相同扫描；改变 M 时也使用相同扫描。SGP-XY 是本地消融对照，未声称实现论文引用的完整 SGP 基线。

诊断参考由完整无噪点云的曲率/高差及局部 PCA 法向坡度形成，平面、斜坡用解析坡度。
参考计算不使用 GP；真实官方场景没有提供 cost 标签，因此称为“几何参考”，不是作者 ground truth。
各方法在相同当前支持区域评测；历史额外覆盖率单列。
汇总 MAE/RMSE 按有效格子数汇总，不平均各帧 RMSE。
报告中耗时为该 CPU 实现的算法延迟，不能与论文 RTX 4060 的 33.84 ms 直接比较。

## 同一场景的拟合曲面与剖面曲线

打开 **http://127.0.0.1:42482/fsgp-fit/**，原首页顶部也有入口。原页面六种场景全部接入，
并新增连续的“平地—斜坡—高处平地”（坡度0.3，高差0.6m）；组合场景3个M各8帧。
三个同步三维视图显示论文特征点处理、FSGP高程曲面和二次函数对照，滑动Y剖面查看拟合曲线。

论文不包含逐面片二次曲面分割。这里按论文进行20近邻特征提取、阈值分类、非特征体素采样与M点选择；
局部二次函数和全窗单个二次函数都仅作为额外对照，拟合同一组M个保留点，不使用区域生长。
旧场景严格沿用原首页第8帧输入与GP模型；组合地形有解析高程真值，单独报告公共有效格上的误差。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.visualize_regression

PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_regression_fits
```

[测试说明与汇总](outputs/regression_fits/REPORT.md) ·
[组合地形剖面](outputs/regression_fits/flat_ramp-profile.png) ·
[8帧汇总指标](outputs/regression_fits/combined_summary.json) ·
[逐帧指标](outputs/regression_fits/combined_per_frame.csv) ·
[独立复核](outputs/regression_fits/independent_audit.json)。

当前局部二次函数的速度另行实测，包含建树、KNN、矩阵构造、退化检查与求解，并单列原始XYZ特征采样开销：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.benchmark_regression_speed
```

[局部拟合耗时与GP对照](outputs/regression_speed/REPORT.md) ·
[逐次计时、输入哈希与环境](outputs/regression_speed/metrics.json)。
速度比较区分GP仅高程与高程/方差/梯度三种输出；未将不同输出功能的时间差当成同等功能加速。

## 大桌子：30 近邻、高度门限与多层拟合

打开 **http://127.0.0.1:42482/table-layers/**，首页顶部有新入口。
每层先按自动检测的高度峰限制为 ±2/3/4 cm，再取半径 0.75 m 内的 30 近邻；
不足 30 留空。同一 XY 分别保存地面、桌底、桌面，分别建网，避免旧单层高度图的跨层连接。
页面提供同帧 2125 点、完整 32683 点及原 M=125 稀疏输入，含同步三维、剖面、逐层开关和选中近邻。
这是用户要求的二次函数对照扩展，不是 FSGP 原文的分割方法；旧页面和旧速度记录仍对应原 K=20 单层实现。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.table_multilayer
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest fsgp_bgk_repro.tests.test_table_multilayer -v
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_table_multilayer
```

[方法与覆盖率](outputs/table_multilayer/REPORT.md) ·
[真实高度剖面](outputs/table_multilayer/profile.png) ·
[所有输入和门限的结果](outputs/table_multilayer/metrics.json)。

当前多层版本的速度应另行实测（包含重新分层、建树、30 近邻、有效性检查、拟合，另列建网）：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.benchmark_table_multilayer
```

[多层版本耗时与计时边界](outputs/table_multilayer_speed/REPORT.md) ·
[逐次计时、环境与输入源码哈希](outputs/table_multilayer_speed/metrics.json)。

## 不预设三层：原始点的局部二次拟合与组合

打开 **http://127.0.0.1:42482/table-local/**。每个原始 XYZ 点作为种子，半径 0.75 m、
相对该种子高差 ±4 cm 过滤后取 30 近邻独立拟合，再按局部预测与法向一致性连接。
没有全局高度直方图、面数参数或已知面高度。当前同一帧 2125 点自然得到三个主要组件；
保留混合边缘点、拟合失败与网格缺口。页面可切换单个局部函数和组合网格，并放大查看所选面片。
连接阈值等仍是明确的几何规则；这是独立二次拟合实验，不是 FSGP 原文方法。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.table_local_surfaces
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest fsgp_bgk_repro.tests.test_table_local_surfaces -v
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_table_local_surfaces
```

[方法、限制与结果](outputs/table_local_surfaces/REPORT.md) ·
[局部面片与组合图](outputs/table_local_surfaces/surfaces.png)。
测试包含 1、2、4 个平面及两张高度分布重叠的斜面，验证输出数量没有固定为 3。
此实现每个原始点查询一次，工作量与原全局三层 101×101 网格不同，不能套用旧 151 ms。

独立小曲面默认统一蓝色，右侧组合网格按自动组件着色。可选“按最终组件回看前两图颜色”，
仅用于查看后续分组标签；颜色不参与拟合。原始点默认灰色，避免看起来像提前分割。
地面缺口按候选三角形首次失败原因单独诊断；中图每片只绘制 24×24 cm 的局部范围。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.benchmark_table_local_surfaces
```

[当前完整流程耗时、空洞原因与配色说明](outputs/table_local_speed/REPORT.md) ·
[逐次计时与来源](outputs/table_local_speed/metrics.json) ·
[逐组件缺口诊断](outputs/table_local_speed/holes.json)。

每个局部函数的误差与实际 30 点覆盖范围，可在页面“查看每个曲面的误差”中点选，
并切换中图为逐函数 RMSE 着色。训练残差与解析场景几何距离分别展示，未求解项留空；
1964 个已求解函数包含 9 个最终未通过的函数，避免只看筛选后的结果。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.analyze_table_local_errors
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_table_local_errors
```

[逐曲面误差图](outputs/table_local_errors/per-curve-errors.png) ·
[三维逐曲面误差着色](outputs/table_local_errors/error-map.png) ·
[真实候选三角形预测分歧示例](outputs/table_local_errors/triangle-example.png) ·
[全部 2125 个种子误差与邻域范围 CSV](outputs/table_local_errors/per-curve.csv) ·
[误差定义与统计](outputs/table_local_errors/REPORT.md)。
解析几何只用于事后评价，不参与拟合；它是在原拟合邻点 XY 位置计算的最近真实表面距离，
不是独立新扫描测试或对应面的高度误差。误差分析和新图形不改变已计时的原算法结果。

## 丘陵、森林和桌子的大场景

打开 **http://127.0.0.1:42482/combined-scene/**，首页顶部也提供入口。
场景名义范围 **32 × 12 m**，共 **439,501 点**：保留全部 200,000 个丘陵点、
141,068 个森林点和原桌子 32,683 个点，另补中央空地、两条 2 m 过渡带及丘陵边沿地面。
丘陵沿 X 平移 −10 m，森林沿 X 平移 +11 m，原始地形不缩放、不改变高度。
桌子仍为 3 × 2 m、总高 1.20 m，桌下净空 1.10 m；原地面、桌底和桌面三层都保留。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.composite_scene
```

[完整 PCD](outputs/visualization/combined-scene/combined_scene.pcd) ·
[带来源索引的 NPZ](outputs/visualization/combined-scene/combined_scene.npz) ·
[场景来源与变换](outputs/visualization/combined-scene/scene.json) ·
[静态预览](outputs/visualization/combined-scene/overview.png) ·
[保存文件校验](outputs/visualization/combined-scene/audit.json) ·
[浏览器验证](outputs/visualization/combined-scene/browser-audit.json)。

网页完整加载全部点，支持来源或高度着色、分图层隐藏及桌底视角。
这是构造场景，过渡带属于新增几何；未模拟传感器遮挡或噪声，未在该新场景上运行拟合算法。
来源色并非分割标签；过渡带的边界地面估计方法和逐点来源字段见输出目录 README。

## 三个不同净空的通道场景

新增 [0.40 m 通道](http://127.0.0.1:42482/corridor-040/)、
[0.60 m 通道](http://127.0.0.1:42482/corridor-060/)、
[0.80 m 通道](http://127.0.0.1:42482/corridor-080/)。首页和原拼接场景均有入口。
三个通道是独立小场景，只包含通道和 6 × 4 m 地面，沿 X 方向长 4 m、净宽 2 m，
两端敞开，墙厚与顶板厚度均为 0.12 m。丘陵、森林、桌子拼接大场景单独保留一个。
**净高指地面到顶板底面的距离**，上表面高度分别为 0.52 / 0.72 / 0.92 m。
三个独立通道分别有 30,761 / 32,841 / 34,921 点。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.corridor_scenes
```

三个场景的地面逐点相同，只改变通道结构高度。
构造时检查内部和两端开口的空体积、中心多层高度、独立场景部件及范围，
并回读校验 PCD、NPZ 和网页二进制缓冲。统一首页的曲面拟合已使用这三份独立点云重算。
网页提供统一比例尺的截面对照图、相同入口相机、内部视角、顶板隐藏及完整点云下载。
各目录保存 `scene.json`、`audit.json`、`browser-audit.json`，当前未执行拟合或通行性判定。

## 官方源码复核

未修改的官方源码在 `third_party/FSGP_BGK`，固定提交：
`6e5976957734b90f367f8460a6eea6a0fc18c1cc`。
`sources.lock.json` 记录每份文件的上游路径、SHA-256 和字节数，MIT 声明保留。
`fetch_sources.py` 可按哈希恢复快照。

与论文公式不同或值得注意的代码路径：

| 位置（相对于官方 `src/fsgp_bgk/python/`） | 观察 | 本参考实现 |
|---|---|---|
| `f_sgp_bgk.py:190,217` | 启用 PCA 时训练与预测各调用 `fit_transform` | 仅训练拟合、预测复用 |
| `traversability_lib/traversability.py:156` | `kernel(dists * mask)` 将半径外距离变为 0，故其核权重为最大值 | 使用式 (9) 的同格时间融合，再用二维核空间平滑 |
| `traversability_lib/sgp_model.py:32` | 诱导变量直接设为整个 `train_x`，传入的数值 M 未直接使用 | 保留观测数明确限制为 M |
| `f_sgp_bgk.py:177` | 额外追加覆盖整个局部窗口的 225 个恒高虚拟点 | 不添加未观测地形 |
| `traversability_lib/traversability.py:64` | 坡度取整个 4D 特征导数的范数 | 仅取物理 XY 的偏导 |
| `traversability_lib/traversability.py:176` | 历史键使用局部索引，没有与绝对世界栅格一一对应 | 保存绝对世界 cell ID |
| BGK 两条分支、调用方再取 `1-value` | cost/可通行性方向随分支与多次取反而混杂 | 全链路统一 cost 越高越难通过 |

上表来自静态源码复核；没有声称已跑通原版 GPU 程序或测出其效果。
严格运行官方 ROS 2 版本时，需要另行 clone 完整上游仓库，
在匹配 CUDA 驱动的环境配置 PyTorch、GPyTorch、CuPy、Open3D 和 ROS 依赖，
将 `src/simulation_env_ros1/COLCON_IGNORE` 创建后按上游说明构建，
并把 ROS 2 配置中的 `pcd_file_path` 改成实际绝对路径。
当前快照仅包含评估所需文件，不是完整 ROS 仿真工作区。

## 高低组合多级台阶（2026-11-17）

统一入口 `http://127.0.0.1:42482/?scene=steps&view=stairs&color=uniform`。8 × 6 m 地面、3 m 宽台阶、最高 1 m；上升高差 8/18/12/30/10/22 cm，下降高差 25/10/40/25 cm。包含实际踏面、立面和侧壁，约 4 cm 点间距。原单级场景保留为 `scene=steps-single`。

生成与重新测试：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m fsgp_bgk_repro.mixed_steps_scene
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 -m fsgp_bgk_repro.scene_fit_catalog --scenes steps --repeats 3
```

生成代码为 `mixed_steps_scene.py`；页面可切换侧视、最大误差着色，并下载 PCD/NPZ。`outputs/visualization/mixed-steps/` 保存场景元数据、剖面图和浏览器检查结果。几何标签只用于导出说明，不进入 30 近邻拟合或兼容分组。

## 拟合／组合状态对照

“着色”新增 `fit-status`（有效拟合为蓝，其余白）和 `assembly-status`（中心至少参与一个最终三角形为蓝，其余白）。状态对照使用全部有效函数的独立面片并绘制全部中心，不采用 3,000 面片的抽样上限；未求解、无效函数用白色中心表示，不生成假曲面。两种状态只改变颜色，组合状态不表示所有邻接面都被保留。图例提供返回实际组合网格的按钮。

既有结果可运行 `python3 -m fsgp_bgk_repro.export_scene_status_patches` 生成状态面片；新场景导出时自动生成。文件只在进入状态模式时由浏览器加载，原始拟合结果保持不变。

## 简洁评价页面的单独发布与检查

```bash
# 使用同一份已保存 XYZ 运行 17 场景配对测试，校验原结果并独立导出 15 cm 结果。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.assessment_profiles

# 只发布页面，不改点云、模型、评价数值或原始计时。
PYTHONNOUSERSITE=1 /usr/bin/python3 -m fsgp_bgk_repro.publish_metric_viewer
# 可选：从已保存森林／桌子原始点复核拟合与缺面归因，不改实验结果。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m fsgp_bgk_repro.audit_metric_coverage
node fsgp_bgk_repro/tests/metric_viewer_browser.cjs
```

前端源码在 `visualization/metrics/`，发布于同端口首页与 `outputs/visualization/metrics/`。
灰色未知不当作零值；平均值与最大值统计全场景有效函数的有限指标值，已知比例的分母也是有效函数数，不是场景面积。
白红为固定物理范围的线性映射，超过图例上限饱和为红色；颜色不等同经标定的通过概率。

15 cm 新结果在 `outputs/visualization/assessment/profiles/q15-n15/<scene>/`，原 20 cm 目录保留。对照报告与逐阶段耗时在 `outputs/visualization/assessment/profile-comparison/`；每组每场景单次 CPU 测量，训练残差与中心数量不代表独立精度或覆盖面积。净空按局部宽度改为 3×3 个 5 cm 列，法向角点和机身查询支持范围也跟随所选宽度；默认 20 cm 模型及网格已与原数据逐场景核对一致。
