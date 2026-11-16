# Quadric LiDAR：RA-L 2023 论文方法独立复现

论文：[Quadric Representations for LiDAR Odometry, Mapping and Localization](https://arxiv.org/abs/2304.14190v1)。
作者主页中的“Project Page”链接指向 IEEE 论文页面；本次没有找到作者公开的实现仓库。
本目录按论文建立 CPU 参考实现，不是作者源码；未声称复现 KITTI 00–10 / UrbanLoco 全部实验。

实现链路：**球面距离图 → 面片分割 → 平面/二次曲面/分布拟合 → 面片关联 → SE(3) 配准 →
里程计 → 增量矩统计建图 → 地图保存/空间哈希 → 载入地图后的定位**。

与原有 `fsgp_bgk_repro`、M20/Go2 模块相互独立，没有修改它们的算法。

## 运行

本机已有 `/usr/bin/python3`、NumPy、SciPy、Matplotlib 和 g++，不需要 CUDA、ROS 或 Open3D。

```bash
# 从 KITTI 官方 84 GB 归档中按 HTTP Range 只提取短序列04，共271帧
# 该序列的原始点云约520 MiB；同时下载标定、真值和官方评测程序
python3 -m quadric_lidar_repro.fetch_kitti --sequence 04

# 完整序列；不得在运行期间修改算法源码
bash quadric_lidar_repro/run_cpu.sh benchmark --sequence 04

# 检查残差/Jacobian、SE(3)、增量矩统计与地图重载
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest discover -s quadric_lidar_repro/tests -v

# 对全部输入、代码版本、坐标变换、KITTI指标、耗时及地图文件复核
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m quadric_lidar_repro.audit

# 关闭明确列出的法向一致性门限和鲁棒损失，保留更直接的公式残差
bash quadric_lidar_repro/run_cpu.sh benchmark --sequence 04 --literal \
  --output quadric_lidar_repro/outputs/literal
```

数据读取器强制要求 HTTP 206、ZIP CRC32 和 SHA-256；如果服务忽略 Range，则停止，
不会退化为整包下载。数据来源、每帧成员及哈希保存在 `data/kitti/sequence_04_manifest.json`。
数据归 KITTI 作者所有，使用条件以 [KITTI 官网](https://www.cvlibs.net/datasets/kitti/) 为准。

## 结果与查看

[完整实测报告](outputs/kitti04/REPORT.md) · [轨迹与耗时](outputs/kitti04/trajectory.png) ·
[面片类型](outputs/kitti04/patches.png) · [定位对照](outputs/kitti04/localization.png) ·
[原始指标](outputs/kitti04/metrics.json) · [复核记录](outputs/kitti04/audit.json)。

本工作区已有的42482服务可查看 **http://127.0.0.1:42482/quadric-lidar/**。
单独启动本模块的结果页：

```bash
python3 -m http.server 42483 --bind 127.0.0.1 \
  --directory quadric_lidar_repro/outputs/kitti04
```

然后访问 http://127.0.0.1:42483/ 。输出包括逐帧时间、相机坐标轨迹、KITTI逐子段误差、
保存的二次曲面地图和定位查询记录。

## 公式对应与边界

| 论文部分 | 实现 | 约定/差异 |
|---|---|---|
| Eq.(1) | `core.distance_and_gradient` | Taubin平方距离 f²/‖∇f‖²；不是单值高程场 |
| Eq.(2)–(6) | `core.register` | 平方距离作为残差、再做LM，按正文雅可比形成四次距离目标；解析SE(3)导数 |
| Eq.(7) | `core.associate` | α=1、β=0.1、γ=1.9；确定性抽样48点，加入中心距离门限 |
| Eq.(8),(9) | `core.Moments` | 用10项基函数的对称矩阵等价保存均值和S/P/Q；合并不重访历史点 |
| Eq.(10) | `core.fit_patch` | 由矩统计计算代数MSE；拟合使用参考文献[34]的Schur消元和二次系数归一化 |
| Patch segmentation | `segmentation.py`, `segment.cpp` | HDL-64标称束角、2048列距离图；增加35°法向分割门限 |
| Fitting | `core.fit_patch` | PCA识别平面；一般二次曲面用Q−P S⁻¹Pᵀ求解；未实现传感器噪声模型的ML优化；`--fitting taubin`保留另一拟合对照 |
| Map localization | `core.SpatialHash` | 1.5m整数单元哈希，冲突保留最低MSE；不是原版二进制GeoHash |

论文没有给出全部分割、拟合、退化阈值和优化配置，这些选择均在 `Config` / `protocol.json` 中明确记录。
默认版本额外使用45°近似平面法向关联门限和Cauchy损失（scale=1），以减少不相关的无限延伸曲面匹配。
它们是本地稳定性扩展，不是作者算法已验证的组成部分，`--literal` 可关闭。
首次未添加这两项的完整序列轨迹、指标和源代码快照另存 `outputs/literal_initial/`，不隐藏其漂移。

里程计/建图使用恒速初值，不读取真值。定位实验按论文使用真值位姿加 XY ±0.2m、yaw ±5° 扰动作为初值；
地图由估计轨迹建立，未用真值修正。报告同时展示定位前后误差，避免将初始先验精度当成定位成功。

KITTI ATE(%) / ARE(°/100m) 是100–800m子段的相对误差，不是绝对轨迹APE。
本实现同时保留无后对齐的APE RMSE（m），并逐子段交叉验证官方 `calcSequenceErrors`。
官方文件里的服务器/邮件/绘图入口不会执行：仅提取不变的指标函数进行编译和调用。

目前不包含ROS接入、IMU融合、扫描去畸变、回环，以及作者全部序列与基线的复现。
CPU时延和文件压缩比只描述此实现及当前序列，不能代替论文性能数字。

## 单独测拟合速度

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m quadric_lidar_repro.benchmark_fitting
```

[拟合耗时报告](outputs/fitting_speed/REPORT.md) ·
[逐次计时、配置和输入哈希](outputs/fitting_speed/metrics.json)。

Ryzen 9 7945HX、Python/NumPy、单线程BLAS，预热后实测默认Schur方法：
1,000点球面完整拟合中位数0.146ms，10,000点0.565ms；预存矩统计后约0.077ms。
球面明确检查为一般二次曲面，避免将平面快捷分支当作一般曲面拟合。
KITTI04每30帧取1帧，共10帧，每帧重复15次：236–489个面片全部重拟合，
各帧中位数23.21–41.59ms，10个帧中位数的中位数为27.82ms。
计时包含统计量、类型判定、系数拟合和协方差处理，不含读盘、分割、关联、位姿优化。
这项性能测试没有解决上述轨迹精度问题。

## 解析曲率的精度与速度对照

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m quadric_lidar_repro.benchmark_curvature
```

[完整验证报告](outputs/curvature_validation/REPORT.md) ·
[真值与估计曲率](outputs/curvature_validation/curvature_maps.png) ·
[原始精度与逐次耗时](outputs/curvature_validation/metrics.json)。
现有本地服务可直接打开 **http://127.0.0.1:42482/quadric-curvature/**。

`curvature.differential_geometry(patch, queries)` 对拟合曲面做零水平集投影，再解析计算法向、
坡度角、两个主曲率、平均曲率和高斯曲率；`valid` 标记梯度非零且投影收敛的位置。
有符号曲率取决于法向，当前约定下外法向球面的主曲率为−1/R。

验证覆盖平面、球面、圆柱、鞍面、椭球、缓弯球面及非二次起伏面，
每例4096个支持点、400个独立查询点、四档法向噪声、五个随机种子。
在1cm噪声下，Taubin对球面/圆柱/鞍面的曲率RMSE为0.0016–0.0024 m⁻¹；
当前默认Schur则为0.108–0.526 m⁻¹，存在明显拟合误差，不能直接用于可靠的曲率估计。
默认平面阈值也会压平R=10m的缓弯曲面；非二次起伏面需要更小的合适面片。
这些对照没有修改原里程计的默认参数。

单线程CPU、4096点构成一个面片、1000个查询点：Taubin拟合+投影+全部几何量为1.119ms，
已有模型时为0.749ms，局部二次64近邻为15.079ms，256近邻为57.662ms。
解析方法使用整个面片，局部方法分别只用64/256近邻，不能将不同支持范围下的精度差都归因于求导。
KITTI三帧密集查询6.1–7.7万点，Taubin拟合+几何为124–168ms，局部二次64近邻为873–1098ms，
不含分割和位姿优化；真实点云没有曲率真值，这部分只比较计算与速度。

## 使用高飞团队公开地形，从点云分割开始测试

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m quadric_lidar_repro.benchmark_terrain
```

[完整报告](outputs/fsgp_terrain/REPORT.md) · [逐帧指标](outputs/fsgp_terrain/per_frame.csv) ·
[场景与源码协议](outputs/fsgp_terrain/protocol.json)。
网页：**http://127.0.0.1:42482/quadric-terrain/**。

数据来自FSGP-BGK官方仓库固定提交的7份原始PCD：hill、forest、quick、desert、mountain、snow、vocano。
文件、源链接、Git blob SHA1和SHA256见 `data/fsgp_terrain/manifest.json`；hill/quick复用原目录文件。
它们是作者公开示例，不是论文完整2500组场景；本发布不含Road/Indoor/Ruin原始批测集和代价标签。

本次自动分割无序点云，不读取PCD自带法向或任何语义标签：20近邻PCA法向、1.5m空间块内的
距离/法向/点到平面约束连通分量，随后Taubin拟合并在625格地图上计算高程、坡度和物理曲率。
FSGP原方法不需要这种预分割。该空间分割是本地扩展，不是Quadric论文的64线距离图分割。

7场景×8帧、每方法每帧重复7次；同一输入和查询栅格上，包含分割的解析分支8.48–16.24ms，
局部高度二次拟合7.19–7.96ms，FSGP125 CPU参考9.46–14.66ms。
丘陵中法向估计7.51ms、区域分割0.83ms、拟合2.39ms、地图几何3.36ms。
因此当前端到端方案没有比局部拟合更快，其曲率参考误差也更大，不能沿用理想面片的速度/精度结论。

另测不设点数上限的完整PCD输入，计时内裁剪局部窗口：丘陵实际局部70974点，完整解析分支326.9ms；
局部高度二次拟合40.4ms、FSGP125参考306.0ms。所有耗时均不含PCD读盘。
精度参照来自完整干净点云的几何，曲率参考本身是估计值，未声称复现原论文的通行代价真值。

## 点云与拟合曲面的交互叠加

打开 **http://127.0.0.1:42482/quadric-fit/**。可切换全部7场景、8帧，显示原始点云、
实际拟合的二次曲面、分割颜色与Taubin近似距离；点击曲面可查看系数、局部距离统计，
勾选“只看选中面片”后放大观察。支持曲面透明度、网格线、投影连线及俯视/侧面视角。

页面默认显示100%不透明的拟合实面，顶部直接展示输入点云和实面的并排图；
“④ 放大局部二次曲面”一键显示最大一般二次曲面及其网格线。42482原首页也有明显入口。
静态对照无需WebGL即可查看：[输入点云与拟合实面对照](outputs/fitted_surfaces/fit-compare.png)。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  MPLCONFIGDIR=/tmp/quadric-lidar-matplotlib \
  /usr/bin/python3 -m quadric_lidar_repro.visualize_fits
```

导出器读取冻结实验的XYZ、分割标签和拟合配置；56帧重建高度与有效掩码全部和原记录一致。
曲面网格由每个面片的隐式方程F=0解析求根，在局部PCA坐标中选取靠近观测的实根，
裁剪到凸包和3D观测支持范围；不将点云直接三角化当作拟合面。面片间的空白保留。
检查网格顶点的|F|/‖∇F‖、三角形中点/重心偏差、支持范围与边长，避免跨越不同曲面分支。
距离统计不含未分配面片的点，Taubin距离与法向迭代投影都不是精确最近点距离。

[静态总览](outputs/fitted_surfaces/fit-overview.png) ·
[局部放大](outputs/fitted_surfaces/fit-detail.png) ·
[逐帧重建校验](outputs/fitted_surfaces/audit.json)。
新增可视化不修改上一轮的算法、分割参数或计时结果。
