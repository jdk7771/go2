# 本机复验记录（2026-10-21）

## 版本与环境

- 仓库：`https://github.com/jdk7771/go2.git`
- 复验分支：`research/3d-wheeled-planning-notes-20261021`
- 提交：`d5ed1e3d5d09a64f770092e4fc002a6623f2a29f`（远端分支提交时间 2026-10-21 20:24:29 +0800；复验结束前再次核对，未变化）
- 数值 Python：`/home/jiang/miniconda3/envs/nero/bin/python`，Python 3.10.20、NumPy 1.26.4、SciPy 1.15.2、Matplotlib 3.10.9。
- PhysX：本机 `/home/jiang/Work_mark/isaac-sim-5.1.0/python.sh`，Isaac Sim 5.1.0，RTX 4060 Laptop GPU 8 GiB，NVIDIA 驱动 580.178.04。
- 这里的四份 `.py` 是原分支 `validation_20261021/` 的独立副本；仅将 Isaac 结果的版本说明改为 5.1.0。所有生成结果写在此目录，原分支已有结果未覆盖。

## 已执行的检查

在仓库根目录执行：

```bash
MPLCONFIGDIR=/tmp/go2-mpl-cache OPENBLAS_NUM_THREADS=1 /home/jiang/miniconda3/envs/nero/bin/python local_validation_20261021/run_checks.py > local_validation_20261021/numerical.log 2>&1
cd local_validation_20261021
MPLCONFIGDIR=/tmp/go2-mpl-cache OPENBLAS_NUM_THREADS=1 /home/jiang/miniconda3/envs/nero/bin/python transition_checks.py > transition.log 2>&1
MPLCONFIGDIR=/tmp/go2-mpl-cache OPENBLAS_NUM_THREADS=1 /home/jiang/miniconda3/envs/nero/bin/python contact_graph_demo.py > contact_graph.log 2>&1
```

Isaac Sim 命令从仓库根目录执行，需主机 GPU 访问：

```bash
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=YES OPENBLAS_NUM_THREADS=1 timeout --kill-after=15s 240s /home/jiang/Work_mark/isaac-sim-5.1.0/python.sh /home/jiang/Work_mark/go2/local_validation_20261021/isaac_collision_checks.py > /home/jiang/Work_mark/go2/local_validation_20261021/isaac.log 2>&1
```

四项命令均以退出码 0 结束。核心结果：

| 检查 | 本机结果 | 与原记录对比 |
|---|---|---|
| 38 个简化几何场景 | 两种方法均返回 38 条机身路径；缺口相关 24 个场景的 body-only 路径均未通过逐节点支撑筛选，加入筛选后均找到替代路径 | 分类计数一致 |
| 摆轮路径 | 固定摆动最小净空 −0.110 m；搜索路径最小净空 0.0128011 m | 一致 |
| 1175 条路径边审计 | 834 条符合固定轮向纯 x 滚动简化条件；1174 条可保持原四轮接触跨越单边；台阶有 1 条因静态支撑裕度不足失败 | `transition_results.json` 完全一致 |
| 四点足接触图 | 连续平地找到 50 动作序列（22 次机身前移、28 次单腿换足）；横贯缺口搜索穷尽、未找到路径 | `contact_graph_results.json` 完全一致 |
| Isaac Sim 5.1 PhysX 静态碰撞 | 固定摆轮 301 点中 111 点碰撞；搜索摆轮 616 点中 0 点碰撞；机身偏航 0°/30° 为不碰撞/碰撞；膝反例的机身/轮/膝为不碰撞/不碰撞/碰撞 | 与原 Isaac Sim 6.0.1 的计数和布尔结果一致 |

`results.json` 与原结果的非计时字段只有 4 个浮点末位差异，最大绝对差 `5.55e-17`；场景、路径、布尔结论和其他数值均一致。计时随机器和依赖版本变化，不用于跨机结论。

## 证据边界与下一步

上述是简化运动学/静力检查和静态 PhysX 重叠查询，不是四轮足机器人闭环行走。最新分支只包含研究文档与这些验证脚本，没有目标四轮足 URDF/USD、执行器参数或参考跟踪控制器；本机 Isaac Sim 安装中也没有可直接代表目标四轮足的模型。因此文档 `rolling_stepping_experiment_protocol.md` 的 E0 标定和 E1 滚动—抬腿—滚动闭环仍未执行。要开展 E0，首先需要确定具体机器人模型、轮轴/关节及执行器参数、控制接口；之后按该文档第 6.1 节做静止承载、IK 工作空间、滚动与单腿承载测试，E0 通过后再做第 6.2 节的 E1。

原分支自带的检查限制详见 `../validation_20261021/README.md`。本次 Isaac Sim 5.1 与原 6.0.1 的一致性，只支持这些静态几何反例的跨版本复现，不证明真实机器人可通过目标场景。
