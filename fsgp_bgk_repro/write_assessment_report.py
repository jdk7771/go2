"""Regenerate step.md from the implemented definitions and saved evidence."""
import json
from .assessment_export import OUT, NAMES, ROOT


def main():
    catalog=json.loads((OUT/'catalog.json').read_text())['scenes']
    audit=json.loads((OUT/'numerical-audit.json').read_text())
    timing=json.loads((OUT/'repeat-timing.json').read_text())['scenes']
    cases=json.loads((OUT/'controlled-noise.json').read_text())['cases']
    report=r'''# 20×20 cm 局部二次曲面与 50×80 cm 机器人窗口评估

更新：2026-11-17。本文将原始需求整理为已实现方法、计算口径和实测结果。原始文字保存在 [step.original.2026-11-17.md](step.original.2026-11-17.md)。

## 1. 做了什么，在哪里看

本次已实现严格 **20×20 cm 局部拟合**、独立拟合面与组合面分窗显示、误差与参数协方差、曲面上方 **10–60 cm 障碍证据**、A/B/C 三种跳变指标、落足代价以及 **50×80 cm 机身范围的几何综合评估**。17 个已有场景均生成了结果，包括没有足够支持、结果为 Unknown 的场景。

这是局部二次函数的工程实验，与 FSGP 论文的 GP 方法分开。当前机身评估是有假设的几何／准静态代理指标，没有输入实机质量分布、摩擦、关节可达性或运动状态，因此不能据此宣称实机动力学稳定或保证通过。

- 统一首页：<http://127.0.0.1:42482/>，简洁单图切换指标，统一白→红；白色实体面表示已拟合且指标低。勾选“拟合覆盖（蓝色）”查看有效曲面、未拟合中心与有效但未成面中心；点击曲面查看实际局部网格和数值。森林／桌子缺面归因见 [实验记录](experiment-results.md)，[指标与计算方法](result.md) 单独保留。
- 新工作台：<http://127.0.0.1:42482/assessment/details.html?scene=steps>。
- 丘陵：<http://127.0.0.1:42482/assessment/details.html?scene=hill>。
- 平地与斜坡：<http://127.0.0.1:42482/assessment/details.html?scene=flat_ramp>。
- 通道： [0.4 m](http://127.0.0.1:42482/assessment/details.html?scene=corridor-040)、[0.6 m](http://127.0.0.1:42482/assessment/details.html?scene=corridor-060)、[0.8 m](http://127.0.0.1:42482/assessment/details.html?scene=corridor-080)。三个都是仅含通道与地面的独立小场景。

原详细工作台保留 8 个可分别收起／展开的窗口：原始点云、独立拟合面、组合面、误差与不确定性、净空、跳变、落足、机身。相机同步。点击曲面或点云可选择附近的拟合中心，也能输入编号；紫色点表示实际拟合点。未知指标显示灰色，不填成零代价。

通道顶板和桌面会遮住下方地面。查看下方地面时，点击“选中央低表面”，勾选“曲面仅显示当前高度 ±10 cm”，再点击“放大当前位置”。这个开关仅裁剪显示，不删减用于净空或机身计算的点。

误差窗口默认将**当前场景有效函数的最大值映射为红色**，也可切换固定物理色标；全零数据使用固定参考门限。其他指标按明确物理门限或 0–1 代价着色。低值绿、高值红，不使用亮度光照改变误差颜色。显著性 S 的颜色表示数值，不表示危险度。

## 2. 局部面到底是什么

### 2.1 输入、选点与有效性

所有计算只使用 XYZ，不使用已知部件标签、真实台阶高度或预设表面数量。不设“地面／桌底／桌面三层”。每个中心独立选择自身高度附近的点；这是本地工程流程，不能称作论文的分割算法。

| 项目 | 当前设置 | 含义 |
|---|---:|---|
| 拟合中心 | 0.10 m 三维体素各取一个原始点 | 不把全部原始点都当中心 |
| 实际 XY 拟合域 | 20×20 cm | 中心左右各 10 cm，不是仅缩小显示面片 |
| 相对高度门限 | ±4 cm | 相对中心 Z，筛选发生在求解之前 |
| 最少点数 | 30 | 使用域内全部合格点，**不再封顶取最近 30 点** |
| 函数参数 | 6 个 | 常数、一阶和二阶项 |
| 数值正则 | 归一化 Gram 矩阵 trace/6 的 1e−10 倍 | 微小岭回归 |
| 满秩检查 | 最小／最大特征值比 >1e−10 | 退化、近共线点不能提供可靠二维曲面 |
| 中心预测位移 | ≤4 cm | 避免中心高度被拟合明显拉离 |
| 训练 RMSE | ≤2 cm | 质量检查，不是独立测试精度 |

索引树采用缩放后 XYZ 的无穷范数查询，得到精确的方形 XY 范围与高度带交集。点不足、秩不足、残差超限等情况保留缺失；不向其他高度表面借点凑数。

**点云密度变化必须单独说明：**历史平地等 8 cm 网格在 20 cm 方域内通常不足 30 点。因此平地、斜坡、平地与斜坡、两类台阶、桌子和三个通道，使用原来的解析几何显式重新生成 2.5 cm 间距点云；没有把原扫描插值成“实测点”。丘陵、森林、沙漠、山地、雪地、火山、quick 和拼接大场景使用原有 XYZ 文件，未为本次实验加密。各场景 `data.json` 中记录了来源。

历史 30 近邻结果仍保存在首页原查看器与 `outputs/visualization/scene-fits/`，它使用 0.12 m 中心体素、0.75 m 搜索半径和最多 24×24 cm 的显示域；与本次严格 20 cm 新流程分别保存。数值与运行量不能混用。

### 2.2 二次函数与显示组合

以中心为局部原点，u=x−x_seed，v=y−y_seed：

\[
f(u,v)=z_{seed}+c_0+c_1u+c_2v+c_3u^2+c_4uv+c_5v^2,
\quad \phi=[1,u,v,u^2,uv,v^2]^T.
\]

独立面片：在每个有效函数的 20 cm 方域内采样 5×5 个点，顶点高度确实来自该二次函数；仅保留顶点都在原拟合点 XY 凸包和 ±4 cm 预测高度带内的三角片。它是函数的离散渲染近似，实际显示面积可能小于 0.04 m²，也可能为零。不同中心的面片可重叠，不能累加为场景覆盖面积。

组合面：沿用原有一致性组网。中心间三维距离 ≤35 cm，双向模型预测差 ≤2 cm，中心法向夹角 ≤15°；保留至少 30 个有效中心的连通组件，再做 XY Delaunay、三边与重心预测一致性检查。**最终组合网格是拟合中心之间的分片线性三角网，不是把所有独立二次面连续焊成一张光滑函数。**仍保留局部二次系数用于导数与评估。

“有效函数”“实际显示面片”“参加组合的函数”“组合三角形”是不同数量。没有三角片，不一定没有拟合函数；没有参加组合，也不一定拟合失败。

## 3. 误差、协方差和不确定性如何计算

对本次选中的 n 个点，残差 e_j=f(x_j,y_j)−z_j：

\[
RMSE=\sqrt{\frac1n\sum_j e_j^2},\qquad E_{max}=\max_j |e_j|.
\]

两者都是**训练残差**。另设独立解析坐标测试，用于后面的受控实验，不把训练误差称为泛化精度。

为减小条件数，将 XY 按支撑点均方根半径 s 归一化。设归一化设计矩阵为 A，G=AᵀA，M=(G+λI)⁻¹，D=diag(1,1/s,1/s,1/s²,1/s²,1/s²)。使用有限岭回归的参数协方差：

\[
\hat\sigma_\epsilon^2=\max\left(\frac{\sum e_j^2}{n-6},(0.003\,m)^2\right),
\qquad\Sigma_\theta=\hat\sigma_\epsilon^2 D M G M^T D^T.
\]

残差方差用自由度 n−6，参考 [NIST 最小二乘说明](https://www.itl.nist.gov/div898/handbook/pmd/section4/pmd431.htm)。极小岭回归仍使用这一近似自由度。3 mm 是本实验设置的噪声下限，避免解析平面因为残差为零而给出零不确定性；它不是传感器标定结果。

任意位置的**拟合均值高度方差**：

\[
\sigma_z^2(x,y)=\phi(x,y)^T\Sigma_\theta\phi(x,y).
\]

页面可查看完整 6×6 协方差、中心 σ、实际显示域内采样点的最大 σ。将误差和不确定性相加时，使用相同的米单位：

\[
E=E_{max}+2\max_{\text{显示域内采样点}}\sigma_z.
\]

这不是直接把“残差的米”与“方差的平方米”相加。该协方差以固定 XY、独立同方差高度噪声和局部二次模型近似成立为条件；未包含 XY 测量误差、点间相关、边界位置不确定性或模型偏差，也未加上新观测自身的噪声方差。**2σ 没有经过传感器或场景覆盖率标定，不能直接称 95% 置信保证。**

## 4. 三种跳变方案

### 4.1 怎样选双侧候选

从有效拟合中心出发，在 ±X、±Y 四个方向，目标偏移 24 cm。每个目标附近查询最多 16 个中心，查询半径 10.5 cm，剔除中心间距不足 14 cm 的候选，再按 XY 距离及预测高度差选择一个；双向产生的重复对去重。

对 i、j 两中心，比较位置 p=(XY_i+XY_j)/2，方向 d=XY_j−XY_i。**p 是候选中点，没有先精确定位真实地形边界。**完整场景中的倾角转折、过悬多层、候选错配或三次以上背景都可能造成非零指标。

### 4.2 A：单侧二阶预测残差

\[
r_{i\to j}=\left|\hat z_j-\left(\hat z_i+\nabla f_i^T d+\frac12d^TH_id\right)\right|,
\qquad A_{ij}=\max(r_{i\to j},r_{j\to i}).
\]

对二次函数，二阶 Taylor 预测就是直接代入已有多项式。实现取两个单侧预测的较大值，目标高度是对方中心的拟合高度。它避免重新拟合，比较距离却较远；噪声和连续折角会使它比双侧方案更敏感。

### 4.3 B：双侧在同一位置外推

以过 p、法向为 d 的直线切开原拟合点集。左侧只用投影严格为负的点，右侧只用严格为正的点，边界上的点不共享。各自重新进行二次拟合，每侧仍要求至少 30 点和同样质量检查。

\[
B_{ij}=|f_{right}(p)-f_{left}(p)|.
\]

比较在相同 XY 的高度，因此避免直接把连续坡面上相隔一段距离的自然高差当成台阶。理想无噪声、恰可由二次函数表达的背景叠加在两侧时，在同一点相消。这个性质不代表对任意复杂曲面或错误候选都完全不变。

### 4.4 C：双侧跳变加协方差不确定性

每一对候选的两侧使用不重叠原始点 ID；在独立高度噪声假设下：

\[
\sigma_\Delta^2=\phi_l(p)^T\Sigma_l\phi_l(p)+\phi_r(p)^T\Sigma_r\phi_r(p),
\qquad C_{ij}=B_{ij}+2\sigma_\Delta.
\]

另外导出：

\[
S_{ij}=\frac{B_{ij}}{\sqrt{\sigma_\Delta^2+10^{-12}\,m^2}}.
\]

S 是模型标准差尺度上的比值，不能把 S 当作台阶大小、危险程度或经校准的显著性检验。高不确定性会降低 S，却会提高 C；用于几何代价的是 C。真实点若存在相关误差，还需要两侧交叉协方差项。

每个拟合中心分别取所有参与候选对的最大 A、B、σΔ、S、C。**这些列的最大值可能来自不同候选对**；C 是先对每对计算 B+2σΔ，再取最大，并非“最大 B + 2×最大 σΔ”。逐对 CSV 保留了各对数值。候选对不足或任一侧不支持，B/C 为 Unknown，不设为零。已知方向数量只统计成功的候选，不宣称四周都已观测完整；多个中心反向选中时数量可以超过四。

## 5. 上方 10–60 cm 的空间占用

对每个有效地面函数，在 20 cm XY 方域中分别检查：原始点，以及**实际拟合三角片使用的顶点采样**。每个样本都用自身 XY 处的地面预测值计算相对高度：

\[
\Delta h=z_{sample}-f_{ground}(x_{sample},y_{sample}).
\]

若 0.10≤Δh≤0.60 m，记为障碍证据；端点包含，浮点容差为 1e−7 m。用 5 cm 列把 20 cm 域分成 4×4=16 列，原始点与拟合面证据做并集：

\[
O=\frac{\text{存在障碍证据的列数}}{16}\in[0,1].
\]

O 越大，观测到的上方占用越多。CSV 同时保留原始命中数和拟合面顶点命中数；它们不是独立障碍数量，不对重叠面片的点数直接累加打分。

这里的“曲面检查”是三角面顶点采样证据，没有求解连续二次曲面／机身实体的精确碰撞，也没有射线可见性与遮挡建模。稀疏样本可能漏掉薄障碍；O=0 只表示没有观测到该检查带内的障碍，不能证明自由空间。0.8 m 顶板在这项指定检查带以外，不据此推断任意高度机器人都可通过。

## 6. 坡度、粗糙度与落足综合指标

二次函数可以直接求导：

\[
\nabla f=(c_1+2c_3u+c_4v,\ c_2+c_4u+2c_5v),\qquad
H=\begin{bmatrix}2c_3&c_4\\c_4&2c_5\end{bmatrix}.
\]

Hessian 的系数不是一般曲面的主曲率；本次没有将其直接冒充曲率。中心坡度 α=atan(√(c₁²+c₂²))。粗糙／弯曲代理用训练 RMSE 和 20 cm 域四角相对中心的最大法向夹角 Δn；角点梯度是模型外推，角点可能在显示凸包外。

令 clip(t)=min(1,max(0,t))：

\[
P=clip(\alpha/45^\circ),\quad
R=\max\{clip(RMSE/0.01m),clip(\Delta n/15^\circ)\},
\]
\[
J=clip(C/0.12m),\quad U=clip(E/0.02m),\quad
F=0.25P+0.20R+0.25J+0.20O+0.10U.
\]

中心坡度 ≥45° 时 F=1，表示本实验判为不宜落足。其他必要指标未知时保留 Unknown。R 包含平滑弯曲本身的影响，不能等同于纯测量噪声或严格表面粗糙度。

45°、1 cm、15°、12 cm 及上述权重都是本次工程设定；没有声称来自 FSGP 论文或已经完成 Go2 实机标定。综合代价主要用于观察各项变化，没有给出可靠的实机通过分数阈值。

## 7. 50×80 cm 机身下方如何汇总

机身长边固定沿 X，为 80 cm；短边沿 Y，为 50 cm，当前不搜索不同朝向。对每个有效中心，建立：

- 机身内含边界的 5×5 网格，共 25 个查询位置。
- 4 个假定足点，偏移为 (±35 cm,±20 cm)。这是支撑点布局假设，不是实机运动学求解结果。

每个位置从有效中心中查询最多 16 个附近候选，限定在该模型的 20 cm 方域及 ±4 cm 预测高度带内，按 XY 距离和相对于当前切平面预期高度的差选支持表面，以尽量保持同一高度分支；不指定“地面”标签。当前查询没有逐次检查原始点 XY 凸包，因此边缘可能有小范围外推，多层表面附近也可能错配。

25 点覆盖率 ≥80%，且四足均找到支持，才继续汇总。四足拟合平面 z=a+gₓu+gᵧv，得到平面坡度 α_b、共面 RMSE r_b 和最大垂直残差。窗口中的净空、保守跳变、粗糙度、误差取 **25 点与四足所引用模型中的已知最大值**。它是有限采样的最坏观测值，不是整个连续矩形上的严格最大值；部分候选方向缺失仍可能被其他已知值覆盖，不能将它称为全域完整观测。

假定质心在该支撑平面中心沿法向高 0.30 m，则其重力投影的 XY 偏移：

\[
q_x=-0.30g_x/\sqrt{1+g_x^2+g_y^2},\qquad
q_y=-0.30g_y/\sqrt{1+g_x^2+g_y^2}.
\]

矩形四足支撑多边形下的投影裕量：

\[
m=\min(0.35-|q_x|,\ 0.20-|q_y|)\quad(m).
\]

它衡量假定重力投影到最近支撑边的距离；平地假设下为 20 cm。支撑多边形只是静态平衡分析的一部分，接触法向和摩擦约束仍然必要，参见 [Robotic Systems 的接触与平衡章节](https://motion.cs.illinois.edu/RoboticSystems/AdvancedTopicsInPlanning.html)。本次没有求完整接触力平衡。

机身代价：

\[
D=clip(r_b/0.02m),\quad M=clip((0.10m-m)/0.10m),
\]
\[
B_{body}=0.20clip(\alpha_b/45^\circ)+0.15D+0.25J_{max}
+0.20O_{max}+0.10R_{max}+0.05U_{max}+0.05M.
\]

平面坡度 ≥45°、投影裕量 ≤0，或四足相对平面的最大垂直残差 >15 cm 时，代价置为 1；缺少四足、覆盖不足或必要指标不可得时保持 Unknown。质心高度可以在配置中调整，权重在评分函数中显式列出；当前没有把这些假设当作实机参数。

这里未检查足底力／摩擦锥、整机质心随姿态和腿位变化、完整关节可达性、动态冲击、侧倾惯性、连续机身碰撞或步态稳定性。低代价表示在这些假设下几何更有利，不表示已验证机器人一定稳定。

## 8. 实际效果

### 8.1 17 个场景

下表是新 20 cm 流程。求解列只统计**主窗口**求解；三种方法的双侧重拟合另列在每个场景 JSON 中。有效率的分母是中心数，不是原始点数。机身“已知”不是“安全”。各场景时间为首轮单次，另有三个代表场景的三次重复中位数。

| 场景 | 原始点 | 中心 | 主求解 | 有效函数 | 有实际显示面片的函数 | 机身代价已知 | 单次总耗时 ms |
|---|---:|---:|---:|---:|---:|---:|---:|
'''
    by_id={r['scene']:r for r in audit['scenes']}
    for meta in catalog:
        r=by_id[meta['id']]
        report+='| '+ ' | '.join([meta['name'],*[f'{x:,}' for x in [r['raw'],r['centers'],r['solved'],r['valid'],r['functions_with_visible_patch'],r['body_known']]],f"{meta['timings']['stages_ms']['total_assessment_ms']:.1f}"])+' |\n'
    report+=r'''
丘陵有 13,860 个中心，13,791 次主求解，13,789 个有效函数：69 个点不足，2 个求解后未通过检查。其中 12,448 个函数真正显示出三角片，1,341 个有效函数因显示采样／裁剪没有三角片；452 个有效函数未参加最终组合。这两类可重叠，不能相加当作互斥“失败数”。显示洞、求解失败和组合失败要分别看。

quick 在 20 cm 和至少 30 点这一约束下，99,942 个中心没有一个能实际求解；沙漠只有 844 个有效函数，尚无机身已知代价。这说明当前固定窗口与密度假设对这些输入不适用；没有通过补点隐藏问题。拼接大场景也保留大量 Unknown，因此不能将局部有结果理解为整个大场景都完成了稳定性判断。

### 8.2 通道中央地面的可重复对照

| 顶板净空 | 上方 10–60 cm 观测占用 O | 机身窗口最大占用 | 中央机身代价 | 假设下的投影裕量 |
|---|---:|---:|---:|---:|
'''
    for name in ('corridor-040','corridor-060','corridor-080'):
        r=by_id[name];m=r['central_metrics']
        report+=f"| {int(name[-3:])/100:.1f} m | {m['clearance_cost']:.2f} | {m['body_max_clearance']:.2f} | {m['body_cost']:.5f} | {100*m['body_support_margin_m']:.1f} cm |\n"
    report+=r'''
0.4 m 和 0.6 m 的顶板在指定检查带内，中央地面净空窗口为红色；0.8 m 顶板在检查带外，该项为绿色。三个场景中央地面支撑都平，投影裕量相同；它们的净空代价不同。这正好区分“脚下平整”和“机身上方被占用”。综合代价采用加权求和，障碍项不是一票置 1 的碰撞判定。

中央平地的机身代价约 0.01624，即使无几何误差也不严格为零：3 mm 噪声下限带来了预测不确定性和保守跳变项。大桌子中央地面的 10–60 cm 占用为零，是由于本场景桌板高于该检查带；并不表示桌板不存在。

![0.6 m 通道中央地面，剖切显示后的净空占用](fsgp_bgk_repro/outputs/visualization/assessment/corridor-060-clearance.png)

![50×80 cm 机身、四足与质心投影](fsgp_bgk_repro/outputs/visualization/assessment/corridor-060-body.png)

### 8.3 三种跳变的受控实验

设置两个 16×16 cm 支撑点集，中心在 x=±12 cm，每侧 9×9 点，真实边界固定在 x=0；真实跳变为 0 或 12 cm。背景包含平面、斜坡、二次面、连续折角和三次面。加 0、3、15 mm 独立高度高斯噪声，每个条件 100 次，共 30 条件、3,000 次。

真实函数用于生成观测并作事后评价，算法只收到 XYZ；拟合中心高度也取含噪声的实际观测点，没有用真实高度来限制邻点。该受控布局的候选中点恰好落在真实边界，完整场景不提供这项有利条件。另在每侧独立随机 XY 坐标上评价有效模型对无噪声真实函数的均值高度误差；它与训练残差、对新含噪观测的预测误差分开。

下面只列真实跳变 120 mm 的代表结果；A/B 的 RMSE 是相对真实跳变的估计误差，C 是保守代价量，不能将其均值直接称作估计精度。

| 背景 | 噪声 σ mm | A 跳变 RMSE mm | B 跳变 RMSE mm | C 平均值 mm | 双侧 2σ 诊断覆盖率 |
|---|---:|---:|---:|---:|---:|
'''
    for kind,noise,label in [('flat',.003,'平面'),('quadratic',0.,'二次面'),('quadratic',.003,'二次面'),('quadratic',.015,'二次面'),('kink',0.,'连续折角'),('cubic',0.,'三次面（不匹配）')]:
        row=next(r for r in cases if r['background']==kind and r['observation_noise_m']==noise and r['true_jump_m']==.12)
        coverage=row['absolute_jump_error_within_2sigma_fraction']
        report+=f"| {label} | {noise*1000:g} | {row['a_rmse_m']*1000:.3f} | {row['b_rmse_m']*1000:.3f} | {row['c_mean_m']*1000:.3f} | {coverage*100:.0f}% |\n"
    report+=r'''
无噪声且符合二次模型时，A/B 能以数值精度恢复 120 mm 台阶；0 mm 台阶也得到接近零的 A/B。C 由于噪声下限会额外增加约 5.4 mm，不应期待它与真值完全相等。

二次背景加 3 mm 噪声时，B 的跳变 RMSE 约 2.47 mm，优于 A 的 6.91 mm；该条件独立 XY 的拟合均值高度 RMSE 约 0.61 mm。连续折角在已知正确边界处高度连续，B 不把这一坡度转折当成台阶；A 有 36 mm 误差。**完整场景不提供真实边界**，不能把这个理想条件的结果直接当作完整场景的检测准确率。

三次背景反例更重要：无噪声时 B 仍有约 11.62 mm 模型偏差，双侧 2σ 诊断覆盖率为 0%。这表明只加参数协方差不能吸收模型不匹配偏差。本实验没有宣称“对任意连续地形严格不变”或“2σ 已经覆盖所有误差来源”。

覆盖率这里定义为 |B−J_true|≤2σΔ 的经验比例，使用非负跳变幅度，分母为双侧支持已知的试验数（CSV 的 bilateral_known），每条件至多 100 次；不是通用 95% 校准结论。C 的保守上调与 A/B 点估计也不能放在同一精度名义下排名。

![独立受控试验的台阶估计误差](fsgp_bgk_repro/outputs/visualization/assessment/controlled-jump-errors.png)

完整 [CSV](fsgp_bgk_repro/outputs/visualization/assessment/controlled-noise.csv) 和 [JSON](fsgp_bgk_repro/outputs/visualization/assessment/controlled-noise.json) 保留全部 30 个条件与独立坐标误差。

## 9. 速度，算到了哪一步

环境：AMD Ryzen 9 7945HX，系统 Python 3.10、NumPy 1.21、SciPy 1.8；CPU 计算，BLAS 单线程。下面是三个代表场景各运行 3 次的中位数，输入 XYZ 已在内存。

| 阶段 / ms | 丘陵 | 高低组合台阶 | 拼接大场景 |
|---|---:|---:|---:|
'''
    timing_by={r['scene']:r['medians_ms'] for r in timing}
    for key,label in [('select_centers_ms','选中心'),('window_query_ms','建树与严格方域查询'),('fit_and_covariance_ms','主窗口拟合 + 参数协方差'),('assembly_ms','一致性组合'),('display_patch_geometry_ms','全部局部面片显示几何'),('three_jump_methods_ms','A/B/C（含双侧重拟合）'),('clearance_ms','上方净空'),('local_costs_ms','局部代价'),('body_50x80_ms','50×80 cm 机身评估'),('total_assessment_ms','完整新增流程')]:
        report+='| '+label+' | '+' | '.join(f"{timing_by[name][key]:.1f}" for name in ('hill','steps','combined'))+' |\n'
    report+=r'''
各阶段独立取中位数，阶段中位数相加不一定恰好等于总时间中位数。单次网页场景卡与此处三次中位数口径不同。

以丘陵为例：13,791 次主窗口求解以外，B/C 共同使用了 37,396 个候选对，触发 74,792 个侧窗口，其中 **74,785 次实际侧求解**；37,389 对双侧都通过检查。侧求解包含重复支撑集，目前没有缓存同一个侧点集的重复拟合。不能把整套算法说成“只拟合了 13,791 次”。

结果表明主窗口二次拟合和协方差仍较快，丘陵约 211 ms；完整流程约 **6.07 s**，明显更贵的部分是双侧评估、全部显示面片与净空查询。当前代码是离线预计算工作台，不是已经满足实时帧率的在线机器人模块。网页加载已保存结果的时间也不等于算法运行时间。

总时间包括选中心、树与查询、拟合、协方差、组合、显示几何、三种跳变、净空和机身评估；不包括点云生成、文件读写、网络传输和浏览器渲染。历史几百毫秒的 30 近邻拟合＋组合，配置、中心数、点云密度和计算范围均不同，不能直接拿来声称本算法更快／更慢多少倍。

![三次重复中位数的时间组成](fsgp_bgk_repro/outputs/visualization/assessment/pipeline-timing.png)

原始重复数据：[repeat-timing.json](fsgp_bgk_repro/outputs/visualization/assessment/repeat-timing.json)。

## 10. 验证证据、文件和复现命令

本次检查包括：

- 6 项新增解析单元测试：真实二次函数恢复、独立设计矩阵协方差、相同二次背景不变性／台阶恢复、缺失侧为 Unknown、严格方域与高度带、相对斜面的净空，以及坡度硬门限和假定平地机身裕量。
- 与已有邻点／几何测试合计 13 项单元测试通过。
- 17 场景数值审计：原始点、中心、求解、有效模型、显示三角片、组合引用、邻点窗口、残差与协方差独立复算、CSV 行数、机身支持覆盖，以及额外双侧实际求解数。
- 新工作台浏览器检查：17 场景，8 个窗口，A/B/C 与误差选择、折叠、局部面、按高度剖切、机身标记、下载、首页场景同步、手机无横向溢出；无页面错误或资源失败。
- 原查看器的 17 场景回归通过；17 个历史拟合 NPZ 的 SHA256 与此前审计一致。42482 常驻用户服务运行中。

数值审计对每场景抽查最多约 200 个求解模型（不是每个模型都做独立矩阵复算）；计数、二进制尺寸、索引与 CSV 行数按全量检查。当前最大绝对训练残差复算差约 4.2e−17 m，协方差元素最大复算差约 2.2e−11。这些检查验证实现一致性，不替代真实 LiDAR 精度或实机通行验证。

证据文件：[数值审计](fsgp_bgk_repro/outputs/visualization/assessment/numerical-audit.json)、[新工作台浏览器审计](fsgp_bgk_repro/outputs/visualization/assessment/browser-audit.json)、[原查看器浏览器审计](fsgp_bgk_repro/outputs/visualization/scene-fits/browser-audit.json)。

实现入口：

| 文件 | 职责 |
|---|---|
| [terrain_assessment.py](fsgp_bgk_repro/terrain_assessment.py) | 严格窗口拟合、协方差、三种跳变、净空与局部／机身代价 |
| [assessment_export.py](fsgp_bgk_repro/assessment_export.py) | 17 场景新结果导出，来源记录与首轮阶段计时 |
| [validate_assessment.py](fsgp_bgk_repro/validate_assessment.py) | 数值审计、受控噪声实验与重复阶段计时 |
| [新工作台页面](fsgp_bgk_repro/visualization/assessment/index.html) | 8 个可折叠窗口与使用说明 |
| [viewer.js](fsgp_bgk_repro/visualization/assessment/viewer.js) | 实际几何渲染、绿红色标、选择模型与机身显示 |

每个新场景的输出在 `fsgp_bgk_repro/outputs/visualization/assessment/<scene>/`：

- `data.json`：参数、来源、各项统计、实际求解数量和阶段耗时。
- `per-model.csv`：全部中心，每个二次函数的误差、方差、跳变、净空与机身指标；未知留空。
- `jump-pairs.csv`：每个候选对的 A、B、σΔ、C；真实边界未知。
- `result.npz`：模型、完整 6×6 协方差、实际邻点 CSR 索引、原始中心索引、最终三角网和四足支持引用。
- 页面 `.bin`：原始点、模型、协方差、独立面片、组合网、实际邻点与四足支持数据。

```bash
cd /home/jiang/snap/go2

# 重算并发布全部 17 个场景；保留历史 scene-fits 结果。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.assessment_export

# 数值审计、每条件 100 次噪声试验、三个代表场景各 3 次重复计时。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.validate_assessment \
  --repeats 100 --timing-scenes hill steps combined --timing-repeats 3

# 解析单元测试；浏览器检查需要已安装的本机 Chrome 与 Playwright。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest fsgp_bgk_repro.tests.test_terrain_assessment -v
node fsgp_bgk_repro/tests/terrain_assessment_browser.cjs

# 由保存的数值证据重新整理本文。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.assessment_figures
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.write_assessment_report

# 启动／查看既有同端口服务；无需再起一个端口。
systemctl --user start go2-pointcloud-viewer.service
systemctl --user status go2-pointcloud-viewer.service
```

若没有常驻服务，可在确保 42482 没有被既有服务占用时直接执行 `/usr/bin/python3 fsgp_bgk_repro/serve_visualization.py`。监听仍为 `127.0.0.1:42482`。

当前已完成上述本地计算与展示；尚未做的是真实传感器不确定性标定、完整机器人运动学／接触力／动力学验证和实时在线优化。这些边界不影响查看已生成的几何指标，但限制其在机器人控制中的直接使用。
'''
    timing_section=OUT/'timing/step-section.md'
    if timing_section.exists():
        report=report.replace('## 9. 速度，算到了哪一步\n', '## 9. 速度，算到了哪一步\n\n本节保留首轮三个代表场景的重复计时；最新的全部 17 场景、逐图计时见第 11 节。重复运行存在小幅波动。\n')
        report+='\n\n'+timing_section.read_text()
    comparison_section=OUT.parent/'fsgp-q20/step-section.md'
    if comparison_section.exists():
        report+='\n\n'+comparison_section.read_text()
    profile_section=OUT/'profile-comparison/step-section.md'
    if profile_section.exists():
        report+='\n\n'+profile_section.read_text()
    (ROOT.parent/'step.md').write_text(report)
    print('Wrote',ROOT.parent/'step.md')


if __name__=='__main__':main()
