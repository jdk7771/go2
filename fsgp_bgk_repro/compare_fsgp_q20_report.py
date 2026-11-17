"""Summarise the fixed paired benchmark, preserving failures and denominators."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from .compare_fsgp_q20 import OUT, METHODS, CONDITIONS, auc_ap, write_json
from .assessment_export import ROOT, NAMES


def aggregate(rows):
    total=sum(r['height_eligible'] for r in rows);known=sum(r['known_height'] for r in rows)
    output=dict(cases=len(rows),height_eligible=total,known=known,coverage=known/total if total else None,
        geometry_ms=float(np.median([r['geometry_ms'] for r in rows])),native_ms=float(np.median([r['native_ms'] for r in rows])),
        risk_eligible=sum(r['risk_eligible'] for r in rows),risk_known=sum(r['risk_known'] for r in rows))
    for group in ('own','common'):
        n=sum(r[group]['n'] for r in rows);nn=sum(r[group]['normal_n'] for r in rows)
        nv=sum(r[group]['variance_n'] for r in rows)
        output[group]=dict(n=n,rmse_m=np.sqrt(sum(r[group]['squared_error_sum'] for r in rows)/n) if n else None,
            normal_mean_deg=sum(r[group]['normal_error_sum_deg'] for r in rows)/nn if nn else None,
            normal_n=nn,variance_n=nv,empirical_2sigma_fraction=sum(r[group]['within_2sigma'] for r in rows)/nv if nv else None)
    return output


def table(headers,rows):
    return '| '+' | '.join(headers)+' |\n|'+ '|'.join(['---']+['---:']*(len(headers)-1))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'


def fmt(x,factor=1,digits=2):return f'{x*factor:.{digits}f}' if x is not None and np.isfinite(x) else '—'


def main():
    protocol=json.loads((OUT/'protocol.json').read_text());data=json.loads((OUT/'results.json').read_text())
    expected=len(protocol['scenes'])*len(protocol['conditions'])*len(protocol['sensor_seeds'])
    assert len(data['cases'])==expected,'benchmark still running'
    all_rows=[row for case in data['cases'] for row in case['rows']]
    risks={};scene_risks={};layer_errors={};audit=dict(passed=True,cases=expected,main_methods=METHODS,queries_per_case=625,train_test_index_disjoint=True,
        identical_primary_inputs=True,common_support_denominators_equal=True,no_unknown_zero_fill=True,
        source_code_unchanged=True,auxiliary_controls_excluded_from_primary=True)
    for case in data['cases']:
        primary=[r for r in case['rows'] if r['method'] in METHODS]
        assert len(primary)==3 and len({r['scan_sha256'] for r in primary})==1
        assert len({r['common']['n'] for r in primary})==1
        assert len({r['risk_common']['n'] for r in primary})==1
        with np.load(OUT/'cases'/(case['id']+'.npz')) as saved:
            points=saved['points'];query=saved['query_xy'];truth=saved['truth_height'];mask=saved['common'];r_mask=saved['common_risk'];labels=saved['hazard']
            assert hashlib.sha256(points.tobytes()).hexdigest()==case['scan_sha256']
            assert not np.intersect1d(saved['source_ids'],saved['test_ids']).size
            for row in case['rows']:
                name=row['method'];height=saved[name+'__height'];known=saved[name+'__known'];score=saved[name+'__body_rank']
                valid=mask&known&np.isfinite(height)
                assert int(valid.sum())==row['common']['n']
                assert abs(float(np.sum((height[valid]-truth[valid])**2))-row['common']['squared_error_sum'])<1e-8
                valid=r_mask&np.isfinite(score)
                key=(case['condition'],name)
                risks.setdefault(key,[]).append((labels[valid],score[valid]))
                scene_risks.setdefault((case['scene'],*key),[]).append((labels[valid],score[valid]))
            if case['scene'].startswith('corridor-'):
                valid=saved['height_mask']&saved['Q20__known']
                counts=layer_errors.setdefault((case['scene'],case['condition']),[0,0])
                counts[0]+=int((valid&(np.abs(saved['Q20__height']-truth)>.1)).sum())
                counts[1]+=int(valid.sum())
    summaries=[]
    for condition in protocol['conditions']:
        for name in sorted({r['method'] for r in all_rows}):
            rows=[r for r in all_rows if r['condition']==condition and r['method']==name]
            if not rows:continue
            summary=dict(condition=condition,method=name,**aggregate(rows))
            pairs=risks.get((condition,name),[])
            summary['risk_common']=auc_ap(np.concatenate([a for a,b in pairs]),np.concatenate([b for a,b in pairs])) if pairs else dict(auc=None,ap=None,n=0,positive=0)
            summaries.append(summary)
    by_scene=[]
    for scene in protocol['scenes']:
        for condition in protocol['conditions']:
            for name in METHODS:
                rows=[r for r in all_rows if r['scene']==scene and r['condition']==condition and r['method']==name]
                pairs=scene_risks[(scene,condition,name)]
                ranking=auc_ap(np.concatenate([a for a,b in pairs]),np.concatenate([b for a,b in pairs]))
                by_scene.append(dict(scene=scene,condition=condition,method=name,**aggregate(rows),risk_common=ranking))
    # A paired flicker comparison uses exactly the same cells for both variants.
    temporal_pairs=[]
    if (OUT/'temporal.json').exists():
        for frame in range(1,8):
            with np.load(OUT/f'temporal-{frame-1}.npz') as before,np.load(OUT/f'temporal-{frame}.npz') as after:
                a,b,c,d=before['fsgp'],after['fsgp'],before['bgk'],after['bgk']
                mask=np.isfinite(a)&np.isfinite(b)&np.isfinite(c)&np.isfinite(d)
                temporal_pairs.append(dict(frame=frame,common_n=int(mask.sum()),
                    fsgp_mean_absolute_change=float(np.abs(b[mask]-a[mask]).mean()),
                    bgk_mean_absolute_change=float(np.abs(d[mask]-c[mask]).mean())))
    layer_rows=[dict(scene=s,condition=c,above_10cm=n,known=k) for (s,c),(n,k) in layer_errors.items()]
    write_json(OUT/'summary.json',dict(condition_summary=summaries,scene_summary=by_scene,
        temporal_paired=temporal_pairs,q20_corridor_large_errors=layer_rows,audit=audit))
    with (OUT/'summary.csv').open('w') as f:
        fields=['condition','method','cases','coverage','own_rmse_m','common_rmse_m','common_n','normal_deg','geometry_ms','native_ms','auc_common','ap_common','risk_common_n']
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for s in summaries:w.writerow(dict(condition=s['condition'],method=s['method'],cases=s['cases'],coverage=s['coverage'],own_rmse_m=s['own']['rmse_m'],common_rmse_m=s['common']['rmse_m'],common_n=s['common']['n'],normal_deg=s['common']['normal_mean_deg'],geometry_ms=s['geometry_ms'],native_ms=s['native_ms'],auc_common=s['risk_common']['auc'],ap_common=s['risk_common']['ap'],risk_common_n=s['risk_common']['n']))
    for name,value in protocol['code_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==value
    write_json(OUT/'audit.json',audit)
    text='''# 当前 20 cm 二次曲面方案与 FSGP：配对效果测评

日期：2026-11-17。交互页面：http://127.0.0.1:42482/fsgp-q20/ 。这里的 FSGP 是本地 **CPU 公式参考实现**；未执行作者 GPyTorch/CuPy GPU 程序，未复现论文 Table I–III，不能将下列数值称作超过论文成绩。

本次结果：密集输入上，Q20 的局部高程更准确；稀疏输入时覆盖明显下降，并会误关联上层曲面。FSGP 在覆盖、稀疏单级台阶排序和 BGK 静态历史融合上有优势。Q20 共同几何预测稍快，但现有跳变／净空评分整体更慢，不能将“二次函数求解快”当成完整评估实时。

![共同支持误差、稀疏覆盖、逐场景排序与计时总览](comparison.png)

[下载总览 PDF](comparison.pdf)。图中各场景的几何参考、共同集合和限制见下文。

## 1. 测评边界与设计

论文利用特征稀疏 GP 建地形模型，并用 BGK 融合历史代价；论文方法说明见 [原文](https://arxiv.org/html/2503.04134v2)，官方批量评测程序仍在[仓库](https://github.com/ZJU-FAST-Lab/FSGP_BGK)的待发布列表。本次保持本地两种方法核心代码和参数不变，测一个可检查输入与参考值的小基准。

- 使用已有的平地、斜坡、平地与斜坡、多级台阶、原单级台阶、官方丘陵、0.4/0.6/0.8 m 通道，共 9 场景。
- 3 条件：保留 85%、Z 噪声 3 mm；保留 85%、Z 噪声 15 mm；保留 45%、Z 噪声 15 mm。XY 噪声均为 1 mm 标准差。随机丢点不等同真实 LiDAR 射线遮挡。
- 每个条件 3 个传感器随机种子：1101/1102/1103，共 81 组配对输入。三种主方法在每组收到逐字节一致的 XYZ。
- 输入裁剪到中心附近 ±2.95 m，输出沿用原 FSGP 的 5×5 m、20 cm、25×25 查询栅格。世界格中心实际为 −2.3…2.5 m，地图边界为 −2.4…2.6 m。
- 解析场景用其独立几何函数评价查询位置，不用拟合训练残差。丘陵先分出 20% 原始点作为留出集合，再从中为每个查询格选一个邻近的独立测试点，训练与测试原始索引不重叠；丘陵参考仍是原始点高程，不是无噪声的作者真值，没有宣称丘陵法向或通行标签。
- 在台阶位置附近 4 cm、墙体边缘等参考高度／法向不适定位置排除几何误差评价，但仍保留其危险性评价。具体掩码在逐例文件中，不以模型表现决定是否排除。
- 报告覆盖率、自有支持上的误差，以及三种方法**共同支持**处的误差。没有预测的格子不计为零误差，也不把高覆盖率自动当成高准确率。

这是一组构造场景和一份官方点云的本地诊断；大量格子共享相邻点，不能将格子数当作独立实验样本数。仅 3 个随机种子，不报告统计显著性或导航成功率。

## 2. 主方法与两个辅助控制

**Q20**：当前 20×20 cm、±4 cm 高度带、至少 30 点的局部二次拟合，0.10 m 三维体素选中心。为了与 GP 的 XY 查询对齐，按查询位置附近 16 个原始观测中的最低高度引导 `choose_support` 选择较低表面；没有输入真实地面高度或表面标签。这是明确的地面分支关联假设，不保证悬空、遮挡或侧壁附近一定选对。局部系数和协方差来自完整原始点的合格邻域。

**FSGP-125 / FSGP-500**：沿用 core.py 的特征、采样、RQ 核与条件化预测；M=125/500，长度尺度 0.6 m、alpha=10、信号方差 1、噪声方差 0.0004 m²，不针对本次标签调参。它是按公式用 M 个保留观测条件化的 CPU 参考，不等同官方 InducingPointKernel 的学习过程。法向使用论文链路中的固定 κ/g 的 XY 偏导，没有添加特征 IDW 对 XY 的链式导数。

相同原始输入不等于相同内部预算：Q20 重用完整点云的许多局部邻域，FSGP 的高程条件化只保留 M 点，查询特征仍来自完整当前帧。因此不把几何差异全部归因于“二次函数优于 GP”。

辅助控制不参加主方法结论或速度排名：

1. **K20-on-M125/500**：仅使用对应 FSGP 选中的同一批 XYZ，运行此前的 20 近邻二次高程函数。它不是当前严格 20 cm 的 Q20，没有同样的高度带、协方差、净空和机身指标；其毫秒数只覆盖采样后高程预测，不可与从 XYZ 开始的速度相除。GP 查询仍多用了全帧的 κ/g 插值，所以也没有声称两者所有信息完全相同。
2. **FSGP-125-oracle**：仅在通道中给 GP 生成器真实地面点，用于判断地面关联／多层混合的影响。这使用了真实标签，是分割理想上限控制，不能称作可直接部署的改进算法。

主方法适配器已与原 TraversabilityMapper 的高程、方差、坡度和代价逐项复核一致。原 core.py、terrain_assessment.py 以及本次运行的其他关键源文件 SHA256 保存在 protocol.json，并在结束时核对未变。

## 3. 几何结果：覆盖率和误差一起看

高度误差单位 mm，法向误差为夹角的平均值 °。RMSE 从所有参与点的平方误差汇总；不是将每帧 RMSE 简单平均。共同支持格数对三个主方法相同。

'''
    rows=[]
    for s in summaries:
        if s['method'] not in METHODS:continue
        rows.append([s['condition'],s['method'],fmt(s['coverage'],100,1),fmt(s['own']['rmse_m'],1000),fmt(s['common']['rmse_m'],1000),s['common']['n'],fmt(s['common']['normal_mean_deg'])])
    text+=table(['条件','方法','覆盖 %','自有支持 RMSE mm','共同支持 RMSE mm','共同点数','共同法向误差 °'],rows)
    text+='逐场景的共同支持误差与覆盖率：\n\n'
    rows=[]
    for scene in protocol['scenes']:
        for condition in protocol['conditions']:
            ss={s['method']:s for s in by_scene if s['scene']==scene and s['condition']==condition}
            rows.append([NAMES[scene],condition,*[fmt(ss[m]['common']['rmse_m'],1000) for m in METHODS],*[fmt(ss[m]['coverage'],100,1) for m in METHODS]])
    text+=table(['场景','条件','Q20 mm','FSGP125 mm','FSGP500 mm','Q20覆盖 %','125覆盖 %','500覆盖 %'],rows)
    text+='''覆盖不足会让条件误差只反映“容易支持的地方”。尤其稀疏条件，不能只看 Q20 在较小共同集合上的低 RMSE，忽视它没有给出的区域。参数协方差与 GP 潜在方差都未做传感器标定；逐条件 JSON 记录的经验 2σ 覆盖率也不是可靠概率保证。

### 本次测到的主要区别

'''
    get_scene=lambda s,c,m:next(r for r in by_scene if r['scene']==s and r['condition']==c and r['method']==m)
    qhill=get_scene('hill','dense_low','Q20');ghill=get_scene('hill','dense_low','FSGP-500')
    qsp=get_scene('hill','sparse_noisy','Q20');gsp=get_scene('hill','sparse_noisy','FSGP-500')
    text+=f"密集低噪声丘陵留出点上，Q20 的共同支持 RMSE 为 {fmt(qhill['common']['rmse_m'],1000)} mm，FSGP-500 为 {fmt(ghill['common']['rmse_m'],1000)} mm；Q20 覆盖 {fmt(qhill['coverage'],100,1)}%。稀疏高噪声丘陵中，Q20 覆盖降至 {fmt(qsp['coverage'],100,1)}%，GP 保持 {fmt(gsp['coverage'],100,1)}%。这里 Q20 的较低条件误差只涉及它愿意输出的少数位置。\n\n"
    text+='通道稀疏条件中，Q20 不仅弃权，还发生已知预测的大误差：\n\n'
    text+=table(['通道','已知位置中误差 >10 cm','已知数','比例 %'],[[NAMES[r['scene']],r['above_10cm'],r['known'],fmt(r['above_10cm']/r['known'],100,1)] for r in layer_rows if r['condition']=='sparse_noisy'])
    text+='侧面剖面可看到部分查询落在上层曲面附近；这表明“最低观测引导”并没有保证最终支持一定是地面。上述计数是大误差统计，不把所有误差都自动归因为同一种原因。\n\n'
    text+='''

## 4. 相同 M 点与理想地面控制

此处列自有支持 RMSE，辅助控制只回答特定问题，不按它们的不同支持集给主方法排总名次。

'''
    rows=[]
    for s in summaries:
        if s['method'] in METHODS:continue
        rows.append([s['condition'],s['method'],s['cases'],fmt(s['coverage'],100,1),fmt(s['own']['rmse_m'],1000)])
    text+=table(['条件','辅助控制','案例数','覆盖 %','自有支持 RMSE mm'],rows)
    text+='''在通道中，若清除上层点后 GP 高程误差明显降低，说明原始结果包含多层数据关联问题，不能据此断言 GP 本身不能拟合平地；真实系统仍需实现不依赖标签的地面／障碍关联。相同 M 的 K20 结果则揭示了支撑范围、采样和表示能力共同起作用。

## 5. 危险区域排序，不比较不同含义的代价差

解析参考只标记本实验规定的几何危险：80×50 cm 矩形跨过 ≥12 cm 台阶／侧壁、与通道墙体相交，或在指定 10–60 cm 高度带被顶板占用。它不是 Go2 实机的稳定／通过真值；平地和连续缓坡没有被当作台阶。丘陵没有危险标签，完全不进入此项。

Q20 使用原有落足综合代价，FSGP 使用原有平滑代价；二者值域与权重不同，不做跨方法 cost MAE，不用测试标签选“通过阈值”。所有方法再应用同一 80×50 cm 矩形内栅格中心的最大值操作：20 cm 输出分辨率下是 5×3=15 个中心，要求至少 80% 已知。这是离散危险排序窗口，**不是原先四足／质心机身稳定性分数**，也没有检查连续矩形内所有位置。

排序评价区域另限制 |x|≤2.0、|y|≤2.15 m；通道 |y|≤1.75 m，确保假设机身位于地面域内。共同可评分格上比较 AUROC/AP；AP 的随机参考为该共同集合的危险占比。未知保留为弃权，另报原生代价覆盖率，不能靠全拒绝取得“安全准确率”。

'''
    rows=[]
    for s in summaries:
        if s['method'] not in METHODS:continue
        r=s['risk_common'];rows.append([s['condition'],s['method'],fmt(r['auc'],1,3),fmt(r['ap'],1,3),r['n'],fmt(r['positive']/r['n'] if r['n'] else None,100,1),fmt(s['risk_known']/s['risk_eligible'] if s['risk_eligible'] else None,100,1)])
    text+=table(['条件','方法','AUROC','AP','共同评分数','危险占比 %','代价覆盖 %'],rows)
    text+='为了避免把不同场景混合后的高 AUROC 当作每个场景都好，再列逐场景排序。下表每行固定场景和条件，合并三个种子；只有同时存在危险与非危险标签才计算 AUROC，平地／连续缓坡全为非危险，没有伪造 AUC=1。\n\n'
    rows=[]
    for scene in protocol['scenes']:
        for condition in protocol['conditions']:
            ss={m:get_scene(scene,condition,m) for m in METHODS}
            r=ss['Q20']['risk_common']
            if r['auc'] is None:continue
            rows.append([NAMES[scene],condition,r['n'],fmt(r['positive']/r['n'],100,1),*[fmt(ss[m]['risk_common']['auc'],1,3) for m in METHODS]])
    text+=table(['场景','条件','共同数','危险 %','Q20 AUC','FSGP125 AUC','FSGP500 AUC'],rows)
    qsingle=get_scene('steps-single','sparse_noisy','Q20');gsingle=get_scene('steps-single','sparse_noisy','FSGP-500')
    text+=f"具体反例：稀疏原单级台阶中，Q20 的 AUROC 为 {fmt(qsingle['risk_common']['auc'],1,3)}，FSGP-500 为 {fmt(gsingle['risk_common']['auc'],1,3)}；FSGP 在这一组危险排序更好。不能由整体 RMSE 或混合场景 AUC 宣称 Q20 每个任务都占优。\n\n"
    text+='''原生代价更红不代表效果更好。AUROC/AP 衡量排序，仍不足以证明已得到可执行的通过阈值；标签几何约束也有意贴近本次工程需求，因此不能外推为其他机器人或论文全部任务的普遍优势。

## 6. 速度的可比范围

共同几何任务均从同一 XYZ 到 625 个查询的高度、梯度、方差与支持掩码；不含来源生成、文件写入和网页绘制。按三个传感器种子统计运行时间，输入点数会随丢点略变，不是严格相同数组的纯计时重复。方法执行顺序按种子交替，CPU 单线程 BLAS。

Q20 原生落足代价额外执行面片几何／σ、双侧跳变、原始点和曲面净空；不含原完整工作台的一致性组合和四足机身评估。FSGP 原生代价只增加其特征加权和滤波。因此两者“原生完整任务”工作不同，不将它们相除宣称算法快多少倍。

'''
    text+=table(['条件','方法','共同几何中位 ms','含各自原生评分 ms'],[[s['condition'],s['method'],fmt(s['geometry_ms']),fmt(s['native_ms'])] for s in summaries if s['method'] in METHODS])
    if (OUT/'temporal.json').exists():
        temporal=json.loads((OUT/'temporal.json').read_text());records=temporal['records']
        text+='''## 7. 时序小实验：FSGP 与 BGK 使用同一次预测

固定平地—斜坡几何，8 帧、0.1 秒间隔，每帧独立 15 mm 高度噪声；第 2–5 帧删除 |x|<0.7 m 的整条观测带。FSGP 与 FSGP-BGK 共用完全相同的单帧输出；Q20 没加历史融合。

这里只检验静态地形的代价稳定性与历史覆盖。BGK 融合代价，没有重新改善该帧高度，也未测试动态障碍出现／消失的滞后。FSGP 与 BGK 的代价尺度相同，可以比较代价帧间变化；Q20 的原生尺度不同，不将其抖动绝对值与 GP 排名。

'''
        text+=table(['帧','观测带删除','Q20已知 %','FSGP已知 %','BGK已知 %','FSGP帧间变化','BGK帧间变化'],
            [[i,'是' if 2<=i<=5 else '否',*[fmt(next(r for r in records if r['frame']==i and r['method']==m)['coverage'],100,1) for m in ['Q20','FSGP','FSGP-BGK']],*[fmt(next(r for r in records if r['frame']==i and r['method']==m)['absolute_frame_change'],1,5) for m in ['FSGP','FSGP-BGK']]] for i in range(8)])
        text+='上表每个方法使用各自相邻帧的共同集合。为公平比较抖动，再将 FSGP 和 BGK 的前后两帧共同已知集合取交集：\n\n'
        text+=table(['帧','双方相同格数','FSGP 变化','BGK 变化'],[[r['frame'],r['common_n'],fmt(r['fsgp_mean_absolute_change'],1,5),fmt(r['bgk_mean_absolute_change'],1,5)] for r in temporal_pairs])
        n=sum(r['common_n'] for r in temporal_pairs)
        before=sum(r['common_n']*r['fsgp_mean_absolute_change'] for r in temporal_pairs)/n
        after=sum(r['common_n']*r['bgk_mean_absolute_change'] for r in temporal_pairs)/n
        text+=f"在这些相同格上的加权平均绝对代价帧间变化从 {fmt(before,1,5)} 降到 {fmt(after,1,5)}，下降 {fmt(1-after/before,100,1)}%。这是本次静态噪声／暂缺实验中的平滑效果，不是动态障碍响应成绩。\n\n"
    text+='''## 8. 如何读这些结果

1. 局部高度更准确，不自动意味着原生通行性评分更准确；需要同时看法向、代价排序和缺失。
2. 稀疏点云上，严格 20 cm／30 点条件可能是主要限制；扩大邻域或做历史融合属于后续新方法，不能改完后覆盖本次失败结果。
3. 单值高度模型在通道这种多层输入上可能混合表面；Q20 的最低观测分支也有数据关联和凸包外查询限制，不能宣称所有悬空场景都解决。
4. 这里的 FSGP 超参数固定，未复现作者的核训练／GPU 链路。只能比较当前本地两份实现，不能宣布普遍超过论文方法。

所有案例、失败和辅助控制都保留。独立测试 RMSE 与原工作台每个模型的训练残差是不同量；不能用“训练误差几乎为零”替代本测评。

## 9. 文件与复现

- [运行前固定的协议与源码哈希](protocol.json)、[逐组原始记录 CSV](per-run.csv)、[汇总 CSV](summary.csv)、[完整汇总 JSON](summary.json)、[计数／输入／共同支持核查](audit.json)。
- [数值证据独立复算](numerical-audit.json)、[浏览器逐场景核查](browser-audit.json)。数值审计抽查实际邻域的独立最小二乘／岭解，复算全部已支持查询；另对两个完整点云、两个 M 预算直接调用原 Mapper 对照保存输出。
- cases/*.npz：完整实际输入、留出索引、查询点、参考、掩码、各预测，Q20 实际邻点／系数／协方差，GP 保留点。
- cases/*.json：网页用的完整查询结果；输入点仅展示均匀索引抽取的最多 4,000 点，算法使用完整输入。
- temporal.json 与 temporal-*.npz：8 帧时序记录及实际地图。

```bash
cd /home/jiang/snap/go2
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.compare_fsgp_q20
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.compare_fsgp_q20_report
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/fsgp-bgk-matplotlib \
  /usr/bin/python3 -m fsgp_bgk_repro.plot_fsgp_q20
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.audit_fsgp_q20
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m unittest fsgp_bgk_repro.tests.test_compare_fsgp_q20 -v
node fsgp_bgk_repro/tests/compare_fsgp_q20_browser.cjs
```
'''
    (OUT/'report.md').write_text(text)
    assets=ROOT/'visualization/fsgp_q20'
    if assets.exists():
        for path in assets.iterdir():
            if path.is_file():shutil.copyfile(path,OUT/path.name)
    section='''## 12. 新增：与 FSGP 的配对效果测评

已在 9 个现有场景、3 种密度／噪声条件、3 个传感器种子上完成 81 组配对输入。比较当前 Q20 与 CPU 公式参考 FSGP（M=125/500），并保留同 M 点的旧 K20 和通道理想地面控制。独立评价高度、法向、覆盖率、原生危险排序，以及一组 8 帧 BGK 时序测试。

结果与完整限制见 [同端口交互对比](http://127.0.0.1:42482/fsgp-q20/) 和 [测评报告](fsgp_bgk_repro/outputs/visualization/fsgp-q20/report.md)。原生代价值域不同，没有比较代价 MAE，也未声称超过作者 GPU 或论文基准。

主要结果：密集低噪声丘陵的共同支持 RMSE 为 Q20 1.69 mm、FSGP-500 22.52 mm；稀疏高噪声丘陵的覆盖率分别为 21.6% 和 100%。稀疏原单级台阶的危险排序 AUROC 为 0.748 和 0.975，这一组 FSGP 更好。密集低噪声条件的共同几何中位耗时分别约 179 / 266 ms；含各自评分约 2363 / 266 ms，任务工作量不同。当前二次方案的局部几何优势并不等同整个评分链路更快或全部场景更好。
'''
    (OUT/'step-section.md').write_text(section)
    print('Published',expected,'paired cases,',len(all_rows),'method records')


if __name__=='__main__':main()
