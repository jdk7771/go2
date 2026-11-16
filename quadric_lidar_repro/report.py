"""Scientific figures and a local, inspectable HTML/Markdown experiment report."""

import csv
from html import escape
import json
from pathlib import Path
import shutil

import numpy as np

ROOT=Path(__file__).resolve().parent


def create_report(output,metrics,protocol):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    output=Path(output)
    with np.load(output/'trajectories.npz') as data:
        gt,odo,mapping=[data[name] for name in ['ground_truth','odometry','mapping']]
    with (output/'per_frame.csv').open() as stream: rows=list(csv.DictReader(stream))
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for points,name,color in [(gt,'Ground truth','#555555'),(odo,'Quadric odometry','#d2742c'),(mapping,'Quadric mapping','#246b9d')]:
        axes[0,0].plot(points[:,0,3],points[:,1,3],label=name,color=color)
        axes[0,1].plot(points[:,2,3],label=name,color=color)
        if name!='Ground truth':
            delta=np.linalg.norm(points[:,:3,3]-gt[:,:3,3],axis=1)
            axes[1,0].plot(delta,label=name,color=color)
    axes[0,0].set(xlabel='LiDAR world x / m',ylabel='LiDAR world y / m',aspect='equal')
    axes[0,0].set_title('轨迹俯视；仅首帧对齐，未作尺度/轨迹拟合',fontproperties=font)
    axes[0,1].set(xlabel='frame',ylabel='world z / m')
    axes[0,1].set_title('高度轨迹',fontproperties=font)
    axes[1,0].set(xlabel='frame',ylabel='position error / m')
    axes[1,0].set_title('无后对齐的位置误差',fontproperties=font)
    for key,name,color in [('odometry_total_ms','odometry','#d2742c'),('mapping_total_ms','mapping','#246b9d')]:
        axes[1,1].plot([int(r['frame']) for r in rows],[float(r[key]) for r in rows],label=name,color=color)
    axes[1,1].set(xlabel='frame',ylabel='algorithm time / ms')
    axes[1,1].set_title('含特征提取/关联/优化；mapping 含地图更新',fontproperties=font)
    for ax in axes.ravel(): ax.grid(alpha=.15);ax.legend(fontsize=8)
    fig.suptitle(f'KITTI {protocol["sequence"]} · {protocol["frames"]} 帧 · CPU 独立实现',fontproperties=font)
    fig.savefig(output/'trajectory.png',dpi=160);plt.close(fig)

    with np.load(output/'first_scan_patches.npz') as data:
        p,labels,centers=data['points'],data['labels'],data['centers']
    fig=plt.figure(figsize=(11,6),constrained_layout=True)
    ax=fig.add_subplot(111,projection='3d')
    for code,name,color in [(0,'Plane','#306b97'),(1,'Quadric','#df963a'),(2,'Distribution','#98598c')]:
        selected=labels==code
        if selected.any():ax.scatter(*p[selected].T,c=color,s=1,alpha=.65,label=name,rasterized=True)
    ax.set(xlabel='LiDAR x / m',ylabel='LiDAR y / m',zlabel='LiDAR z / m')
    ax.set_box_aspect((2.1,1.5,.65));ax.view_init(elev=23,azim=-125);ax.legend(markerscale=4)
    ax.set_title(f'首帧拟合类型 · {len(centers)} 个面片；显示的是采样观测点及所属类型',fontproperties=font)
    fig.savefig(output/'patches.png',dpi=160);plt.close(fig)

    loc=[]
    if (output/'localization.csv').exists():
        with (output/'localization.csv').open() as stream:loc=list(csv.DictReader(stream))
    if loc:
        fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
        x=[int(r['frame']) for r in loc]
        axes[0].plot(x,[float(r['initial_xy_noise_m']) for r in loc],'o--',label='initial XY perturbation')
        axes[0].plot(x,[float(r['translation_error_m']) for r in loc],'o-',label='after registration')
        axes[0].set(xlabel='query frame',ylabel='translation error / m')
        axes[1].plot(x,[abs(float(r['initial_yaw_noise_deg'])) for r in loc],'o--',label='initial yaw perturbation')
        axes[1].plot(x,[float(r['rotation_error_deg']) for r in loc],'o-',label='after registration (SO3)')
        axes[1].set(xlabel='query frame',ylabel='rotation error / degree')
        for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.15)
        fig.suptitle('保存地图后的定位；初值按论文协议由真值加噪声生成',fontproperties=font)
        fig.savefig(output/'localization.png',dpi=160);plt.close(fig)
    paper={'odometry':(1.66,1.22),'mapping':(.87,.51)}
    lines=['# Quadric LiDAR 论文方法独立复现','',
           '论文：[Quadric Representations for LiDAR Odometry, Mapping and Localization](https://arxiv.org/abs/2304.14190v1)。',
           '没有找到作者公开的代码仓库；本目录是根据论文公式编写的 CPU 参考实现，不是作者代码。','',
           f'实测 KITTI {protocol["sequence"]}，共 {protocol["frames"]} 帧；完整官方短序列约 393.65 m。',
           '点云和真值来自 KITTI 官方公开存储；原始归档按 HTTP Range 选取该序列，不下载其余 84 GB 数据。','',
           '## 实测与论文对应序列','',
           '| 模块 | 本地平移误差 % ↓ | 论文序列04平移 % | 本地旋转误差 °/100m ↓ | 论文序列04旋转 | 本地中位 ms/帧 |',
           '|---|---:|---:|---:|---:|---:|']
    for name in ['odometry','mapping']:
        m=metrics[name]
        lines.append(f'| {name} | {m["kitti_translation_error_percent"]:.3f} | {paper[name][0]} | '
                     f'{m["kitti_rotation_error_deg_per_100m"]:.3f} | {paper[name][1]} | {metrics["timing"][name+"_total_ms"]:.1f} |')
    variants=[{'variant':'Schur LS + normal gate + Cauchy (current)',
               'odometry_percent':metrics['odometry']['kitti_translation_error_percent'],
               'mapping_percent':metrics['mapping']['kitti_translation_error_percent']}]
    for folder,label in [('taubin_stability','Taubin + normal gate + Cauchy'),('literal_initial','Taubin, direct formula residuals')]:
        path=output.parent/folder/'metrics.json'
        if path.exists():
            record=json.loads(path.read_text())
            variants.append({'variant':label,'odometry_percent':record['odometry']['kitti_translation_error_percent'],
                             'mapping_percent':record['mapping']['kitti_translation_error_percent']})
    (output/'variant_comparison.json').write_text(json.dumps(variants,indent=2)+'\n')
    lines+=['','上表是 KITTI 100–800 m 子段、每10帧起点的相对位姿指标；本序列仅足够生成100/200/300 m子段。',
            '论文将这两个指标记为 ATE(%) / ARE(°/100m)，这里明确标为相对误差，避免与绝对轨迹 APE 混淆。',
            '每个子段用未修改的官方 `calcSequenceErrors` 独立交叉验证；没有用刚体/尺度后对齐掩盖漂移。',
            '**本地精度尚未对齐论文，不能称为已复现论文实验指标。**','','![轨迹与误差](trajectory.png)','',
            '## 实现范围','',
            '- spherical range image → patch segmentation → plane / quadric / distribution fitting。',
            '- Taubin点到二次曲面残差、点到分布残差、式(7)加权面片关联、解析SE(3)雅可比和LM。',
            '- scan-to-scan里程计、scan-to-submap建图、按式(8)/(9)更新矩统计；目标地图不保留历史原始点。',
            '- 1.5 m空间哈希、最低MSE冲突保留规则、地图保存和重载后的scan-map定位。','',
            '## 与论文未指定细节/作者实现的差异','']
    lines.extend('- '+item for item in protocol['deviations'])
    lines+=['','## 已保留的同序列拟合/稳定性对照','',
            '| 版本 | 里程计平移误差 % | 建图平移误差 % |','|---|---:|---:|']
    for r in variants:lines.append(f'| {r["variant"]} | {r["odometry_percent"]:.3f} | {r["mapping_percent"]:.3f} |')
    lines+=['','所有版本都没有达到论文精度。这里是同一序列上的方法诊断，不是未见数据上的独立验证。']
    lines+=['',
            '默认版本在关联中增加近似平面法向一致性门限，并使用Cauchy损失以抑制错误匹配。',
            '这些是明确列出的稳定性扩展，不属于已验证的作者原实现；`--literal`可关闭它们。',
            '首次无这两项扩展的完整序列结果保存在 `../literal_initial/`，并保存对应源代码快照。','',
            '![首帧面片](patches.png)','',
            '## 地图和定位','',
            f'保存 {metrics["storage"]["serialized_map_patches"]} 个面片，NPZ {metrics["storage"]["serialized_map_bytes"]:,} bytes。',
            f'相对完整输入的XYZ float32载荷，文件尺寸比约 {metrics["storage"]["raw_xyz_to_npz_ratio"]:.1f} 倍。',
            '这个比值同时包含几何近似和NPZ压缩，不是内存峰值，也不是论文1508.9倍结果的复现。']
    if loc:
        valid=sum(int(r['selected_points'])>=12 and int(r['accepted_steps'])>0 for r in loc)
        improved=sum(float(r['translation_error_m'])<float(r['initial_xy_noise_m']) for r in loc)
        lines += ['',f'定位查询 {len(loc)} 次，有匹配并接受优化步 {valid} 次，平移误差低于初始扰动 {improved} 次。',
                  f'最终平移误差中位数 {metrics["localization"]["translation_error_m"]:.3f} m；'
                  f'旋转误差中位数 {metrics["localization"]["rotation_error_deg"]:.3f}°。',
                  '定位初值按照论文由真值+XY±0.2m、yaw±5°生成；仅用于定位实验。',
                  '里程计和建图未读取真值。定位地图由估计轨迹建立，其漂移会影响定位结果；不把接近真值的初始先验当作定位成功。','',
                  '![保存地图定位](localization.png)']
    lines+=['','## 检查和证据','',
            '`per_frame.csv`：实际面片数、选点数、优化步数、逐阶段耗时。',
            '`*_camera.txt`：预测和真值的相机坐标轨迹，可直接用KITTI devkit评测。',
            '`*_segment_errors.csv`：逐子段误差；`metrics.json`：汇总；`protocol.json`：参数、数据和代码哈希。',
            '`quadric_map.npz`：origin、55项对称矩统计和点数，不含历史点云。',
            '单元检查包括平面、球面、两类残差雅可比、矩统计变换/增量合并、已知SE(3)配准及地图保存重载。','']
    (output/'REPORT.md').write_text('\n'.join(lines))
    metric_rows=''.join(f'<tr><td>{name}</td><td>{metrics[name]["kitti_translation_error_percent"]:.3f}%</td>'
                        f'<td>{metrics[name]["kitti_rotation_error_deg_per_100m"]:.3f}°/100m</td>'
                        f'<td>{metrics["timing"][name+"_total_ms"]:.1f} ms</td></tr>' for name in ['odometry','mapping'])
    variant_rows=''.join(f'<tr><td>{escape(r["variant"])}</td><td>{r["odometry_percent"]:.3f}%</td><td>{r["mapping_percent"]:.3f}%</td></tr>' for r in variants)
    loc_valid=sum(int(r['selected_points'])>=12 and int(r['accepted_steps'])>0 for r in loc)
    status_text='轨迹发散，论文精度复现未成功。' if metrics['mapping']['kitti_translation_error_percent']>100 else '精度尚未达到论文。'
    html=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quadric LiDAR · KITTI {escape(protocol['sequence'])} 实测</title><style>
body{{margin:0;background:#f6f7f9;color:#23313d;font:15px/1.8 system-ui,sans-serif}}main{{max-width:1100px;margin:auto;padding:30px 22px}}h1{{font-size:28px;margin:15px 0}}h2{{font-size:20px;margin:30px 0 10px}}p{{color:#536371}}table{{width:100%;border-collapse:collapse;background:white}}td,th{{text-align:left;padding:12px;border-bottom:1px solid #dce1e6}}figure{{margin:20px 0}}img{{width:100%;height:auto}}a{{color:#246b9d}}.status{{background:#fff0d5;padding:16px;border-radius:8px}}details{{margin:20px 0}}code{{overflow-wrap:anywhere}}@media(max-width:600px){{main{{padding:20px 12px}}h1{{font-size:23px}}td,th{{padding:8px 4px;font-size:12px}}}}</style>
<main><a href="../">← 已有实验</a><p>RA-L 2023 / arXiv:2304.14190</p><h1>Quadric LiDAR：里程计、建图与定位</h1>
<p>KITTI {escape(protocol['sequence'])} · {protocol['frames']} 帧 · 官方原始点云与真值 · 独立 CPU 实现</p>
<p class="status">{status_text} 已运行完整流程；默认版本包含明确列出的法向关联门限与鲁棒损失，不是作者代码。</p>
<table><thead><tr><th>模块</th><th>平移相对误差 ↓</th><th>旋转相对误差 ↓</th><th>中位耗时</th></tr></thead><tbody>{metric_rows}</tbody></table>
<p>用官方 KITTI 函数逐子段交叉验证；轨迹只对齐首帧，没有进行尺度或刚体后对齐。CPU耗时不能直接与作者实现比较。</p>
<figure><img src="trajectory.png" alt="真实、里程计及建图轨迹，位置误差与逐帧耗时"></figure>
<h2>同序列诊断对照</h2><table><thead><tr><th>版本</th><th>里程计平移误差</th><th>建图平移误差</th></tr></thead><tbody>{variant_rows}</tbody></table><p>全部版本均未达到论文精度；这是同一序列上的诊断，未将表现较好的版本当作论文复现结果。</p>
<h2>二次曲面表示</h2><p>面片保存隐式二次函数、均值和矩统计；增量更新不保留历史原始点云。蓝色为平面，橙色为一般二次曲面。</p>
<figure><img src="patches.png" alt="首帧采样点及其拟合面片类型"></figure>
<h2>保存地图后的定位</h2><p>初值按论文由真值加XY±0.2 m、yaw±5°扰动生成。里程计/建图不使用真值；定位结果另受估计地图漂移影响。本轮有匹配并接受优化步的查询为 {loc_valid}/{len(loc)}，不能将未优化的初始先验精度当作定位成功。</p>
<figure><img src="localization.png" alt="定位前后的平移和旋转误差"></figure>
<details><summary>实现约定与差异</summary><ul>{''.join('<li>'+escape(x)+'</li>' for x in protocol['deviations'])}</ul></details>
<p><a href="REPORT.md">完整报告</a> · <a href="metrics.json">实测指标</a> · <a href="protocol.json">参数与来源</a> · <a href="per_frame.csv">逐帧数据</a> · <a href="quadric_map.npz">二次曲面地图</a></p>
<p><a href="https://arxiv.org/abs/2304.14190v1">论文</a> · <a href="https://www.cvlibs.net/datasets/kitti/eval_odometry.php">KITTI 官方评测</a></p></main></html>'''
    (output/'index.html').write_text(html)
    existing_viewer=ROOT.parent/'fsgp_bgk_repro/outputs/visualization'
    if existing_viewer.exists():
        target=existing_viewer/'quadric-lidar'
        target.mkdir(exist_ok=True)
        for name in ['index.html','trajectory.png','patches.png','localization.png','REPORT.md','metrics.json','protocol.json','per_frame.csv','quadric_map.npz']:
            if (output/name).exists():shutil.copyfile(output/name,target/name)
