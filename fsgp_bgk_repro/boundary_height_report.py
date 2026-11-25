"""Audit same-XY height differences and export scientific overview/section plots."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection,LineCollection
from matplotlib.colors import LinearSegmentedColormap,Normalize
from matplotlib.font_manager import FontProperties

from .assessment_export import sha,finite_json
from .boundary_heights import REPORT,predict,COLOR_MAX
from .convex_surface_assessment import OUT,FIELDS,inside
from .hill_small_gaps import load_models
from .polygon_topology import PolygonTopology


def audit(meta,folder):
    original=OUT/meta['profile']/meta['scene']
    assert all(sha(original/name)==digest for name,digest in meta['original_files_sha256'].items())
    model=np.fromfile(original/'models.bin','<f8').reshape(-1,len(FIELDS))
    records=meta['records'];assert meta['new_fits']==0
    for r in records:
        assert model[r['left'],FIELDS.index('valid')] and model[r['right'],FIELDS.index('valid')]
        delta=abs(predict(model,r['left'],r['xy'])-predict(model,r['right'],r['xy']))
        np.testing.assert_allclose(delta,r['delta_m'],rtol=1e-12,atol=1e-12)
        assert r['left_region']!=r['right_region'] and r['boundary_xy_gap_m']<=.1+1e-8
    positions=np.fromfile(folder/'lines.bin','<f4').reshape(-1,3).astype(float)
    values=np.fromfile(folder/'values.bin','<f4').astype(float)
    ids=np.repeat(np.fromfile(folder/'line-records.bin','<u4'),2)
    left=np.array([r['left'] for r in records],int)[ids];right=np.array([r['right'] for r in records],int)[ids]
    a,b=predict(model,left,positions[:,:2]),predict(model,right,positions[:,:2])
    height_error=np.minimum(abs(positions[:,2]-a),abs(positions[:,2]-b)).max(initial=0)
    color_error=np.abs(values-abs(a-b)).max(initial=0)
    assert max(height_error,color_error)<5e-5 and np.isfinite(positions).all()
    return dict(scene=meta['scene'],profile=meta['profile'],samples=len(records),
        source_hashes_verified=len(meta['original_files_sha256']),same_xy_difference_verified=True,
        maximum_float32_height_error_m=float(height_error),maximum_float32_delta_error_m=float(color_error))


def plot(meta):
    _,_,model,active,polygons=load_models(meta['profile'],meta['scene']);records=meta['records']
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():plt.rcParams.update({'font.family':FontProperties(fname=str(font)).get_name(),'axes.unicode_minus':False})
    cmap=LinearSegmentedColormap.from_list('height_difference',['white','red']);norm=Normalize(0,100*COLOR_MAX,clip=True)
    fig,(top,section)=plt.subplots(1,2,figsize=(15,6),gridspec_kw={'width_ratios':[1,1.25]},constrained_layout=True)
    top.set_facecolor('#dfe6ed');top.add_collection(PolyCollection(polygons,facecolors='#2586cb',edgecolors='none'))
    gap=OUT/'small-gaps'/meta['profile']/meta['scene']
    if (gap/'positions.bin').exists():
        v=np.fromfile(gap/'positions.bin','<f4').reshape(-1,3);f=np.fromfile(gap/'faces.bin','<u4').reshape(-1,3)
        if len(f):top.add_collection(PolyCollection(v[f,:2],facecolors='#2586cb',edgecolors='none'))
    top.add_collection(LineCollection([r['xy_ends'] for r in records],array=np.array([100*r['delta_m'] for r in records]),cmap=cmap,norm=norm,linewidths=2.1))
    xy=np.concatenate(polygons);top.set_xlim(xy[:,0].min()-.2,xy[:,0].max()+.2);top.set_ylim(xy[:,1].min()-.2,xy[:,1].max()+.2)
    top.set_aspect('equal');top.set_xlabel('X (m)');top.set_ylabel('Y (m)');top.set_title('俯视：蓝色拟合显示，白→红为边界高度差')
    cut=-1.6 if meta['scene']=='flat-corridor-platforms' else 0.;curves=[]
    for owner,p in zip(active,polygons):
        if not p[:,1].min()-1e-9<=cut<=p[:,1].max()+1e-9:continue
        x=np.linspace(p[:,0].min(),p[:,0].max(),max(3,int(np.ptp(p[:,0])/.005)+1))
        q=np.c_[x,np.full(len(x),cut)];q=q[inside(q-model[owner,:2],p-model[owner,:2])]
        if len(q)>1:curves.append(np.c_[q[:,0],predict(model,owner,q)])
    section.add_collection(LineCollection(curves,colors='#2586cb',linewidths=2))
    top.axhline(cut,color='#273746',lw=.8,ls='--',alpha=.7)
    selected={}
    for r in records:
        if abs(r['xy'][1]-cut)>.025:continue
        key=round(r['xy'][0]/.1)
        if key not in selected or abs(r['xy'][1]-cut)<abs(selected[key]['xy'][1]-cut):selected[key]=r
    for r in selected.values():
        x=r['xy'][0];low,high=sorted((r['left_z_m'],r['right_z_m']))
        section.plot([x,x],[low,high],color=cmap(norm(100*r['delta_m'])),lw=3,zorder=4)
        section.scatter([x,x],[low,high],s=13,c=[cmap(norm(100*r['delta_m']))],edgecolors='#4f5963',lw=.3,zorder=5)
        section.text(x,high+.035,f"{100*r['delta_m']:.0f} cm",ha='center',fontsize=9,color='#313942')
    section.set_xlim(xy[:,0].min()-.15,xy[:,0].max()+.15)
    high=max((max(r['left_z_m'],r['right_z_m']) for r in records),default=1.)
    section.set_ylim(-.06,high+.16);section.set_xlabel('X (m)');section.set_ylabel('拟合高度 Z (m)')
    section.set_title(f'剖面 Y={cut:g} m：蓝色为各自公式，竖线为同 XY 的 Δz');section.grid(alpha=.18)
    scalar=matplotlib.cm.ScalarMappable(norm=norm,cmap=cmap);bar=fig.colorbar(scalar,ax=[top,section],shrink=.7,pad=.025)
    bar.set_label('边界高度差 (cm)');bar.set_ticks([0,15,40,60,80]);bar.set_ticklabels(['0','15','40','60','≥80'])
    fig.suptitle(f"{meta['name']} · {meta['label']}\n"+r'$\Delta z(q)=|f_i(q)-f_j(q)|$'+'；不是拟合中心高度相减',fontsize=15)
    fig.savefig(REPORT/f"{meta['scene']}.png",dpi=170);plt.close(fig)


def run():
    records=[];audits=[];synthetic_checks=[]
    for folder in sorted(REPORT.glob('*/*')):
        if not (folder/'data.json').exists():continue
        meta=json.loads((folder/'data.json').read_text());audits.append(audit(meta,folder));records.append(meta)
        if meta['profile']=='qcircle-adaptive':
            plot(meta)
            if meta['scene']=='steps':
                from .mixed_steps_scene import BREAKS,HEIGHTS
                checks=[]
                for edge,expected in zip(BREAKS,np.abs(np.diff(HEIGHTS))):
                    found=[r['delta_m'] for r in meta['records'] if abs(r['xy'][0]-edge)<.12 and abs(r['xy'][1])<1.]
                    assert found and np.max(np.abs(np.array(found)-expected))<1e-4
                    checks.append(dict(edge_x_m=float(edge),expected_m=float(expected),samples=len(found),measured_m=float(np.median(found))))
                synthetic_checks.append(dict(scene='steps',central_risers=checks))
            elif meta['scene']=='flat-corridor-platforms':
                checks=[]
                for x,expected in [(-1.8,.15),(1.8,.8)]:
                    found=[r['delta_m'] for r in meta['records'] if abs(r['xy'][0]-x)<.9 and abs(r['xy'][1]+1.6)<.8]
                    assert found and np.max(np.abs(np.array(found)-expected))<1e-4
                    checks.append(dict(expected_m=expected,samples=len(found),measured_m=float(np.median(found))))
                synthetic_checks.append(dict(scene='flat-corridor-platforms',platforms=checks))
    manifest=Path('/tmp/go2-before-all-small-gaps.json');preservation=None
    if manifest.exists():
        snapshot=json.loads(manifest.read_text());changed=[p for p,digest in snapshot.items() if sha(Path(p))!=digest]
        assert not changed,changed;preservation=dict(files_verified=len(snapshot),changed_files=changed)
    (REPORT/'audit.json').write_text(json.dumps(dict(passed=True,records=audits,synthetic_geometry_checks=synthetic_checks,original_data_preservation=preservation),ensure_ascii=False,indent=2)+'\n')
    lines=['# 邻层曲面：边界高度差图','','日期：2026-11-22。端口仍为 http://127.0.0.1:42482/ 。','',
        '[三维：高低台与地面](http://127.0.0.1:42482/?scene=flat-corridor-platforms&profile=qcircle-adaptive&coverage=1&boundary=1) · '
        '[三维：多级台阶](http://127.0.0.1:42482/?scene=steps&profile=qcircle-adaptive&coverage=1&boundary=1)','',
        '## 看什么','',
        '蓝色保留各自二次函数的拟合曲面。白到红色线只表示两侧函数在同一个边界 XY 的高度差，0–80 cm 固定色标，超过 80 cm 同为红色但保留原数值。上方六项指标读数保持原定义，边界平均／最大值单独显示在图下。勾选“边界高度差”，可旋转、俯视、侧视；点击色线显示两侧函数编号和高差。竖向短线是两层高度的标尺，不是新拟合面。色线为诊断叠加，允许透过蓝色面显示。','',
        '默认合成高低台中，算法从拟合域识别出地面、矮台、高台、通道顶板下表面和上表面等区域，没有预设层数。矮台／地面的边界差约 15 cm，高台／地面约 80 cm，通道下／上表面边缘到地面约 60／72 cm。这些层间差不能全部解释成可行走台阶或机器人抬腿阈值。','',
        '俯视蓝色与当前页面一致，含已经启用的小缝显示补全；剖面蓝线只画原凸包内的解析函数。边界计算始终使用原始拟合域，没有把补缝当成新增观测。','',
        '![高低台边界与解析剖面](flat-corridor-platforms.png)','',
        '![多级台阶边界与解析剖面](steps.png)','',
        '## 如何计算','',
        '1. 仅使用保存的有效二次函数、原拟合点 XY 凸包；没有使用场景部件标签、人工台阶边线或已知高度来构造图，也没有重新拟合。',
        '2. 以函数为节点：凸包 XY 间距≤2.5 cm，且在两中心 XY 与中点三个共同查询位置的函数预测差都≤2 cm，才连接为同一局部一致区域。按连通分量得到区域，少于三个函数的分量不画边界；不预设三层或固定高度分层。',
        '3. 对每个区域的凸包并集提取外边界及 ≥9 cm² 的孔洞边界，每隔最多 2 cm 采样。沿外法向取 2 cm 探针，只选择向外侧延伸的其他区域，XY 边界间距≤10 cm；搜索 32 个最近拟合中心。同样近（5 mm 内）的候选中选择高度最接近的一层，避免一次把全部上下层都连起来。',
        '4. 原拟合域可能有间隙，通常没有严格共同边线。边界查询 q 取源边界和相邻拟合域最近点的 XY 中点，是由观测拟合域估计的边界；并非使用已知物体真实边界。源凸包上无法找到支持函数的点不画。',
        '5. 在完全相同的 q 代入两个原函数：Δz(q)=|f_i(q)−f_j(q)|。它能区分连续坡面与真正预测跳变。颜色按 Δz，图中仅显示≥2 cm 的差值；未显示不代表已证明无跳变，缺乏支持时不把未知记成零。',
        '6. 色线沿两函数的预测高度分别画出，并稀疏增加竖向标尺。不生成组合曲面、不跨层补面。局部函数在间隙处是有限外推，单侧中点外推距离最多约 5 cm；高度一致的连通分量也不保证整个区域由单一公式表示。','',
        '白到红指标色面和边界高差色线是两种量；勾选蓝色拟合覆盖后观察边界最清楚。边界均值是所保留诊断样本的算术平均，既不是面积加权均值，也不是全场景平均台阶高度。','',
        '## 逐设置结果','',
        '| 场景 | 设置 | 邻层区域对 | 边界样本 | 平均 Δz | 最大 Δz |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for r in records:
        mean='无支持' if r['mean_delta_m'] is None else f"{100*r['mean_delta_m']:.2f} cm"
        maximum='无支持' if r['maximum_delta_m'] is None else f"{100*r['maximum_delta_m']:.2f} cm"
        lines.append(f"| {r['name']} | {r['label']} | {r['region_pairs']} | {r['boundary_samples']:,} | {mean} | {maximum} |")
    lines+=['','默认多级台阶的最大 100 cm 来自最高台面侧边与外侧地面的落差，不是中央相邻台阶的最大一级高度。方窗在边缘混点会使原曲面倾斜、碎裂或偏移，因此边界高差也受原拟合误差影响；例如高台最大值约 78.7 cm，不能自动修正成已知的 80 cm。','',
        '## 验证与复现','',
        '逐样本独立重算同 XY 高差、检查函数有效性和边界 XY 距离；导出线顶点符合两侧解析函数，颜色值符合高差；全部原文件哈希不变。连续陡坡测试无跳变，叠加 15 cm 台阶后仍恢复 15 cm，两个平行面恢复 80 cm。默认拟合的实景构造数据另按几何位置核对两平台已知高度及中央十级台阶；这些已知值仅用于事后核查，不参与区域或边界计算，也不是独立观测精度。','',
        '[机器核查](audit.json) · [浏览器核查](browser-audit.json)；逐样本 CSV 位于 `<profile>/<scene>/samples.csv`，含两侧函数、区域、边界 XY、两侧 Z 和 Δz。','',
        '运行：系统 Python 依次执行 `python -m fsgp_bgk_repro.boundary_heights --profiles qcircle-adaptive qcircle-r07 q15-n15 q20-n30`、`python -m fsgp_bgk_repro.boundary_height_report`、`python -m fsgp_bgk_repro.publish_metric_viewer`。默认只计算本次请求的高低台通道场景及多级台阶，不改变其余场景的六指标和小缝补偿。','']
    (REPORT/'report.md').write_text('\n'.join(lines));print('Verified boundary height maps:',len(audits),flush=True)


if __name__=='__main__':run()
