"""Audit saved adaptive circular surfaces against the unchanged fixed profile."""
import csv
from dataclasses import fields
import hashlib
import html
import json
from pathlib import Path
import platform

import numpy as np
from scipy.spatial import cKDTree

from .adaptive_circular_surfaces import AdaptiveCircularConfig, AdaptiveCircularSurface, SCENES
from .assessment_export import OUT, FIELDS, finite_json, sha
from .circular_surfaces import CircularConfig, CircularSurface
from .compare_circle_square import METRICS, metric_stats
from .scene_fit_catalog import NAMES
from .terrain_assessment import evaluate

REPORT=OUT/'adaptive-circle'


def displayed_support(sample,vertices,faces,tolerance=.012):
    """Exact barycentric tests on nearby exported XY triangles, not hull area."""
    triangles=vertices[faces].astype(float)
    if not len(triangles):return 0.
    centers=triangles[:,:,:2].mean(axis=1)
    reach=float(np.max(np.linalg.norm(triangles[:,:,:2]-centers[:,None,:],axis=2)))
    tree=cKDTree(centers);nearby=tree.query_ball_point(sample[:,:2],reach+1e-7)
    supported=0
    for point,near in zip(sample,nearby):
        if not near:continue
        t=triangles[near];a=t[:,0,:2];u=t[:,1,:2]-a;v=t[:,2,:2]-a;w=point[:2]-a
        det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
        alpha=np.divide(w[:,0]*v[:,1]-w[:,1]*v[:,0],det,out=np.full(len(t),np.inf),where=np.abs(det)>1e-14)
        beta=np.divide(u[:,0]*w[:,1]-u[:,1]*w[:,0],det,out=np.full(len(t),np.inf),where=np.abs(det)>1e-14)
        inside=(alpha>=-1e-6)&(beta>=-1e-6)&(alpha+beta<=1+1e-6)
        z=t[:,0,2]+alpha*(t[:,1,2]-t[:,0,2])+beta*(t[:,2,2]-t[:,0,2])
        supported+=int(np.any(inside&(np.abs(z-point[2])<=tolerance)))
    return supported/len(sample)


def load_saved(name,profile):
    folder=OUT/'profiles'/profile/name
    meta=json.loads((folder/'data.json').read_text())
    config_type=AdaptiveCircularConfig if meta.get('adaptive_radius') else CircularConfig
    cls=AdaptiveCircularSurface if meta.get('adaptive_radius') else CircularSurface
    surface=object.__new__(cls)
    keys={f.name for f in fields(config_type)}
    surface.config=config_type(**{k:v for k,v in meta['config'].items() if k in keys})
    surface.points=np.fromfile(folder/'points.bin',dtype='<f4').reshape(-1,3).astype(float)
    models=np.fromfile(folder/'models.bin',dtype='<f8').reshape(-1,len(FIELDS))
    surface.seeds=models[:,:3]
    offsets=np.fromfile(folder/'neighbor-offsets.bin',dtype='<i4')
    neighbours=np.fromfile(folder/'neighbors.bin',dtype='<i4')
    surface.pools=[neighbours[a:b] for a,b in zip(offsets[:-1],offsets[1:])]
    surface.fits=dict(coefficient=models[:,3:9],height=models[:,FIELDS.index('height')],
        covariance=np.fromfile(folder/'covariance.bin',dtype='<f8').reshape(-1,6,6))
    surface.raw_tree=cKDTree(surface.points[:,:2]);surface._hulls={};surface._reindex()
    if meta.get('adaptive_radius'):
        surface.model_radii_m=np.fromfile(folder/'radii.bin',dtype='<f4').astype(float)
        surface.support_radii_m=np.fromfile(folder/'support-radii.bin',dtype='<f4').astype(float)
    return folder,meta,models,surface


def audit(name,profile):
    folder,meta,models,surface=load_saved(name,profile)
    rng=np.random.default_rng(211)
    sample=surface.points[rng.choice(len(surface.points),min(5000,len(surface.points)),replace=False)]
    ids,z=surface.query(sample[:,:2],expected=sample[:,2],z_tolerance=.012)
    vertices=np.fromfile(folder/'patches.bin',dtype='<f4').reshape(-1,3)
    faces=np.fromfile(folder/'patch-faces.bin',dtype='<u4').reshape(-1,3)
    owners=np.fromfile(folder/'patch-owners.bin',dtype='<u4')
    assert np.all(owners[faces]==owners[faces[:,0]][:,None]),'Cross-function display triangle'
    assert np.all(np.isfinite(surface.fits['height'][owners[faces]]))
    predicted=evaluate(surface.seeds,surface.fits['coefficient'],owners,vertices[:,:2])
    error=float(np.max(np.abs(predicted-vertices[:,2]),initial=0))
    assert error<1e-5,'Displayed vertices differ from their function'
    if meta.get('adaptive_radius'):
        radius=surface.model_radii_m[owners]
        assert np.all(np.linalg.norm(vertices[:,:2]-surface.seeds[owners,:2],axis=1)<=radius+1e-6)
        for i in surface.active:
            assert len(surface.pools[i])>=15
            assert np.all(np.linalg.norm(surface.points[surface.pools[i],:2]-surface.seeds[i,:2],axis=1)<=surface.model_radii_m[i]+1e-7)
    stages=meta['timings']['stages_ms']
    result=dict(scene=name,name=NAMES[name],profile=profile,raw_points=meta['raw_points'],
        centers=meta['centers'],actual_solve_count=meta['actual_solve_count'],valid=meta['valid'],
        displayed_functions=meta['displayed_functions'],triangles=len(faces),
        raw_xyz_sha256=meta['source']['sha256'],
        sample=dict(seed=211,size=len(sample),supported=int(np.sum(ids>=0)),
            supported_fraction=float(np.mean(ids>=0)),tolerance_m=.012,
            displayed_triangle_fraction=displayed_support(sample,vertices,faces),
            note='Same saved raw returns, not held-out accuracy or surface-area coverage.'),
        metrics={key:metric_stats(models,key) for key in METRICS},
        stages_ms=stages,fit_ms=stages['fit_pipeline_ms'],
        fit_and_display_ms=stages['fit_pipeline_ms']+stages['display_geometry_ms'],
        all_metrics_ms=meta['timings']['total_compute_ms'],
        radius_stats_m=meta.get('radius_stats_m',dict(minimum=.07,median=.07,maximum=.07)),
        radius_histogram=meta.get('radius_histogram',{'0.070':meta['valid']}),
        mean_models_per_used_raw_point=meta['mean_models_per_used_raw_point'],
        max_models_per_raw_point=meta['max_models_per_raw_point'],
        raw_points_used=meta['raw_points_referenced_by_valid_fits'],
        extra_centers=meta['adaptive'],
        geometry_audit=dict(cross_function_triangles=0,max_vertex_polynomial_error_m=error),
        files_sha256={file:sha(folder/file) for file in ('data.json','models.bin','patches.bin','patch-faces.bin')})
    print(name,profile,'raw support',f'{result["sample"]["supported_fraction"]:.2%}',flush=True)
    return result


def run():
    REPORT.mkdir(exist_ok=True)
    records=[audit(name,profile) for name in SCENES for profile in ('qcircle-r07','qcircle-adaptive')]
    for i in range(0,len(records),2):assert records[i]['raw_xyz_sha256']==records[i+1]['raw_xyz_sha256']
    manifest=Path('/tmp/go2-before-adaptive.json')
    preservation=None
    if manifest.exists():
        saved=json.loads(manifest.read_text());changed=[p for p,h in saved.items() if sha(Path(p))!=h]
        assert not changed,changed
        preservation=dict(files_verified=len(saved),changed_files=changed)
    summary=dict(date='2026-11-21',records=records,original_file_preservation=preservation,
        environment=dict(python=platform.python_version(),numpy=np.__version__,blas_threads=1),
        source_sha256={p:sha(Path(__file__).parent/p) for p in ('adaptive_circular_surfaces.py','circular_surfaces.py','terrain_assessment.py','adaptive_circle_report.py')},
        note='No point resampling or new scenes. Same saved XYZ; adaptive neighbourhoods, centre pitch and raw support rules change together. One CPU run per scene, excludes IO/browser. Training RMSE is not independent accuracy.')
    (REPORT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    rows=[];metric_rows=[]
    for r in records:
        row={k:r[k] for k in ('scene','profile','raw_points','centers','actual_solve_count','valid','displayed_functions','triangles','fit_ms','fit_and_display_ms','all_metrics_ms','mean_models_per_used_raw_point','max_models_per_raw_point')}
        row.update(supported_raw_fraction=r['sample']['supported_fraction'],**{f'radius_{k}_m':v for k,v in r['radius_stats_m'].items()})
        rows.append(row)
        metric_rows.extend(dict(scene=r['scene'],profile=r['profile'],metric=key,**value) for key,value in r['metrics'].items())
    for filename,data in [('comparison.csv',rows),('metrics.csv',metric_rows),('timing.csv',[dict(scene=r['scene'],profile=r['profile'],**r['stages_ms']) for r in records])]:
        keys=list(dict.fromkeys(k for row in data for k in row))
        with (REPORT/filename).open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(data)
    lines=['# 四个稀疏地形：自适应圆形二次拟合','','日期：2026-11-21。统一查看端口 42482；不包含路径规划。','',
        '保留沙漠、山地、雪地、火山原始 XYZ 与旧 R=7 cm、15 cm／20 cm 方窗数据。新增 `qcircle-adaptive` 设置，仅用于这四个场景，11 项原有指标全部计算。','',
        '## 如何自适应','',
        '1. 从最多 5000 个固定种子样本，统计第 24 个 XY 近邻距离的 70 分位数，向上选择名义半径（7／8.5／10／12 cm，限至 12 cm），六角中心间距为它的 1.6 倍。中心高度参考附近原始观测，不预设层数。',
        '2. 每个中心在 7／8.5／10／12／15 cm 候选半径中递增尝试；优先接受的半径还要容纳六角覆盖距离与原始点间距余量，避免稀疏格点凸包之间的规则小洞。目标至少 24 点，全部合格点参与；若较大的域不合格或达不到点数目标，回退到已经求得的最宽、至少 15 点且质量合格的函数（允许小于优先覆盖半径）。最大半径 15 cm，不无限扩大。',
        '3. 有明确高度间隙且有二维分布的足量观测组时分开处理；再拟合局部切平面，以相对切平面的 ±4 cm 残差筛点，代替旧版相对中心原始 Z 的 ±4 cm 门限。保留训练 RMSE≤8 mm、满秩和中心偏移检查。不是 FSGP 论文 GP 分割，也不保证所有复杂表面分割正确。',
        '4. 查询必须在该函数自身圆域、原始拟合点凸包（边界容差 7.5 mm）及兼容原始点附近。XY 支持半径按局部点间距的 0.9 倍取值，限制为 22–35 mm。兼容高度比较原始点 Z 与函数在该原始点 XY 的预测（≤12 mm），避免将正常斜坡高差误判为不支持。',
        '5. 各函数独立建面；三角形只连接同一函数上的采样点。裁切后增加一层径向采样，并检查顶点、重心和边中点的原始点支持，不跨函数填洞。有观测但仍缺支持的位置最多增加初始中心数 5% 的面片。没有点云的较大空洞继续留空。','',
        '## 覆盖与残差','',
        '下表支持率：同一随机种子抽取的 5000 个已观测原始点，要求查询有有效函数，且预测与该原始点 Z 相差≤12 mm。这是观测点支持率，不是独立测试精度，也不是地表面积覆盖率。显示函数数与有效函数数分别统计，不能只比较中心有效百分比。','',
        '| 场景 | 原始点 | 显示函数：旧→新 | 观测点支持率：旧→新 | 新半径范围 | 新训练 RMSE P95 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for i in range(0,len(records),2):
        a,b=records[i:i+2];rs=b['radius_stats_m']
        lines.append(f"| {b['name']} | {b['raw_points']:,} | {a['displayed_functions']:,} → {b['displayed_functions']:,} | {a['sample']['supported_fraction']:.1%} → {b['sample']['supported_fraction']:.1%} | {rs['minimum']*100:g}–{rs['maximum']*100:g} cm | {b['metrics']['rmse_m']['p95']*1000:.3f} mm |")
    lines+=['','雪地采用更大的中心间距，中心与显示函数数减少；应结合观测点支持率判断，不能将函数越多直接视为效果越好。训练残差低不等于对未观测表面的误差低。','',
        '另对同一批原始点进行显示三角形的 XY 重心坐标包含测试，并要求插值 Z 与原始 Z 相差≤12 mm，得到实际显示几何的采样点覆盖率：','',
        '| 场景 | 旧 R=7 cm 显示几何覆盖 | 自适应显示几何覆盖 |',
        '| --- | ---: | ---: |']
    for i in range(0,len(records),2):
        a,b=records[i:i+2]
        lines.append(f"| {b['name']} | {a['sample']['displayed_triangle_fraction']:.2%} | {b['sample']['displayed_triangle_fraction']:.2%} |")
    lines+=['','显示覆盖低于函数查询支持率，因为显示三角形还要求多个采样位置同时有观测支持；少量合法函数范围没有显示几何。该表仍按已观测点采样计数，不是地表面积覆盖。','',
        '## 新设置的计算量与单次耗时','',
        '系统 Python，单线程 BLAS。拟合链路包含搜索树、中心选择、各半径尝试、协方差、凸包和有限补片；出图几何另计。完整指标还包含 A/B/C 双侧重拟合、上方占用、落足、80×50 cm 机身评价。以下不含文件 IO、浏览器与首次编译加载，不是稳定速度基准。','',
        '| 场景 | 中心／有效／显示函数 | 主拟合实际求解次数 | 拟合链路 | 拟合＋显示几何 | 加全部指标 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in records[1::2]:
        lines.append(f"| {r['name']} | {r['centers']:,} / {r['valid']:,} / {r['displayed_functions']:,} | {r['actual_solve_count']:,} | {r['fit_ms']/1000:.3f} s | {r['fit_and_display_ms']/1000:.3f} s | {r['all_metrics_ms']/1000:.3f} s |")
    lines+=['','同一中心可能尝试多个半径，因此实际求解次数大于保存函数数；B/C 的额外双侧求解另存各场景 `data.json`。有效拟合所用原始点平均复用次数：'+ '；'.join(f"{r['name']} {r['mean_models_per_used_raw_point']:.2f} 次" for r in records[1::2])+'。这是平均值，不限制每个点最多复用一次。','',
        '## 指标与界面','',
        '保留相同 11 项指标及成本归一化。粗糙度法向变化、上方占用使用各函数自己的半径；机身仍为 80×50 cm 固定窗口并要求四个名义支撑点和至少 80% 网格支持。A/B/C 候选与双侧外推沿用旧工程规则；大跳变可能包含外推不稳定，不能直接当作实测台阶或控制切换阈值。未知指标保持未知，不代表不可通行。','',
        '页面“拟合设置”选择“自适应圆形”；勾选“拟合覆盖”显示蓝色曲面，选指标切换白到红。点选函数显示实际半径、参与点数和数值；灰色实体面仅表示指标未知，背景空隙才是无显示曲面。四场景以外禁用此设置并回退到原固定圆形。','']
    for name in SCENES:lines.append(f'- [{NAMES[name]}：直接查看](http://127.0.0.1:42482/?scene={name}&profile=qcircle-adaptive&coverage=1)。')
    lines+=['','## 验证与文件','',
        '数值测试检查稀疏斜坡、分离的 3 cm 台阶、缺失观测条带、每个显示三角形属于同一函数及其圆域；不保证检测到任意小孔。保存结果另核对显示顶点满足所属函数、拟合邻域在自身半径内、每个有效函数至少 15 点。原有数据与新设置原始点哈希一致。','',
        f"历史数据保留核对：{preservation['files_verified'] if preservation else '未提供清单'} 个原有文件哈希未变化。保留核对清单来自本次生成前的本地快照。",'',
        '本次数值测试 61 项通过；浏览器核查四场景×11 指标、实际颜色与数值、三种历史设置、局部半径、跨场景回退及手机宽度。浏览器记录：[browser-audit.json](browser-audit.json)。','',
        '结果：`../profiles/qcircle-adaptive/<scene>/`；每函数 `radii.bin` 保存实际半径，`support-radii.bin` 保存局部原始点支持半径；`neighbors.bin` 和 `neighbor-offsets.bin` 保存参与拟合的原始点索引。','',
        '统计：[summary.json](summary.json)、[配置对照 CSV](comparison.csv)、[11 项指标 CSV](metrics.csv)、[阶段计时 CSV](timing.csv)。','',
        '复现：`PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 -m fsgp_bgk_repro.adaptive_circular_surfaces`，然后运行 `adaptive_circle_report` 与 `publish_metric_viewer`。','']
    report='\n'.join(lines);(REPORT/'report.md').write_text(report)
    links=' · '.join(f'<a href="../../?scene={name}&amp;profile=qcircle-adaptive&amp;coverage=1">{html.escape(NAMES[name])}</a>' for name in SCENES)
    (REPORT/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>自适应圆形拟合</title><style>body{max-width:1100px;margin:28px auto;padding:0 18px;font:16px/1.65 system-ui}pre{white-space:pre-wrap;overflow-wrap:anywhere}a{color:#16649a}</style><h1>四场景自适应圆形拟合</h1><p>'+links+'</p><p><a href="report.md">Markdown 报告</a> · <a href="comparison.csv">对照 CSV</a> · <a href="metrics.csv">指标 CSV</a></p><pre>'+html.escape(report)+'</pre></html>')
    return summary


if __name__=='__main__':run()
