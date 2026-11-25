"""Verify complete convex display domains and publish the six-metric readout."""
import csv
import json
from pathlib import Path
import platform
import numpy as np
from .convex_surface_assessment import OUT, SCENES, PROFILES, FIELDS, METRICS, derivative_metrics
from .assessment_export import OUT as HISTORY, sha, finite_json, stats
from .scene_fit_catalog import NAMES


def audit(profile,name):
    folder=OUT/profile/name;meta=json.loads((folder/'data.json').read_text())
    m=np.fromfile(folder/'models.bin','<f8').reshape(-1,len(FIELDS));columns={k:m[:,i] for i,k in enumerate(FIELDS)}
    active=np.flatnonzero(columns['valid']>0)
    v=np.fromfile(folder/'patches.bin','<f4').reshape(-1,3).astype(float);f=np.fromfile(folder/'patch-faces.bin','<u4').reshape(-1,3)
    owner=np.fromfile(folder/'patch-owners.bin','<u4');polygons=np.fromfile(folder/'polygons.bin','<f8').reshape(-1,2);offsets=np.fromfile(folder/'polygon-offsets.bin','<i4')
    assert np.all(owner[f]==owner[f[:,0]][:,None])
    a=v[f[:,1],:2]-v[f[:,0],:2];b=v[f[:,2],:2]-v[f[:,0],:2]
    areas=np.abs(a[:,0]*b[:,1]-a[:,1]*b[:,0])/2
    rendered=np.bincount(owner[f[:,0]],weights=areas,minlength=len(m))
    np.testing.assert_allclose(rendered[active],columns['domain_area_m2'][active],rtol=3e-4,atol=1e-7)
    assert meta['valid']==meta['displayed_functions']==len(np.unique(owner[f]))
    assert meta['geometry']['observation_face_deletion'] is False and meta['geometry']['cross_function_assembly'] is False
    for field in METRICS:assert np.all(np.isfinite(columns[field][active])),field
    assert np.all((columns['clearance_cost'][active]>=0)&(columns['clearance_cost'][active]<=1))
    assert np.all((columns['slope_deg'][active]>=0)&(columns['slope_deg'][active]<=90))
    covariance=np.fromfile(folder/'covariance.bin','<f8').reshape(-1,6,6)
    rng=np.random.default_rng(441)
    for i in rng.choice(active,min(32,len(active)),replace=False):
        q=polygons[offsets[i]:offsets[i+1]];result=derivative_metrics(m[i,3:9],q,covariance[i])
        for key in ('std_patch_m','slope_deg','normal_spread_deg'):np.testing.assert_allclose(result[key],columns[key][i],rtol=1e-12,atol=1e-12)
    max_vertex_error=0.
    for start in range(0,len(v),100000):
        raw=v[start:start+100000];ids=owner[start:start+100000];u=raw[:,0]-m[ids,0];w=raw[:,1]-m[ids,1];c=m[ids,3:9]
        predicted=m[ids,2]+c[:,0]+c[:,1]*u+c[:,2]*w+c[:,3]*u*u+c[:,4]*u*w+c[:,5]*w*w
        max_vertex_error=max(max_vertex_error,float(np.max(np.abs(predicted-raw[:,2]),initial=0)))
    assert max_vertex_error<5e-5,max_vertex_error
    preserved_fit=None
    if meta['fit']['source']=='unchanged_saved_fit':
        old=HISTORY/name if profile=='q20-n30' else HISTORY/'profiles'/profile/name
        old_meta=json.loads((old/'data.json').read_text());old_model=np.fromfile(old/'models.bin','<f8').reshape(-1,len(old_meta['fields']))
        for key in ('x','y','z','c0','c1','c2','c3','c4','c5','height','valid','point_count','rmse_m','max_error_m'):
            np.testing.assert_array_equal(columns[key],old_model[:,old_meta['fields'].index(key)])
        preserved_fit=True
    else:
        # Independently check training residuals against the actual raw pools,
        # also for new scenes that have no historical fitted models to reuse.
        points=np.fromfile(folder/'points.bin','<f4').reshape(-1,3).astype(float)
        neighbor_ids=np.fromfile(folder/'neighbors.bin','<i4')
        neighbor_offsets=np.fromfile(folder/'neighbor-offsets.bin','<i4')
        for i in rng.choice(active,min(32,len(active)),replace=False):
            raw=points[neighbor_ids[neighbor_offsets[i]:neighbor_offsets[i+1]]]
            u,w=(raw[:,:2]-m[i,:2]).T;c=m[i,3:9]
            predicted=m[i,2]+c[0]+c[1]*u+c[2]*w+c[3]*u*u+c[4]*u*w+c[5]*w*w
            error=predicted-raw[:,2]
            np.testing.assert_allclose([np.sqrt(np.mean(error**2)),np.max(np.abs(error))],
                [columns['rmse_m'][i],columns['max_error_m'][i]],rtol=1e-6,atol=1e-12)
    assert meta['field_stats']=={key:finite_json(stats(columns[key][active])) for key in METRICS}
    result=dict(scene=name,name=NAMES[name],profile=profile,raw_points=meta['raw_points'],centers=meta['centers'],solved=meta['solved'],
        actual_solve_count=meta['actual_solve_count'],valid=meta['valid'],displayed=meta['displayed_functions'],triangles=len(f),
        domain_xy_area_sum_m2=float(np.sum(columns['domain_area_m2'][active])),metrics=meta['field_stats'],timings=meta['timings'],fit=meta['fit'],
        audit=dict(whole_hulls_covered=True,cross_function_triangles=0,max_float32_vertex_error_m=max_vertex_error,fixed_fit_arrays_unchanged=preserved_fit))
    print('Verified',profile,name,'valid/displayed',len(active),flush=True)
    return result


def run():
    records=[audit(profile,name) for profile,_ in PROFILES for name in SCENES]
    manifest=OUT.parent/'surface-planner/projection-input-sha256.json';preservation=None
    if not manifest.exists():manifest=Path('/tmp/go2-before-layer-choice.json')
    if not manifest.exists():manifest=Path('/tmp/go2-before-low-table.json')
    if not manifest.exists():manifest=Path('/tmp/go2-before-convex.json')
    if manifest.exists():
        original=json.loads(manifest.read_text());changed=[p for p,h in original.items() if sha(Path(p))!=h]
        assert not changed,changed;preservation=dict(files_verified=len(original),changed_files=changed)
    summary=dict(date='2026-11-24',scenes=list(SCENES),profiles=[dict(id=p,label=l) for p,l in PROFILES],metrics=METRICS,records=records,
        original_data_preservation=preservation,environment=dict(python=platform.python_version(),numpy=np.__version__,blas_threads=1),
        sources_sha256={name:sha(Path(__file__).parent/name) for name in
            ('convex_surface_assessment.py','convex_assessment_report.py','adaptive_circular_surfaces.py',
             'circular_surfaces.py','terrain_assessment.py','scene_fit_catalog.py','flat_obstacles_scene.py')})
    (OUT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    rows=[dict(scene=r['scene'],profile=r['profile'],raw_points=r['raw_points'],centers=r['centers'],actual_main_solves=r['actual_solve_count'],
        valid=r['valid'],displayed=r['displayed'],triangles=r['triangles'],**r['timings']) for r in records]
    metric_rows=[dict(scene=r['scene'],profile=r['profile'],metric=k,**v) for r in records for k,v in r['metrics'].items()]
    for filename,data in [('timing.csv',rows),('metrics.csv',metric_rows)]:
        keys=list(dict.fromkeys(k for row in data for k in row))
        with (OUT/filename).open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(data)
    lines=['# 当前地形评价：凸多边形裁剪与六项指标','','日期：2026-11-24。统一入口 http://127.0.0.1:42482/ 。','',
        '## 场景与设置','',
        f"首页提供{'、'.join(NAMES[n] for n in SCENES)}，共 {len(SCENES)} 场景，评价方法为局部二次曲面。默认自适应圆形，固定 R=7 cm、15 cm 方窗和20 cm 方窗作为拟合设置保留；各组使用相同保存 XYZ，并统一裁剪和指标定义。",'',
        '新增“平地 · 通道与高低台”：8×8 m 平地，通道长 4 m、净宽 2 m、净高 0.60 m；矮台高 0.15 m，高台高 0.80 m，两台各 1.6×1.4 m。通道内部保留地面，两端敞开；平台为实心，底部内部地面不采样。点间距约 2.5 cm，完整无噪声构造表面，不是模拟 LiDAR 扫描。部件标签仅用于几何核查，拟合和指标只输入 XYZ。','',
        '当前流程不生成跨函数的组合网格，不计算 FSGP、突变 A/B/C、落足或机身综合代价。首页没有这些选项。旧实验的独立页面和原始文件保留在原地址；旧首页查看器保存在 /metrics-history/。','',
        '[拟合曲面上的分层 A*](http://127.0.0.1:42482/surface-planner/)另页展示四足代价、软 IK、准静态 ZMP、滚动及跨层连接候选；两个第二台下方通行版本也按同一四设置与六指标处理。','',
        '## 拟合与凸包显示','',
        '各设置保留自己的选点和质量门限。原有固定设置复用中心、系数、邻域与协方差；新场景的四种设置均从原始 XYZ 重新选点、拟合。自适应使用同一密度、候选半径和质量规则，半径 7–15 cm，至少 15 点、目标 24 点、训练 RMSE≤8 mm，新增中心最多 5%。方窗最低点数分别 15／30，RMSE≤20 mm 门限保留。','',
        '每个有效函数的显示域直接取其合格拟合点 XY 的精确凸包。不再扩大凸包 7.5 mm，不增加显示高度带或近原始点门限，也不因检查位置缺点而删除整三角形。圆形／方形是选点邻域，显示外沿统一为凸多边形。','',
        '显示顶点的 XYZ 由本函数计算；网页所需网格覆盖整个凸包，各三角形只属于一个函数。细分仅用于近似显示弯曲函数，不影响任何指标。相邻函数保留独立系数，不保证交界严格无缝；所有场景统一规则。凸包会包含内部观测空洞，蓝色表示模型显示范围，不是自由空间或实测连续覆盖证明。','',
        '## 六项指标：统一公式','',
        '设局部函数 z=z0+c0+c1u+c2v+c3u²+c4uv+c5v²。内部评价位置取凸包内 1 cm XY 网格中心，等权近似 XY 面积平均；极小域无格点时用凸包顶点平均位置。这个采样独立于显示网格。','',
        '1. 坡度：fx=c1+2c3u+c4v，fy=c2+c4u+2c5v；n=(-fx,-fy,1)/sqrt(1+fx²+fy²)。将各采样位置的单位法向取平均，再归一化为 nbar；坡度=acos(nbar.z)。正负对称坡面可能平均抵消，需同时看平滑性。',
        '2. 平滑性：sqrt(mean(acos(n·nbar)²))，以度表示，越小越平滑。它测法向的空间变化，没有加入 RMSE。它不是跨函数接缝的连续性指标。',
        '3. 拟合 RMSE：sqrt(mean((f(xk,yk)-zk)²))，仅对本次实际参与拟合的原始点。',
        '4. 拟合最大误差：max(abs(f(xk,yk)-zk))，同一批训练点。两者不是独立测试精度。',
        '5. 不确定性：sigma(q)=sqrt(phi(q)^T Sigma phi(q))。在独立内部采样加凸包顶点上取最大值，是采样的预测均值标准差；不是已标定误差，也不是连续域严格上界。',
        '6. 上方占用：只使用原始点。原始点 XY 必须在本函数凸包内，且相对 f 的高度在 10–60 cm；其所在 5 cm 全局 XY 列记为占用。每列与凸包的交叠面积精确裁剪求得，占用比=占用列交叠面积之和/凸包面积。没有复用显示顶点或三角形，零值仅表示没有这类障碍观测。','',
        '所有场景与拟合设置使用同一原始单位和固定白到红色标：占用 0–100%、坡度 0–45°、RMSE 0–20 mm、sigma 0–10 mm、法向变化 0–15°、最大残差 0–20 mm；超过红端仍保存和统计原值。','',
        '## 默认自适应设置的结果','',
        '| 场景 | 原始点 | 中心 | 主拟合求解次数 | 有效／显示函数 | 当前完整计算 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in records[:len(SCENES)]:
        lines.append(f"| {r['name']} | {r['raw_points']:,} | {r['centers']:,} | {r['actual_solve_count']:,} | {r['valid']:,} / {r['displayed']:,} | {r['timings']['total_compute_ms']/1000:.2f} s |")
    lines+=['','有效／显示按函数计数，不是地表面积覆盖率。独立凸包可能重叠；显示网格会近似函数，不会将多个二次函数融合。','',
        '## 验证与计时口径','',
        f'{len(SCENES)} 场景×{len(PROFILES)} 设置共 {len(records)} 组；每个有效函数均完整显示。检查显示三角形 XY 面积和等于对应凸包面积、每个三角形属于单一函数、显示顶点符合函数（允许 float32 量化）、全部有效函数拥有六项有限指标、独立重新计算的平均法向和 sigma 与保存值一致。复用固定设置的原始拟合字段逐项核对未改变；新求解设置抽样复算训练残差。','',
        f"原有结果文件哈希核对：{preservation['files_verified'] if preservation else '未提供快照'} 个文件未改变。新增数据单独存放，不覆盖历史结果。",'',
        '旧场景的数值、颜色、完整凸包、点选边界、CSV、手机布局和旧链接浏览器记录见 browser-audit.json；两版下穿场景的 8 组设置、48 个指标视图见 small-gaps/platform-underpass-browser-audit.json；本轮五平台通行链的 4 组设置、24 个指标视图见 small-gaps/four-platforms-browser-audit.json。Python 测试记录见项目 result.md。','',
        '单次 CPU／单线程 BLAS 计时。新场景四设置均包含重新选点、拟合和协方差；原有固定设置使用保存拟合，因此跨场景时间不能直接当作同工作量速度比较。显示几何、六项指标另列；不含结果导出和浏览器渲染。上方占用闭区间端点允许 float32 数值量化误差（约 0.12 微米），避免 0.60 m 存为 0.6000000238 m 后漏检。','',
        '[summary.json](summary.json) · [六项指标 CSV](metrics.csv) · [阶段计时 CSV](timing.csv) · [浏览器核查](browser-audit.json)','',
        '复现：依次运行 `python -m fsgp_bgk_repro.convex_surface_assessment`、`python -m fsgp_bgk_repro.convex_assessment_report`、`python -m fsgp_bgk_repro.publish_metric_viewer`。本机使用 /usr/bin/python3，PYTHONNOUSERSITE=1，OPENBLAS_NUM_THREADS=1，OMP_NUM_THREADS=1，MKL_NUM_THREADS=1。','']
    if (OUT/'forest-center-offset/report.md').exists():
        lines+=['## 森林中心偏移单项对照（2026-11-22）','',
            '森林另提供“自适应圆形 · 无中心偏移限制”，只取消 |c0|≤4 cm 的有效性检查；选点高度筛选、至少 15 点、秩检查、RMSE≤8 mm 和六项指标定义保留。其他场景保持原四设置。该对照独立保存，不包含在上述基础统计中。','',
            '[原设置与取消上限的实测对照](forest-center-offset/report.md) · [直接查看](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive-no-offset&coverage=1)','']
    if (OUT/'small-gaps/report.md').exists():
        lines+=['## 全场景小缝显示补全（2026-11-22）','',
            f'全部 {len(SCENES)} 场景、四基础设置及森林门限对照共 {4*len(SCENES)+1} 组同步“补齐小缝（<9 cm²）”开关。只考虑整个连通面积严格小于 9 cm² 的封闭 XY 投影空档；大空档与外侧敞口保留，不继续拟合。统一检查原凸包支持顶点的函数预测高度差≤4 cm；不兼容的小缝整块保留。显示高度解析取自原函数，沿用其六指标颜色；不增加观测或有效函数，不改变上述指标与统计。','',
            '[全部场景面积与验证报告](small-gaps/report.md) · [直接查看](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive&coverage=1&gaps=1) · [原丘陵独立实验](hill-small-gaps/report.md)','']
    elif (OUT/'hill-small-gaps/report.md').exists():
        lines+=['## 丘陵小缝显示补全（2026-11-22）','',
            '丘陵提供“补齐小缝（<9 cm²）”显示开关，四种基础设置各自计算。只补原凸包 XY 并集内、整个连通区域面积严格小于 9 cm² 的封闭小缝；较大空档与外侧敞口保留，不继续拟合。补缝高度由邻近原二次函数解析求得，沿用其指标颜色；不增加原始点或有效函数，也不改变上述六指标与基础统计。','',
            '[面积与验证报告](hill-small-gaps/report.md) · [直接查看](http://127.0.0.1:42482/?scene=hill&profile=qcircle-adaptive&coverage=1&gaps=1)','']
    if (OUT/'boundary-heights/report.md').exists():
        lines+=['## 邻层边界高度差诊断（2026-11-22）','',
            '高低台通道与多级台阶的四设置另有边界高差色线，按同一边界 XY 计算 |f_i(q)-f_j(q)|；局部一致区域自动识别，不预设层数。默认矮台／地面约 15 cm、高台／地面约 80 cm。色线与其统计独立于原六项指标，不生成组合曲面或新增拟合。','',
            '[方法、俯视与解析剖面](boundary-heights/report.md) · [三维高低台图](http://127.0.0.1:42482/?scene=flat-corridor-platforms&profile=qcircle-adaptive&coverage=1&boundary=1)','']
    (OUT/'report.md').write_text('\n'.join(lines))
    return summary


if __name__=='__main__':run()
