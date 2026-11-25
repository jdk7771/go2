"""Forest-only center-offset ablation, including an unchanged-fit comparison.

The main case reruns the adaptive pipeline. The frozen comparison changes only
the acceptance of already saved coefficients, isolating the final quality gate.
Support is evaluated on observed training-cloud points, not held-out accuracy.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

from .adaptive_circular_surfaces import AdaptiveCircularSurface
from .assessment_export import sha,finite_json,stats
from .convex_surface_assessment import OUT,FIELDS,METRICS,OFFSET_PROFILE,run as export,raw_points_path,inside
from .convex_assessment_report import audit
from .terrain_assessment import basis

REPORT=OUT/'forest-center-offset'
BASELINE='qcircle-adaptive'


def load(profile):
    folder=OUT/profile/'forest';meta=json.loads((folder/'data.json').read_text())
    model=np.fromfile(folder/'models.bin','<f8').reshape(-1,len(FIELDS))
    return dict(meta=meta,model=model,columns={key:model[:,i] for i,key in enumerate(FIELDS)},
        polygons=np.fromfile(folder/'polygons.bin','<f8').reshape(-1,2),
        offsets=np.fromfile(folder/'polygon-offsets.bin','<i4'),
        neighbors=np.fromfile(folder/'neighbors.bin','<i4'),
        neighbor_offsets=np.fromfile(folder/'neighbor-offsets.bin','<i4'))


def observed_support(case,sample):
    m=case['model'];active=np.flatnonzero(case['columns']['valid']>0)
    tree=cKDTree(m[active,:2]);near=tree.query_ball_point(sample[:,:2],.15+1e-9)
    distance=np.full(len(sample),np.nan)
    for k,(p,ids) in enumerate(zip(sample,near)):
        errors=[]
        for i in active[ids]:
            polygon=case['polygons'][case['offsets'][i]:case['offsets'][i+1]]
            uv=p[:2]-m[i,:2]
            if not inside(uv[None,:],polygon)[0]:continue
            predicted=m[i,2]+basis(uv)@m[i,3:9]
            errors.append(abs(predicted-p[2]))
        if errors:distance[k]=min(errors)
    return dict(sampled_observed_points=len(sample),seed=211,threshold_m=.012,
        inside_any_xy_hull_fraction=float(np.isfinite(distance).mean()),
        height_supported_fraction=float(np.mean(distance<=.012)),
        note='Same observed points in both cases; at least one exact convex domain must contain XY and predict within 12 mm. This is not held-out precision or surface-area coverage.')


def frozen_gate_comparison(case,points):
    c=case['columns'];m=case['model'];accepted=c['valid']>0
    # These are ordered center-offset rejections, including cases that ALSO
    # fail RMSE. Removing the center check cannot rescue the latter cases.
    blocked=np.flatnonzero((~accepted)&(c['point_count']>=15)&(np.abs(c['c0'])>.04))
    eligible=[]
    for i in blocked:
        if not np.isfinite(c['rmse_m'][i]) or c['rmse_m'][i]>.008:continue
        ids=case['neighbors'][case['neighbor_offsets'][i]:case['neighbor_offsets'][i+1]]
        xy=points[ids,:2]-m[i,:2];scale=max(np.sqrt(np.mean(np.sum(xy**2,axis=1))),1e-8)
        a=basis(xy/scale);eig=np.linalg.eigvalsh(a.T@a)
        if eig[0]>max(eig[-1],1e-30)*1e-10:eligible.append(int(i))
    return dict(fixed_saved_centers=len(m),baseline_valid=int(accepted.sum()),
        ordered_center_offset_rejections=len(blocked),rescued_with_same_coefficients=len(eligible),
        valid_without_center_gate=int(accepted.sum())+len(eligible),rescued_model_ids=eligible,
        note='No refitting, radius reselection or new centers. Count/rank/RMSE checks remain active.')


def summarize(case,sample):
    c=case['columns'];active=np.flatnonzero(c['valid']>0);offset=np.abs(c['c0'][active])
    maximum=int(active[np.argmax(offset)]);polygon=case['polygons'][case['offsets'][maximum]:case['offsets'][maximum+1]]
    return dict(profile=case['meta']['profile_id'],raw_points=case['meta']['raw_points'],centers=len(case['model']),
        actual_solves=case['meta']['actual_solve_count'],valid=len(active),displayed=case['meta']['displayed_functions'],
        valid_center_offset_over_4cm=int((offset>.04).sum()),center_offset_m=stats(offset),
        metrics={key:stats(c[key][active]) for key in METRICS},observed_support=observed_support(case,sample),
        fit_pipeline_ms=case['meta']['fit']['fit_pipeline_ms'],total_compute_ms=case['meta']['timings']['total_compute_ms'],
        largest_center_offset=dict(model=maximum,offset_m=float(abs(c['c0'][maximum])),
            center_inside_convex_domain=bool(inside(np.zeros((1,2)),polygon)[0]),
            center_reference_xyz=case['model'][maximum,:3].tolist(),fitted_center_height_m=float(c['height'][maximum]),
            rmse_m=float(c['rmse_m'][maximum]),domain_area_m2=float(c['domain_area_m2'][maximum])))


def run():
    REPORT.mkdir(parents=True,exist_ok=True)
    export(('forest',),(OFFSET_PROFILE,))
    geometry_audit=audit(OFFSET_PROFILE,'forest')
    before,after=load(BASELINE),load(OFFSET_PROFILE)
    points=np.fromfile(raw_points_path('forest'),'<f4').reshape(-1,3).astype(float)
    # Verify default-on numerical behavior against all saved baseline fits.
    control=AdaptiveCircularSurface(points)
    np.testing.assert_array_equal(control.seeds,before['model'][:,:3])
    for key,column in [('coefficient',before['model'][:,3:9]),('height',before['columns']['height']),
            ('rmse',before['columns']['rmse_m']),('max_error',before['columns']['max_error_m'])]:
        np.testing.assert_array_equal(control.fits[key],column)
    offsets=before['neighbor_offsets'];ids=before['neighbors']
    for i,pool in enumerate(control.pools):np.testing.assert_array_equal(pool,ids[offsets[i]:offsets[i+1]])
    sample=points[np.random.default_rng(211).choice(len(points),5000,replace=False)]
    cases=[summarize(case,sample) for case in (before,after)]
    preserved=None;manifest=Path('/tmp/go2-before-offset-ablation.json')
    if manifest.exists():
        hashes=json.loads(manifest.read_text());changed=[p for p,h in hashes.items() if sha(Path(p))!=h]
        assert not changed,changed;preserved=dict(files=len(hashes),changed=changed)
    report=dict(date='2026-11-22',scene='forest',raw_xyz_sha256=sha(raw_points_path('forest')),
        experiment=dict(center_offset_check=False,height_selection_gate_m=.04,rmse_limit_m=.008,minimum_points=15,
            radius_candidates_m=[.07,.085,.10,.12,.15],metrics=METRICS),
        control_baseline_reproduced_exactly=True,fresh_control_fit_pipeline_ms=control.times['fit_pipeline_ms'],
        frozen_gate_comparison=frozen_gate_comparison(before,points),cases=cases,
        geometry_audit=geometry_audit['audit'],baseline_preservation=preserved,
        source_sha256={name:sha(Path(__file__).parent/name) for name in
            ('terrain_assessment.py','adaptive_circular_surfaces.py','convex_surface_assessment.py','forest_center_offset_ablation.py')})
    (REPORT/'summary.json').write_text(json.dumps(finite_json(report),ensure_ascii=False,indent=2)+'\n')
    b,a=cases;f=report['frozen_gate_comparison']
    lines=['# 森林：取消中心预测偏移上限','','日期：2026-11-22。统一入口 http://127.0.0.1:42482/ 。','',
        '取消的是拟合通过判据 |c0|≤4 cm；邻域高度模式与切平面残差 ±4 cm 筛选保留，至少 15 点、秩检查、RMSE≤8 mm 及六项指标定义不变。原四设置与结果保留。','',
        '## 从相同原始 XYZ 重新运行自适应拟合','',
        '| 统计 | 原中心偏移≤4 cm | 取消中心偏移上限 |','| --- | ---: | ---: |',
        f"| 原始点 | {b['raw_points']:,} | {a['raw_points']:,} |",
        f"| 保存中心 | {b['centers']:,} | {a['centers']:,} |",
        f"| 主拟合实际求解次数 | {b['actual_solves']:,} | {a['actual_solves']:,} |",
        f"| 有效／显示函数 | {b['valid']:,} / {b['displayed']:,} | {a['valid']:,} / {a['displayed']:,} |",
        f"| 有效函数中偏移>4 cm | {b['valid_center_offset_over_4cm']} | {a['valid_center_offset_over_4cm']} |",
        f"| 最大中心偏移 | {b['center_offset_m']['maximum']*100:.2f} cm | {a['center_offset_m']['maximum']*100:.2f} cm |",
        f"| 已观测点支持率（同一批 5000 点） | {b['observed_support']['height_supported_fraction']*100:.2f}% | {a['observed_support']['height_supported_fraction']*100:.2f}% |",
        f"| 训练 RMSE P95 | {b['metrics']['rmse_m']['p95']*1000:.3f} mm | {a['metrics']['rmse_m']['p95']*1000:.3f} mm |",
        f"| 训练 RMSE 最大值 | {b['metrics']['rmse_m']['maximum']*1000:.3f} mm | {a['metrics']['rmse_m']['maximum']*1000:.3f} mm |",
        f"| 预测 σ P95 | {b['metrics']['std_patch_m']['p95']*1000:.3f} mm | {a['metrics']['std_patch_m']['p95']*1000:.3f} mm |",
        f"| 当前重新运行的拟合链路 | {control.times['fit_pipeline_ms']/1000:.3f} s | {a['fit_pipeline_ms']/1000:.3f} s |",'',
        '主表中原始中心、邻域、系数和残差在本次默认开关开启的重新计算中逐项复现。取消检查后，会改变接受的半径以及补中心位置，因此最终中心数量也会变化；不是直接给原来 261 个中心全部判通过。','',
        '已观测点支持定义：原始点 XY 落在某个有效函数的精确凸包内，且该函数的预测 Z 与该点差≤12 mm。这些点参与过拟合，是同帧观测支持诊断，不是独立测试精度或地表面积覆盖率。','',
        '## 固定原有中心、半径、邻域和系数的单项对照','',
        f"原有 {f['fixed_saved_centers']:,} 个中心中，按检查顺序归为中心偏移失败的 {f['ordered_center_offset_rejections']} 个，只有 {f['rescued_with_same_coefficients']} 个同时通过点数、秩与 RMSE 检查。仅取消最后这个中心检查、不重新选半径或补中心时，有效函数从 {f['baseline_valid']:,} 增加至 {f['valid_without_center_gate']:,}。",'',
        '## 如何理解','',
        f"完整重跑增加 {a['valid']-b['valid']} 个有效函数（{(a['valid']/b['valid']-1)*100:.2f}%），对空白的改善需结合上述观测支持率判断。稀疏、混合表面或残差超限仍会留空。",'',
        '中心偏移较大可能来自参考高度与拟合面不一致，或者中心处的外推；不能直接当作该面真实重建误差。这里仍只显示拟合点凸包，中心是否位于凸包内及最大偏移示例已记录在 summary.json。训练 RMSE 小不能保证中心参考高度合理。','',
        '## 查看与复现','',
        '[原设置](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive&coverage=1) · '
        '[取消中心偏移上限](http://127.0.0.1:42482/?scene=forest&profile=qcircle-adaptive-no-offset&coverage=1) · '
        '[六指标及逐函数 CSV](../qcircle-adaptive-no-offset/forest/per-model.csv) · [机器可读结果](summary.json)','',
        '在森林的“拟合设置”选择“自适应圆形 · 无中心偏移限制”。点选曲面可见中心预测偏移。其他场景维持原四设置。','',
        '运行 `PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 -m fsgp_bgk_repro.forest_center_offset_ablation`，再运行 `python -m fsgp_bgk_repro.publish_metric_viewer`。CPU 单次测量，不含导出与网页渲染。','']
    (REPORT/'report.md').write_text('\n'.join(lines))
    print(json.dumps(finite_json(report),ensure_ascii=False,indent=2),flush=True)
    return report


if __name__=='__main__':run()
