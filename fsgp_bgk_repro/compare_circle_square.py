"""Recompute square assessments and compare the same metrics with saved circles.

All profiles use identical saved XYZ. Existing square arrays are checked and
preserved; circular results are produced by circular_surfaces.export_profile.
"""
import argparse
import csv
from dataclasses import replace
import gc
import html
import json
import os
from pathlib import Path
import platform

import numpy as np

from .assessment_export import OUT, FIELDS, compute, finite_json, sha
from .scene_fit_catalog import ORDER, NAMES
from .terrain_assessment import AssessmentConfig


REPORT = OUT/'circle-square-comparison'
METRICS = ('jump_b_m','slope_deg','roughness_cost','max_error_m','rmse_m',
           'std_patch_m','clearance_cost','foot_cost','body_cost','jump_a_m','jump_upper_m')
PROFILES = (
    ('q20-n30','20×20 cm · 至少 30 点',OUT,AssessmentConfig()),
    ('q15-n15','15×15 cm · 至少 15 点',OUT/'profiles/q15-n15',
        replace(AssessmentConfig(),patch_size_m=.15,minimum_points=15)),
    ('qcircle-r07','圆形 R=7 cm · 至少 15 点',OUT/'profiles/qcircle-r07',None),
)


def metric_stats(models, field):
    columns = {name:i for i,name in enumerate(FIELDS)}
    valid = models[:,columns['valid']]>0
    values = models[valid,columns[field]]
    finite = values[np.isfinite(values)]
    row = dict(known=len(finite),valid_functions=int(valid.sum()),
               known_fraction=len(finite)/valid.sum() if valid.any() else None)
    for key,operation in (('mean',np.mean),('median',np.median),
                          ('minimum',np.min),('maximum',np.max)):
        row[key] = float(operation(finite)) if len(finite) else None
    row['p95'] = float(np.percentile(finite,95)) if len(finite) else None
    return row


def write_report(records):
    REPORT.mkdir(parents=True,exist_ok=True)
    root = Path(__file__).resolve().parent
    summary = dict(date='2026-11-21',metrics=METRICS,scene_count=len(records)//3,
        configurations=[dict(id=pid,label=label) for pid,label,_,_ in PROFILES],records=records,
        environment=dict(python=platform.python_version(),numpy=np.__version__,
            threads={key:os.environ.get(key) for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}),
        source_sha256={name:sha(root/name) for name in
            ('terrain_assessment.py','assessment_export.py','circular_surfaces.py','compare_circle_square.py')},
        notes=[
            'Same saved raw XYZ, no additional scenes or resampled observations.',
            'Square metrics recomputed from raw XYZ and checked against saved arrays; original square files preserved.',
            'Circle coefficient/centre/support geometry retained; all 11 metric fields now calculated.',
            'Common slope/roughness/jump/uncertainty scales and composite weights. RMSE validity cutoffs remain 20 mm for squares and 8 mm for circles.',
            'Circle normal variation uses 24 circle-boundary samples; squares use four corners. These nominal-domain gradients can extend beyond observed hulls.',
            'Circle clearance uses 5 cm columns clipped to the circular footprint, weighted by intersection area; square columns have equal area.',
            'Body rectangle remains 80x50 cm, four nominal contacts, 25 grid samples and 80% minimum grid coverage. Circles require circle/hull/raw-point support; historical square support rule preserved.',
            'A/B/C use the historical 24 cm candidate offset and two disjoint side refits evaluated at the same candidate midpoint; this can extrapolate beyond local support and is not a measured step-height guarantee.',
            'Unknown values remain NaN/empty/null. Known-centre fractions are not covered surface area.',
            'Different centre selection, neighbourhood sizes, density thresholds and support rules: this is a configuration comparison, not an isolated circle-versus-square shape experiment.',
            'Training residuals are not held-out accuracy; composite scores are geometric proxies, not success probabilities.',
            'One CPU computation per scene/profile, including display geometry and metric stages, excluding IO/browser; not a stable speedup benchmark.',
        ])
    (REPORT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    metric_rows=[]
    for record in records:
        for field,stats in record['metrics'].items():
            metric_rows.append(dict(scene=record['scene'],profile=record['profile'],metric=field,**stats))
    with (REPORT/'metrics.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(metric_rows[0]))
        writer.writeheader();writer.writerows(metric_rows)
    with (REPORT/'timing.csv').open('w') as stream:
        rows=[dict(scene=r['scene'],profile=r['profile'],**r['stages_ms']) for r in records]
        keys=list(dict.fromkeys(key for row in rows for key in row))
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    title='圆形与正方形：17 场景同类指标计算'
    lines=[f'# {title}','','日期：2026-11-21。只计算拟合与几何评价，统一可视化端口 42482。','',
        '## 计算范围','',
        '保留原有 17 个场景与三种拟合设置：20×20 cm／至少 30 点、15×15 cm／至少 15 点、半径 7 cm／至少 15 点。相同原始 XYZ，所有合格点参与拟合。两种方窗重新计算并核对已有数组，历史文件不覆盖；圆形补齐原来缺少的指标。','',
        '三组均提供 11 项：双侧突变 B、坡度、粗糙度、最大训练残差、训练 RMSE、预测标准差、上方占用、落足评价、机身评价、单侧突变 A、加不确定性的突变 C。','',
        '## 统一公式与范围差异','',
        '- 粗糙度 R=max(clip(RMSE/1 cm), clip(边界法向变化/15°))。圆形取 24 个圆周点，方窗取四角。',
        '- 不确定性项 U=clip((最大训练残差+2σpatch)/2 cm)；不与各设置的拟合有效性门限混用。',
        '- 落足 F=0.25P+0.20R+0.25J+0.20O+0.10U，P=clip(坡度/45°)，J=clip(C/12 cm)。缺少必要分量为未知；坡度达到 45° 按原规则置 1。',
        '- 机身窗口仍为 80×50 cm；四个名义足点与 25 个支持采样，网格覆盖至少 80%；权重、平面残差、投影裕量与旧方窗定义相同。圆形用圆域、凸包与兼容原始点检查支持，旧方窗代理规则保留。',
        '- 上方占用检查曲面上方 10–60 cm。方窗使用等面积 5 cm 列；圆形将 5 cm 列裁到圆域并按交叠面积加权，圆外点不计入占用。观测占用为零不等于证明自由空间。',
        '- A/B/C 保留原候选规则：±X/±Y 偏移 24 cm，B/C 两侧原始点不重叠，各自重拟合后在同一候选中点比较。中点可在原局部域外；这是外推不一致量，不保证等于真实台阶高度。',
        '- 圆形保持半径 7 cm、11.5 cm 中心间距、最多 5% 补片与 8 mm 有效性 RMSE；方窗原设置与 20 mm 门限保留。训练残差不是独立精度，当前比较不能单独证明圆形形状更优。','',
        '## 逐场景结果','',
        '已知数量的分母均为本组有效函数数，不能解释为曲面面积。总耗时包含拟合、显示几何及全部指标，方窗另包含原组合计算；单次计时不代表稳定加速。','',
        '| 场景 | 设置 | 中心／有效 | B 已知 | 落足已知 | 机身已知 | 总计算 ms |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    table=[]
    for r in records:
        values=[r['name'],r['label'],f"{r['centers']:,} / {r['valid_functions']:,}",
            str(r['metrics']['jump_b_m']['known']),str(r['metrics']['foot_cost']['known']),
            str(r['metrics']['body_cost']['known']),f"{r['total_compute_ms']:.1f}"]
        lines.append('| '+' | '.join(values)+' |')
        url=f"/?scene={r['scene']}&profile={r['profile']}&metric=jump_b_m&coverage=0"
        table.append('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+
                     f'<td><a href="{html.escape(url,quote=True)}">查看</a></td></tr>')
    lines+=['','完整逐指标均值、中位数、P95、最大值与已知率见 [metrics.csv](metrics.csv) 和 [summary.json](summary.json)；阶段计时见 [timing.csv](timing.csv)。','',
        '## 复现','', '```bash','cd /home/jiang/snap/go2',
        'export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1',
        '/usr/bin/python3 -m fsgp_bgk_repro.circular_surfaces',
        '/usr/bin/python3 -m fsgp_bgk_repro.compare_circle_square',
        '/usr/bin/python3 -m fsgp_bgk_repro.publish_metric_viewer','```','']
    (REPORT/'report.md').write_text('\n'.join(lines))
    (REPORT/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
        '<title>'+title+'</title><style>body{font:14px/1.8 system-ui;color:#293744;margin:24px}a{color:#2675a6}table{border-collapse:collapse;white-space:nowrap}td,th{padding:8px 12px;border-bottom:1px solid #dce2e7;text-align:left}.scroll{overflow:auto}p{max-width:1000px}</style>'
        '<h1>'+title+'</h1><p>三种拟合设置均计算 11 项指标。相同原始 XYZ；未知保留，未恢复路径规划或新增场景。中心选择、有效性门限和支撑范围不同，不能据此单独判断圆形形状优劣。</p>'
        '<p><a href="/">返回可视化</a> · <a href="report.md">计算说明</a> · <a href="metrics.csv">指标 CSV</a> · <a href="timing.csv">计时 CSV</a> · <a href="summary.json">详细 JSON</a></p>'
        '<div class="scroll"><table><thead><tr>'+''.join('<th>'+h+'</th>' for h in ('场景','设置','中心／有效','B 已知','落足已知','机身已知','总计算 ms','曲面'))+
        '</tr></thead><tbody>'+''.join(table)+'</tbody></table></div></html>')


def run(names=ORDER):
    records=[]
    for name in names:
        raw_sha=sha(OUT/name/'points.bin')
        points=np.fromfile(OUT/name/'points.bin',dtype='<f4').reshape(-1,3)
        for pid,label,base,config in PROFILES:
            gc.collect();folder=base/name;meta=json.loads((folder/'data.json').read_text())
            assert sha(folder/'points.bin')==raw_sha,(name,pid,'XYZ mismatch')
            models=np.fromfile(folder/'models.bin',dtype='<f8').reshape(meta['centers'],len(FIELDS))
            if config is not None:
                result=compute(points,config)
                np.testing.assert_allclose(result['packed'],models,rtol=1e-10,atol=1e-12,equal_nan=True)
                for filename,key,dtype in (('mesh.bin','triangles','<u4'),('patches.bin','patches','<f4'),
                    ('patch-faces.bin','patch_faces','<u4'),('patch-owners.bin','patch_owners','<u4')):
                    np.testing.assert_array_equal(result[key].ravel(),np.fromfile(folder/filename,dtype=dtype))
                stages=result['times'];main_solves=int(np.isfinite(result['fits']['rmse']).sum())
                side_solves=result['jumps']['solve_counts']['actual_side_solves']
                exact=bool(np.array_equal(result['packed'],models,equal_nan=True))
                models=result['packed'].copy();del result
            else:
                assert meta.get('metrics_version')=='shared-assessment-v1',(name,'circle metrics not generated')
                stages=meta['timings']['stages_ms'];main_solves=meta['actual_solve_count']
                side_solves=meta['jump_solve_counts']['actual_side_solves'];exact=None
            row=dict(scene=name,name=NAMES[name],profile=pid,label=label,raw_points=len(points),
                raw_xyz_sha256=raw_sha,centers=len(models),valid_functions=int((models[:,FIELDS.index('valid')]>0).sum()),
                actual_main_solves=main_solves,actual_side_solves=side_solves,square_recompute_exact_match=exact,
                metrics={field:metric_stats(models,field) for field in METRICS},stages_ms=stages,
                total_compute_ms=stages['total_assessment_ms'])
            records.append(row)
            print(json.dumps(dict(scene=name,profile=pid,valid=row['valid_functions'],
                jump_known=row['metrics']['jump_b_m']['known'],foot_known=row['metrics']['foot_cost']['known'],
                body_known=row['metrics']['body_cost']['known'],ms=round(row['total_compute_ms'],1))),flush=True)
        write_report(records)
    return records


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes',nargs='+',choices=ORDER,default=ORDER)
    run(parser.parse_args().scenes)
