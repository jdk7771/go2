"""Run paired Q20/N30 and Q15/N15 experiments on the identical saved XYZ.

The original profile is checked against its published arrays and never rewritten.
The new profile includes all assessment fields and actual local patch geometry.
"""
import argparse
from dataclasses import asdict, replace
import csv
import gc
import hashlib
import html
import json
from pathlib import Path
import platform
import time

import numpy as np

from .assessment_export import OUT, ORDER, NAMES, FIELDS, compute, finite_json, publish, save_scene
from .terrain_assessment import AssessmentConfig

PROFILE_ID = 'q15-n15'
PROFILE_LABEL = '15×15 cm · 至少 15 点'
CONFIG = replace(AssessmentConfig(), patch_size_m=.15, minimum_points=15)
PROFILE_OUT = OUT / 'profiles' / PROFILE_ID
REPORT_OUT = OUT / 'profile-comparison'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(models, faces, owners, triangles, cfg):
    col = {key: i for i, key in enumerate(FIELDS)}
    valid = models[:, col['valid']].astype(bool)
    solved = np.isfinite(models[:, col['rmse_m']])
    shown = np.unique(owners[faces])
    used = np.unique(triangles)
    rejected = solved & ~valid
    row = dict(centers=len(models), solved=int(solved.sum()), valid=int(valid.sum()),
               valid_fraction=float(valid.mean()), insufficient=int((models[:, col['point_count']] < cfg.minimum_points).sum()),
               rejected=int(rejected.sum()), displayed_functions=len(shown),
               valid_no_patch=int(valid.sum()-len(shown)), patch_triangles=np.asarray(faces).size//3,
               assembled_functions=len(used), assembled_triangles=np.asarray(triangles).size//3,
               valid_unassembled=int(valid.sum()-len(used)))
    assert row['insufficient'] + row['rejected'] + row['valid'] == len(models)
    for field in ('point_count', 'rmse_m', 'max_error_m', 'std_patch_m', 'jump_b_m', 'jump_sigma_m', 'jump_upper_m', 'slope_deg'):
        values = models[valid, col[field]]
        values = values[np.isfinite(values)]
        row[field] = dict(median=float(np.median(values)), p95=float(np.percentile(values,95)), maximum=float(values.max())) if len(values) else dict(median=None,p95=None,maximum=None)
    for field in ('jump_b_m', 'foot_cost', 'body_cost'):
        row[field+'_known'] = int((valid & np.isfinite(models[:, col[field]])).sum())
    return row


def export_report(records):
    REPORT_OUT.mkdir(parents=True, exist_ok=True)
    report = dict(experiment_date='2026-11-18', timezone='Asia/Shanghai',
        profiles={'q20-n30':asdict(AssessmentConfig()), PROFILE_ID:asdict(CONFIG)}, scenes=records,
        code_sha256={name:sha(Path(__file__).parent/name) for name in ('terrain_assessment.py','assessment_export.py','assessment_profiles.py')},
        environment=dict(python=platform.python_version(),numpy=np.__version__,blas_threads=1),
        notes=[
            'Same saved raw XYZ bytes, same 10 cm voxel-selected centres, same centre order; no resampling or point upsampling.',
            'Only requested config changes: patch_size_m 0.20->0.15, minimum_points 30->15. ALL eligible points are used, not exactly 15 nearest neighbours.',
            'Height gate +/-4 cm, rank/ridge/RMSE checks, jump candidate offset 24 cm, assembly thresholds and 80x50 cm body dimensions are fixed.',
            'Patch geometry and normal-corner evaluation follow the selected width. Clearance uses 5 cm columns: 4x4 for Q20, 3x3 for Q15, normalized by the actual number of columns.',
            'Body support queries respect the selected local half-width; body coverage still describes the same 80x50 cm rectangle. XY support boxes are not an exact continuous collision or convex-hull certificate.',
            'Paired timings: one sequential whole-scene CPU run per profile, baseline first. Total includes fitting, assembly, patch geometry and all assessments, excludes file IO and browser. No claim of stable speedup from a single run.',
            'Residuals are training residuals, not held-out accuracy. Centre counts and triangle counts are not covered area; smaller patches and changed sets of valid centres affect comparisons.',
            'Original Q20 binaries/metadata are preserved; recomputed default models and geometry are checked against saved results. Historical FSGP benchmark remains an independent experiment with its original code hashes.',
        ])
    (REPORT_OUT/'summary.json').write_text(json.dumps(finite_json(report),ensure_ascii=False,indent=2)+'\n')
    csv_fields=['scene','profile','raw_points','centers','solved','valid','valid_fraction','insufficient','rejected',
                'displayed_functions','valid_no_patch','patch_triangles','assembled_functions','assembled_triangles',
                'valid_unassembled','rmse_p95_mm','max_residual_p95_mm','patch_sigma_p95_mm',
                'jump_b_p95_cm','jump_b_max_cm','body_cost_known','fit_pipeline_ms','total_assessment_ms','baseline_models_exact_match']
    with (REPORT_OUT/'summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=csv_fields);writer.writeheader()
        for record in records:
            for profile in ('q20-n30',PROFILE_ID):
                item=record[profile]
                row=dict(scene=record['id'],profile=profile,raw_points=record['raw_points'],**item,
                         baseline_models_exact_match=record['baseline_models_exact_match'])
                for dest,src in [('rmse_p95_mm','rmse_m'),('max_residual_p95_mm','max_error_m'),('patch_sigma_p95_mm','std_patch_m')]:
                    x=item[src]['p95'];row[dest]=None if x is None else x*1000
                for dest,key in [('jump_b_p95_cm','p95'),('jump_b_max_cm','maximum')]:
                    x=item['jump_b_m'][key];row[dest]=None if x is None else x*100
                writer.writerow({key:row.get(key) for key in csv_fields})
    lines=['# 15×15 cm／至少 15 点，与 20×20 cm／至少 30 点对照','',
        f'日期：2026-11-18。{len(records)} 场景使用各自同一份已保存原始点云；中心体素仍为 10 cm，高度门限仍为 ±4 cm。窗口内全部合格点参与拟合。', '',
        '表中比例按拟合中心统计，不是覆盖面积。残差为训练残差，不是独立测试精度。CPU 计时每组每场景一次，完整评估包括组合、面片几何和各项评价，排除文件读写与网页渲染。','',
        '| 场景 | 中心数 | 有效函数 20→15 | 有效比例 20→15 | 实际独立面函数 20→15 | 未组合有效函数 20→15 | 完整评估 s 20→15 |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    rows=[]
    for r in records:
        a,b=r['q20-n30'],r[PROFILE_ID]
        cells=[r['name'],f"{a['centers']:,}",f"{a['valid']:,} → {b['valid']:,}",
               f"{a['valid_fraction']*100:.1f}% → {b['valid_fraction']*100:.1f}%",
               f"{a['displayed_functions']:,} → {b['displayed_functions']:,}",
               f"{a['valid_unassembled']:,} → {b['valid_unassembled']:,}",
               f"{a['total_assessment_ms']/1000:.3f} → {b['total_assessment_ms']/1000:.3f}"]
        lines.append('| '+' | '.join(cells)+' |')
        link=f'/?scene={r["id"]}&profile={PROFILE_ID}&metric=jump_b_m&coverage=1'
        rows.append('<tr>'+''.join(f'<td>{html.escape(x)}</td>' for x in cells)+f'<td><a href="{html.escape(link)}">查看</a></td></tr>')
    findings=[]
    forest=next((r for r in records if r['id']=='forest'),None)
    if forest:
        a,b=forest['q20-n30'],forest[PROFILE_ID];paired=forest['paired_centers']
        findings=[f"森林有效函数 {a['valid']:,}→{b['valid']:,}，实际独立面函数 {a['displayed_functions']:,}→{b['displayed_functions']:,}；新增有效中心 {paired['newly_valid']:,}，原先有效但新设置失效 {paired['lost_valid']:,}。",
                  f"森林面片 σ 中位数 {a['std_patch_m']['median']*1000:.2f}→{b['std_patch_m']['median']*1000:.2f} mm；B 最大外推差 {a['jump_b_m']['maximum']:.2f}→{b['jump_b_m']['maximum']:.2f} m。B 是函数在候选位置的外推差，不能把此极值当作真实测得的台阶高度；覆盖增加没有证明外推更稳定或精度更高。"]
    lines += ['', *findings, '', '仅缩小窗口、降低最小点数不保证每处都更好：点数、秩、残差和显示裁剪可能分别限制结果。机身尺寸仍为 80×50 cm；净空占用评价范围随局部窗口从 20 cm 变为 15 cm，不能把两种占用值当作相同物理足印。', '',
              '源码 `terrain_assessment.py` 已将净空列数、法向角点和机身查询局部范围参数化；默认 20 cm 结果逐场景与原始模型及网格核对。原 FSGP 对比报告和计时快照保留原始协议，不更新成此次 15 cm 结果。', '',
              '[统一页面](http://127.0.0.1:42482/) · [CSV](summary.csv) · [完整统计 JSON](summary.json)', '']
    (REPORT_OUT/'report.md').write_text('\n'.join(lines))
    section=['## 13. 新增：15×15 cm／至少 15 点设置（2026-11-18）','',
             '统一首页增加“拟合设置”：原 20×20 cm／至少 30 点，与新 15×15 cm／至少 15 点。两组使用相同原始 XYZ 和相同中心；窗口内全部合格点参与。高度门限仍为 ±4 cm，机器人窗口仍为 80×50 cm。', '',
             *findings, '',
             '完整 17 场景表、阶段耗时和逐组指标见 [设置对照](http://127.0.0.1:42482/assessment/profile-comparison/)；[直接打开森林 15 cm 版本](http://127.0.0.1:42482/?scene=forest&profile=q15-n15&metric=jump_b_m&coverage=1&mesh=0)。有效比例按中心计算，不是面积；残差为训练残差；本次计时每组各一次。默认 20 cm 模型与网格已复算核对，并保留原文件。', '',
             '对应实现：`assessment_profiles.py`；汇总 `assessment/profile-comparison/summary.json`；新二进制结果 `assessment/profiles/q15-n15/<scene>/`。净空列数、法向角点和机身支持查询跟随所选局部尺寸，默认参数下结果保持一致。浏览器核查 2 设置×17 场景×11 指标。','']
    (REPORT_OUT/'step-section.md').write_text('\n'.join(section).rstrip()+'\n')
    headings=['场景','中心数','有效函数 20→15','有效比例 20→15','独立面函数 20→15','未组合有效函数 20→15','完整评估 s 20→15','可视化']
    (REPORT_OUT/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>拟合设置对照</title><style>body{font:14px/1.7 system-ui;color:#293744;margin:24px}a{color:#2675a6}table{border-collapse:collapse;white-space:nowrap}td,th{padding:9px 12px;border-bottom:1px solid #dce2e7;text-align:left}.scroll{overflow:auto}p{max-width:1100px}</style><h1>15 cm／15 点 与 20 cm／30 点</h1><p>同一原始点云、同一中心；至少 15／30 点，窗口内全部合格点参与。下表按中心和函数计数，不是覆盖面积。完整 CPU 计时每组一次，不能据此断定稳定加速。训练残差不是独立测试精度。</p><p><a href="/">返回统一页面</a> · <a href="summary.csv">下载 CSV</a> · <a href="summary.json">详细统计</a> · <a href="report.md">说明</a></p><div class="scroll"><table><thead><tr>'+''.join('<th>'+h+'</th>' for h in headings)+'</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>'+''.join('<p>'+html.escape(s)+'</p>' for s in findings)+'</html>')


def refresh_report():
    """Rebuild presentation statistics from saved arrays, preserving timings."""
    records=json.loads((REPORT_OUT/'summary.json').read_text())['scenes']
    for record in records:
        for profile,cfg,root in [('q20-n30',AssessmentConfig(),OUT),(PROFILE_ID,CONFIG,PROFILE_OUT)]:
            folder=root/record['id']
            models=np.fromfile(folder/'models.bin',dtype='<f8').reshape(record[profile]['centers'],-1)
            faces=np.fromfile(folder/'patch-faces.bin',dtype='<u4')
            owners=np.fromfile(folder/'patch-owners.bin',dtype='<u4')
            triangles=np.fromfile(folder/'mesh.bin',dtype='<u4')
            record[profile].update(summarize(models,faces,owners,triangles,cfg))
    export_report(records)


def run_scene(name):
    folder=OUT/name
    protected={p.name:sha(p) for p in folder.iterdir() if p.is_file()}
    meta=json.loads((folder/'data.json').read_text())
    frozen=np.fromfile(folder/'models.bin',dtype='<f8').reshape(meta['centers'],-1)
    points=np.fromfile(folder/'points.bin',dtype='<f4').reshape(-1,3)
    record=dict(id=name,name=NAMES[name],raw_points=len(points),source_xyz_sha256=sha(folder/'points.bin'))
    arrays={}
    for pid,cfg in [('q20-n30',AssessmentConfig()),(PROFILE_ID,CONFIG)]:
        gc.collect();start=time.perf_counter();result=compute(points,cfg);wall=(time.perf_counter()-start)*1000
        a=result['packed'];arrays[pid]=a.copy()
        if pid=='q20-n30':
            np.testing.assert_allclose(a,frozen,rtol=1e-10,atol=1e-12,equal_nan=True)
            record['baseline_models_exact_match']=bool(np.array_equal(a,frozen,equal_nan=True))
            for filename,key,dtype in [('mesh.bin','triangles','<u4'),('patches.bin','patches','<f4'),('patch-faces.bin','patch_faces','<u4'),('patch-owners.bin','patch_owners','<u4')]:
                np.testing.assert_array_equal(result[key].ravel(),np.fromfile(folder/filename,dtype=dtype))
        else:
            np.testing.assert_array_equal(a[:,:3],frozen[:,:3])
            new_meta=save_scene(name,points,meta['source'],result,cfg,output_root=PROFILE_OUT)
            new_meta.update(profile_id=PROFILE_ID,profile_label=PROFILE_LABEL,source_xyz_sha256=record['source_xyz_sha256'])
            (PROFILE_OUT/name/'data.json').write_text(json.dumps(new_meta,ensure_ascii=False,indent=2)+'\n')
            assert sha(PROFILE_OUT/name/'points.bin')==record['source_xyz_sha256']
        row=summarize(a,result['patch_faces'],result['patch_owners'],result['triangles'],cfg)
        row.update(stages_ms=result['times'],total_assessment_ms=result['times']['total_assessment_ms'],
                   fit_pipeline_ms=sum(result['times'][k] for k in ('select_centers_ms','window_query_ms','fit_and_covariance_ms')),
                   compute_call_wall_ms=wall)
        record[pid]=row
        print(json.dumps(dict(scene=name,profile=pid,valid=row['valid'],centers=len(a),total_ms=round(row['total_assessment_ms'],2))),flush=True)
        del result
    valid0=arrays['q20-n30'][:,FIELDS.index('valid')].astype(bool);valid1=arrays[PROFILE_ID][:,FIELDS.index('valid')].astype(bool)
    record['paired_centers']=dict(both_valid=int((valid0&valid1).sum()),newly_valid=int((~valid0&valid1).sum()),lost_valid=int((valid0&~valid1).sum()),both_invalid=int((~valid0&~valid1).sum()))
    for filename,expected in protected.items():
        assert sha(folder/filename)==expected,f'Original profile file was modified: {name}/{filename}'
    record['original_files_unchanged']=True
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes',nargs='+',choices=ORDER,default=ORDER)
    parser.add_argument('--report-only',action='store_true',help='Rebuild comparison text/statistics from saved arrays without rerunning fits')
    args=parser.parse_args();records=[]
    if args.report_only:
        refresh_report();return
    for name in args.scenes:
        records.append(run_scene(name));export_report(records)
    publish(PROFILE_OUT,copy_assets=False)
    if (OUT.parent/'convex-assessment/profiles.json').exists():
        from .publish_metric_viewer import publish as publish_viewer
        publish_viewer()
    print('Published profile',PROFILE_ID,'and paired report',REPORT_OUT,flush=True)


if __name__=='__main__':main()
