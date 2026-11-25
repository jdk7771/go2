"""Apply the same bounded small-gap display completion to every current scene."""
import csv
import json
from pathlib import Path

from .assessment_export import finite_json,sha
from .convex_surface_assessment import OUT,SCENES,PROFILES,EXPERIMENTAL_PROFILES,PROFILE_SCENES
from .hill_small_gaps import build,audit_saved
from .scene_fit_catalog import NAMES

REPORT=OUT/'small-gaps'


def run(scenes=None):
    REPORT.mkdir(parents=True,exist_ok=True)
    selected=set(SCENES if scenes is None else scenes)
    saved=REPORT/'summary.json'
    previous={(r['profile'],r['scene']):r for r in json.loads(saved.read_text())['records']} if saved.exists() else {}
    records=[]
    for profile,label in (*PROFILES,*EXPERIMENTAL_PROFILES):
        for scene in PROFILE_SCENES[profile]:
            old=previous.get((profile,scene))
            records.append(build(profile,label,scene,REPORT,check_boundary=True) if scene in selected or old is None else old)
    manifest=Path('/tmp/go2-before-layer-choice.json')
    if not manifest.exists():manifest=Path('/tmp/go2-before-low-table.json')
    if not manifest.exists():manifest=Path('/tmp/go2-before-table-planning.json')
    audit=audit_saved(records,REPORT,manifest if manifest.exists() else Path('/tmp/go2-before-all-small-gaps.json'))
    summary=dict(date='2026-11-24',scenes=list(SCENES),settings=len(records),threshold_cm2=9.,
        raw_points_unchanged=True,coefficients_unchanged=True,metrics_unchanged=True,
        new_fits=0,large_gaps_unchanged=True,records=records,
        sources_sha256={name:sha(Path(__file__).parent/name) for name in
            ('small_gaps.py','hill_small_gaps.py','polygon_topology.py')})
    (REPORT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    columns=['scene','profile','raw_points','valid_functions','small_gaps_before','filled_gaps',
        'unfilled_small_gaps','filled_area_m2','large_gaps_retained','original_xy_union_area_m2',
        'display_xy_union_area_m2','overlay_triangles','new_fits','elapsed_ms']
    with (REPORT/'summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(records)
    lines=['# 全场景：小于 9 cm² 的封闭小缝显示补全','','日期：2026-11-24。统一入口 http://127.0.0.1:42482/ 。','',
        f'全部 {len(SCENES)} 个当前场景、四种基础设置和森林取消中心偏移上限的对照均同步，共 {len(records)} 组。',
        '规则相同：按有效凸包 XY 并集的整个封闭连通空档面积判断，严格小于 9 cm² 才考虑补齐；较大空档及外侧敞口保留，不继续拟合。小于 1e−12 m² 的数值碎片忽略。','',
        '不增加点云、中心或拟合函数。显示三角形 Z 来自一个邻近原函数，沿用其六项指标颜色；指标统计与原凸包域不变，补缝不是新增实测覆盖。','',
        '所有场景另统一检查边界高度：在原凸包支持的空档顶点，父函数预测与至少一个相邻支持面相差不超过 4 cm；保留不同高度表面的候选，避免因另一高度的面存在而否决同层延续。找不到满足条件的父函数时整块小缝保留，不能只补一半。搜索 12 个最近有效中心，顶点不得超出父函数原邻域半径外 4 cm。它是局部高度兼容检查，不是严格无缝约束。','',
        '## 默认自适应圆形结果','',
        '| 场景 | 补全／候选小缝 | 补全面积 | 原 XY 并集 | 补后显示并集 | 保留大空档 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in records:
        if r['profile']!='qcircle-adaptive':continue
        lines.append(f"| {NAMES[r['scene']]} | {r['filled_gaps']:,} / {r['small_gaps_before']:,} | {r['filled_area_m2']:.6f} m² | {r['original_xy_union_area_m2']:.6f} m² | {r['display_xy_union_area_m2']:.6f} m² | {r['large_gaps_retained']:,} |")
    lines+=['','面积是全高度凸包的 XY 投影并集，重叠只计一次；不是逐高度层覆盖率或曲面三维面积。多层场景中，投影已被其他高度曲面覆盖的位置不属于该投影空洞，不额外补面。没有候选小缝的设置也显示开关和零补全统计。','',
        f"## 验证\n\n{len(records)} 组均核对原始文件 SHA256、完整小缝面积门限、大空档面积逐项不变、新增并集面积与三角形面积、单函数归属及解析 Z。新增拟合求解均为 0。"
        +(f" 本轮全部 {audit['baseline_preservation']['files_verified']} 个基础与森林对照文件哈希不变。" if audit['baseline_preservation'] else ''),'',
        '[全部设置 CSV](summary.csv) · [完整 JSON](summary.json) · [几何与数据核查](audit.json) · [浏览器核查](browser-audit.json)','',
        '在 42482 首页切换场景和拟合设置，使用“补齐小缝（<9 cm²）”开关。蓝色覆盖模式区分显示面与空档；指标白红模式中的白色仍表示低指标值。','',
        '复现：系统 Python 依次运行 `python -m fsgp_bgk_repro.small_gaps` 与 `python -m fsgp_bgk_repro.publish_metric_viewer`。沿用系统 GEOS C 库与现有四设置数据，无需重新拟合。丘陵历史独立数据仍保留在 ../hill-small-gaps/。','']
    (REPORT/'report.md').write_text('\n'.join(lines))
    return summary


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scenes',nargs='+',choices=SCENES);args=parser.parse_args()
    run(args.scenes)
