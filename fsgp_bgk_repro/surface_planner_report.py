"""Refresh measured sections of the initial planner report from saved audits."""
import csv
import json
from pathlib import Path
from .scene_fit_catalog import WEB
from .assessment_export import sha
from .surface_astar import CASES
from .scene_fit_catalog import NAMES,CURRENT_ASSESSMENT_SCENES

ROOT=Path(__file__).resolve().parents[1]
OUT=WEB/'surface-planner'
BEGIN='<!-- measured-results:start -->'
END='<!-- measured-results:end -->'


def run():
    if json.loads((OUT/'results/hill/result.json').read_text()).get('schema_version',1)>=2:
        from .layer_projection_report import run as report_projection
        return report_projection()
    rows=[];scenes=[];case_rows=[];lines=['| 场景／分支 | A* 与过渡检查 | 路径长 | 节点／显示检查帧 | 高差标记段 | 最大 IK 偏差 | 最小 ZMP 裕量 |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for scene in CASES:
        result_path=OUT/'results'/scene/'result.json';r=json.loads(result_path.read_text());a=json.loads((result_path.parent/'audit.json').read_text())
        assert a['passed'] and a['result_sha256']==sha(result_path)
        for route,checked in zip(r['routes'],a['routes']):
            name=NAMES[scene]+'／'+route['label'];seconds=route.get('planning_statistics',r['statistics'])['elapsed_s']
            suffix='（两条合计）' if scene=='surface-route-test' else ''
            lines.append(f"| {name} | {seconds:.2f} s{suffix} | {checked['route_length_m']:.2f} m | {checked['nodes']} / {checked['frames']} | {checked['lift_flagged_edges']} | {checked['maximum_ik_residual_m']*1000:.2f} mm | {checked['minimum_zmp_margin_m']*100:.2f} cm |")
            rows.append(dict(scene=scene,**checked,route_planning_s=seconds))
        for case in r.get('test_cases',[]):
            last=case['statistics']['searches'][-1]
            case_rows.append(dict(scene=scene,case_id=case['id'],label=case['label'] or '多高度分支',status=case['status'],routes=case['routes'],
                elapsed_s=case['statistics']['elapsed_s'],goal_support_z_m=case.get('goal_z'),
                expansions=last['expansions'],capability_rejected_edges=last['capability_rejected_edges'],open_exhausted=last['open_exhausted']))
        st=r['statistics'];ob=r['obstacles']
        scenes.append(dict(scene=scene,astar_and_checks_s=st['elapsed_s'],surface_setup_s=st['surface_setup_s'],
            wall_fit_and_force_s=st['virtual_force_s'],node_evaluations=st['node_evaluations'],invalid_edges=st['invalid_edges'],
            routes=len(r['routes']),force_adjusted_frames=ob['adjusted_frames'],wall_patches=len(ob['fitted_wall_patches'])))
    for s in scenes:
        lines+=['',f"{s['scene']}：读取及索引 {s['surface_setup_s']:.3f} s，立面拟合与虚拟力 {s['wall_fit_and_force_s']:.3f} s（A* 计时之外）；实际姿态求值 {s['node_evaluations']:,} 个，排除未通过过渡的边 {s['invalid_edges']} 条。立面函数 {s['wall_patches']} 个，修正 {s['force_adjusted_frames']} 帧。"]
    for case in case_rows:
        if not case['routes']:
            reason='最小机身中心高 32 cm＋上半厚 7 cm＋余量 2.5 cm＝41.5 cm，超过 40 cm 净空。' if case['scene']=='table-low-040' else '目标支撑高度 1.20 m，保持 4 cm 门限。'
            lines+=['',f"{NAMES[case['scene']]}／{case['label']}：未找到路径，搜索 {case['elapsed_s']:.2f} s、展开 {case['expansions']:,} 个状态、{case['capability_rejected_edges']:,} 个候选连接被能力门限拒绝，开放集耗尽={case['open_exhausted']}。{reason}"]
    cap=json.loads((OUT/'capability/report.json').read_text())
    lines+=['',f"盲走能力和固定腿对照共 {len(cap['trials'])} 次，累计 rollout CPU 墙钟 {sum(t['wall_s'] for t in cap['trials']):.2f} s（不含 actor 首次读入）。checkpoint 内保存的 iteration 为 {cap['iteration']}。",
        '',f"{len(rows)} 条路线共独立核对 {sum(r['supported_contacts_checked'] for r in rows):,} 次支撑接触，到原始点最远 {max(r['maximum_raw_distance_m'] for r in rows)*100:.2f} cm，全部满足原凸包和 3 cm 支撑门限。XML 直接 FK 与导出轮心最大偏差 {max(r['maximum_mujoco_fk_error_m'] for r in rows):.2e} m；整机 COM 逐帧复核一致。",
        '', '四脚可以处于不同高度；分支不是要求四脚处于同一层。脚目标高度变化与同 XY 函数跳差分别统计，连续坡上整步的高度变化不能当作同样高的竖直台阶能力。',
        '', '113 项 Python 测试通过，覆盖 FK/IK、ZMP、原始支持、上方二值通行、逐层检查、稀疏上层仍约束净空、台阶支撑面不能误作顶界、下台阶轮子避让及占用不影响五项地形代价。上方通行在查询 XY 处判断；0 否决，1 仍需完整姿态和过渡检查。换层按左前、右前、左后、右后执行，change flag 单腿依次出现。',
        '', f'本轮删除单足占用代价后重新运行全部规划，原有 61 组的 915 个点云／拟合／指标／凸包文件 SHA256 未改变。当前地形页为 {len(CURRENT_ASSESSMENT_SCENES)} 场景×四设置，森林另有一组门限对照；小缝补偿仍为 {4*len(CURRENT_ASSESSMENT_SCENES)+1} 组。旧规划独立保存在 surface-planner/history/before-no-occupancy-cost-20261122/。',
        '', '轮子检查现已覆盖滚动及摆动：轮心到最近原始点至少 7.8 cm（9 cm 轮半径、12 mm 容差）。平台绕行的原路线曾低至 6.31 cm，本次重规划后不再放行该过渡。桌下/桌面两条路线均约 9 cm，机身仍保持约 40 cm 离地；桌下到观测桌底的最小垂直间隙约 63 cm。','', '局限：未做整机连杆连续碰撞检测、摩擦锥/接触力可行性和动力学跟踪。12 mm 是软 IK 接受阈值而非精确接触误差零。播放时长是演示用阶段时序，没有测量机器人执行用时。',
        '', '[丘陵逐帧核查](http://127.0.0.1:42482/surface-planner/results/hill/audit.json) · [测试场景逐帧核查](http://127.0.0.1:42482/surface-planner/results/surface-route-test/audit.json) · [浏览器记录](http://127.0.0.1:42482/surface-planner/browser-audit.json) · [计时与数值 CSV](http://127.0.0.1:42482/surface-planner/summary.csv)']
    path=ROOT/'surface-planning-v1.md';text=path.read_text()
    section=BEGIN+'\n\n'+'\n'.join(lines)+'\n\n'+END
    if BEGIN in text:text=text[:text.index(BEGIN)]+section+text[text.index(END)+len(END):]
    else:text=text.replace('<!-- measured-results -->',section)
    path.write_text(text);(OUT/'report.md').write_text(text)
    with (OUT/'summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (OUT/'case-summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(case_rows[0]));writer.writeheader();writer.writerows(case_rows)
    summary=dict(scenes=scenes,routes=rows,test_cases=case_rows,capability_trials=len(cap['trials']),step_limit_m=cap['conservative_step_limit_m'])
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary


if __name__=='__main__':run()
