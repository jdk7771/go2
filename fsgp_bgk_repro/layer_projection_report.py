"""Current layer-projection method and measured results, from saved audits."""
import csv
import json
from pathlib import Path
from .scene_fit_catalog import WEB
from .surface_astar import CASES
from .assessment_export import sha

ROOT=Path(__file__).resolve().parents[1]
OUT=WEB/'surface-planner'


def run():
    rows=[];scenes=[];failed=[];tables=['| 场景／规划图 | 状态 | A* 与滚动检查 | 路长 | 代价 | 跨层候选 |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for scene in CASES:
        folder=OUT/'results'/scene;r=json.loads((folder/'result.json').read_text());a=json.loads((folder/'audit.json').read_text())
        assert r['schema_version']==2 and a['passed'] and a['result_sha256']==sha(folder/'result.json')
        lookup={p['map_id']:p for p in a['routes']};route_lookup={p['map_id']:p for p in r['routes']}
        for mapping in r['projection_maps']:
            route=route_lookup.get(mapping['id']);check=lookup.get(mapping['id']);elapsed=mapping['statistics']['elapsed_s']
            label=r['name']+'／'+('本层' if mapping['mode']=='base' else '叠加层')+'（'+mapping['case_id']+'）'
            if route:
                tables.append(f"| {label} | 找到路径 | {elapsed:.2f} s | {check['route_length_m']:.2f} m | {route['cost']:.2f} | {check['layer_crossings']} |")
                rows.append(dict(scene=scene,**check,route_planning_s=elapsed))
            else:
                tables.append(f'| {label} | 未找到路径 | {elapsed:.2f} s | — | — | — |');failed.append(dict(scene=scene,map_id=mapping['id'],elapsed_s=elapsed))
        scenes.append(dict(scene=scene,routes=len(r['routes']),projection_maps=len(r['projection_maps']),
            surface_setup_s=r['statistics']['surface_setup_s'],astar_and_checks_s=r['statistics']['elapsed_s'],
            previous_attempt_s=r['statistics'].get('previous_attempt_elapsed_s',0.),retry_setup_s=r['statistics'].get('retry_setup_s',0.)))
    summary=dict(planner='layer-projection-astar',scenes=scenes,routes=rows,failed_maps=failed,
        crossing_height_exclusive_m=.40,threshold_source='User setting, not blind-policy calibration',
        endpoint_height_lift_rule=False,crossing_motion_validated=False)
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(dict.fromkeys(k for row in rows for k in row)));writer.writeheader();writer.writerows(rows)
    with (OUT/'case-summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['scene','map_id','elapsed_s']);writer.writeheader();writer.writerows(failed)
    method='''# 拟合曲面的分层投影 A*（更新 2026-11-25）

当前顺序：**原始XYZ → 局部二次拟合与凸包 → 指标和落足价值 → 连续曲面分层 → 本层A* → 叠加可达层再跑A* → 路线检查／失败重搜 → 可视化**。

当前是普通A*，加权A*、LPA*尚未实现或测试。[完整工作流程、公式和检查顺序](http://127.0.0.1:42482/surface-planner/workflow.md)为当前方法说明。主体表格保留2026-11-24的实验；2026-11-25仅对成功叠加阶段补做分项计时，单独列在后面，不覆盖原路线和历史计时。

1. **曲面直接参与计算。** 当前规划读取已保存的`qcircle-adaptive`函数；在原拟合点XY凸包内求`z=f_i(x,y)`及导数法向，同一XY允许多层。搜索时不重新拟合，显示三角形和补缝不作为搜索节点或新增落足支撑。
2. **先评价四足。** 坡度／法向变化／RMSE／最大误差／σ分别按45°／15°／2 cm／2 cm／1 cm归一化，权重0.30／0.20／0.15／0.10／0.10，总和0.85。单足沿用所属函数指标，四足平均为机身地形代价，再加IK、高度偏好和支撑裕量惩罚。上方占用比例不进入代价；RMSE及最大误差是训练残差。
3. **连续分层和跨层连接分开。** 原凸包间隙≤2.5 cm、共同XY的预测差≤2 cm，结合实际边界复查形成连续层；连续坡保持同层滚动。在不同层的实际交接边界计算`abs(f_i(q)-f_j(q))`，高差严格<40 cm、水平间隙≤2.5 cm才建立跨层连接，重叠位置不能任意切层。40 cm是用户设置，未标定为盲走能力。
4. **A*搜索机身候选。** 状态为16 cm XY网格、8种航向和参考拟合函数编号；每个状态同时求四个支撑点。普通边包含相邻位置和转向，跨层边来自交接边界。按累计代价加到目标的XY直线距离排序；边代价为`距离×(1+2×目标节点代价)+0.015×转向45°档数`。
5. **每个节点检查四足和机身。** 从名义四足及XY各±3.5 cm的9个附近位置选曲面支撑；由四足支撑平面确定机身俯仰／横滚，测试中心离地32／36／40／44／48／52 cm。检查原凸包、原始点距离≤3 cm、IK轮心残差≤12 mm、关节限位、机身／轮子碰撞及准静态支撑裕量≥1 cm。机身碰撞盒75×23×14 cm、轮半径9 cm，轮接触为球形包络近似。
6. **上方净空为独立0/1检查。** 同一XY取高出当前曲面2 cm以上的最近上层；净空≥41.5 cm才保留该层候选，再按实际姿态检查顶部余量。无上层仅表示该项未否决，后续仍检查碰撞。净空42 cm时中心需要降至32 cm，不能保持40 cm。上方几何不随允许落足层过滤。
7. **本层和叠加图分别搜索。** 先只允许起始连续层；存在其他边界可达层时，将其XY和代价叠加再规划，保留真实Z、函数和层编号。每张图最多返回一条路线，未枚举所有层组合，也未规定平台必经点。
8. **检查路线并处理失败。** 滚动段按≤2 cm和≤7.5°采样，四足保持各自原连续层，复查支撑、IK、碰撞和准静态稳定性；失败边被禁用，再从头搜索，复用同一规划器内的姿态／查询缓存。扩大预算重试会新建规划器。跨层仅记录有支撑的两端及`motion_validated=false`，紫色虚线是连接候选，未生成逐腿抬腿或lift/change序列；旧4 cm跳变触发规则关闭。

ZMP采用整机重心在支撑平面上的竖直投影，裕量按XY支撑凸包计算；机身姿态来自四足支撑平面和高度选择，没有进行动态ZMP轨迹优化。跨层过程、腿部全过程碰撞及动态执行尚未验证。

## 实测结果

计时为本机单次 CPU 运行，复用拟合数据。下表的 A* 与滚动检查不含读取数据、分层及边界构图，也不含网页渲染。叠加规划可以选择继续走原层；增加第二层不会强迫路径登上它。

'''
    checked=sum(p['supported_contacts_checked'] for p in rows)
    validation_path=OUT/'preservation-audit.json'
    validation=json.loads(validation_path.read_text()) if validation_path.exists() else {}
    tests=validation.get('python_tests')
    test_text=f'{tests} 项 Python 测试通过。' if tests else ''
    underpass_rows=[p for p in rows if p.get('underpass_frames')]
    underpass=''
    if underpass_rows:
        underpass='''

## 第二个平台改为空心并延伸：下方通行对照（2026-11-24）

原场景第一台仍为实心 15 cm 矮台，原第二台为实心 80 cm 高台。第二台改为 10 cm 厚的顶板和四角 12 cm 支腿，恢复下方原始地面点。初始航向为 0：前方 +X，右侧 −Y。**前后 X 保持 [1, 2.6] m，右侧 Y 从 −2.3 延伸到 −4 m**，左边沿仍为 Y=−0.9 m。两版分别为顶高 80／52 cm、下方净空 70／42 cm。标签只核查几何，拟合和规划输入仍是 XYZ。此前沿 +X 延伸的错误方向版本已归档。

起点 (-3.2, -1.6)，目标 (3.2, -1.6)，均在地面；目标现在位于第二台前方出口外，可以核查驶入、从下方通过、驶出及恢复机身高度。沿用本层／叠加两次 A*，没有指定路径必须如何绕过第一台，也没有把顶板删掉或把占用强制改成可通行。

| 净空／规划图 | 平台下核查帧 | 最低机身中心离地 | 最小机身顶部余量 | 降身帧 | 路长 | 搜索与滚动检查 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
'''
        for p in underpass_rows:
            underpass+=f"| {p['underpass_clearance_m']*100:.0f} cm／{'本层' if p['map_mode']=='base' else '叠加'} | {p['underpass_frames']} | {p['underpass_minimum_body_height_m']*100:.0f} cm | {p['underpass_minimum_ceiling_gap_m']*100:.1f} cm | {p['body_lowering_frames']} | {p['route_length_m']:.2f} m | {p['route_planning_s']:.2f} s |\n"
        underpass+='''
42 cm 版本在进入顶板前降低机身，平台下中心离地 32 cm，机身半高 7 cm，因此顶部还剩 3 cm，大于当前 2.5 cm 几何余量；在同一平台下支撑姿态强制保持 40 cm 的对照被否决。70 cm 版本平台下保持 40 cm 中心高度，该对照可行。两者都检查了真实顶板、四腿观测支撑、IK、轮子与机身碰撞、准静态 ZMP；这是模型几何与采样轨迹检查，没有执行盲走策略或机器人实测。若叠加路线经过第一矮台，其跨层段仍只是未验证动作的连接候选；本层路线为连续滚动和降身。

[70 cm 下方通行](http://127.0.0.1:42482/surface-planner/?scene=platform-underpass-070) · [42 cm 降身通行](http://127.0.0.1:42482/surface-planner/?scene=platform-underpass-042)。顶板拟合面默认蓝色不透明，勾选“透视上层”才变为 18% 透明以查看下方，外沿保留；此前自动 6% 透明导致顶板几乎不可见，属于显示问题。

### 上方通行 0/1 如何计算

查询 XY 时，只取原始拟合凸包覆盖该 XY 的二次函数，计算各自真实 Z 并合并相差小于 1.2 cm 的重复预测。对当前高度，选高差>2 cm 的最近上层；净空 g=f_上层(x,y)−f_当前(x,y)。**g≥41.5 cm 为 1，否则为 0**。无上层时本项记 1，表示没有拟合上层否决，并不证明全空间无障碍。

41.5 cm=最低机身中心 32 cm+机身半高 7 cm+余量 2.5 cm，是当前几何配置。42 cm 和 70 cm 都通过初筛，40 cm 被否决。0/1 不代表可以按正常高度通过：节点还要选机身高度，要求机身顶面加 2.5 cm 不超过最近顶板，并检查整个机身、轮子和原始点碰撞、IK、ZMP。因此 42 cm 净空必须降到中心 32 cm；40 cm 中心姿态被否决。网页绿点=1、红点=0，显示的是查询点的净空初筛；它与六项诊断指标中的上方占用比例分开。

[错误方向版本独立留档](http://127.0.0.1:42482/surface-planner/history/before-right-side-correction-20261124/?scene=platform-underpass-042)。
'''
    footer=f'''

本轮 {len(scenes)} 个规划场景，{sum(s['projection_maps'] for s in scenes)} 张规划图，{len(rows)} 条成功路线，{len(failed)} 张图未找到路径；每张图可能因检查失败或预算耗尽而重试A*，不将图数等同于A*调用次数。独立核对 {checked:,} 次支撑接触；{test_text}其他 69 组、1035 个拟合及指标文件保持不变；五平台链替换原四平台场景的四组设置，旧数据独立归档。首页仍为 18 场景、73 组。

## 查看与复现

[高低台对比](http://127.0.0.1:42482/surface-planner/?scene=flat-corridor-platforms) · [丘陵滚动](http://127.0.0.1:42482/surface-planner/?scene=hill)。路径选项可切换本层／叠加层；“投影代价图（XY）”显示搜索位置的投影代价，白低红高，紫色虚线表示跨层候选。三维图保留原曲面高度。

依次运行 `python -m fsgp_bgk_repro.layer_projection_planner`、`surface_planner_audit`、`surface_planner_report`、`publish_metric_viewer`（后三者同模块前缀）。本机使用系统 `/usr/bin/python3` 及单线程 BLAS。命令详见项目 `step.md` 第 27 节。

[逐场景数据](http://127.0.0.1:42482/surface-planner/summary.json) · [CSV](http://127.0.0.1:42482/surface-planner/summary.csv) · [最终核查](http://127.0.0.1:42482/surface-planner/preservation-audit.json) · [上一版独立查看器](http://127.0.0.1:42482/surface-planner/history/before-layer-projection-20261123/index.html)。
'''
    four=''
    four_path=OUT/'results/four-platforms/audit.json'
    if four_path.exists():
        data=json.loads(four_path.read_text())['four_platforms']
        four='''

## 五平台通行链：仅保留全程规划

20×8 m 平地，初始朝向 +X，左侧 +Y。2、3、4 台均覆盖 Y=[−4,4] m 的完整宽度，不能侧绕；第 1 台可从左侧绕行，第 5 台左侧留 1.2 m 小路。第 4 台顶高改为 30 cm、净空 20 cm：下方否决，但台面高差小于 40 cm，可建立跨层候选。第 5 台顶高 78 cm、净空 38 cm，上下均不通过，只能尝试小路。其板内间距40 cm也被41.5 cm净空规则否决，避免把台板底面当作落足面并站进板内；这一设置不代表算法已能对任意厚实体做体积碰撞。

只运行 (-9.2,−2.4) → (9.2,−2.4) 这一条全程查询，分别在本层、可跨层叠加图运行 A*；删除本页原来的四个局部测试与三个否决目标。没有添加必经点，也没有强制指定五个平台的走法。仍从 XYZ 拟合、按原凸包查询，沿用五项代价、IK、碰撞和准静态支撑检查。

| 平台 | 顶高 | 净空 | 横跨全宽 | 下方 0/1 | 从地面直接上台 | 本次全程实际结果 |
| --- | ---: | ---: | --- | --- | --- | --- |
'''
        for p in data['platforms']:
            gap='实心' if p['clearance_m'] is None else f"{p['clearance_m']*100:.0f} cm"
            flag='无地面支撑' if p['overhead_passable'] is None else str(p['overhead_passable'])
            four+=f"| {p['platform']} | {p['top_height_m']*100:.0f} cm | {gap} | {'是' if p['full_width'] else '否'} | {flag} | {'可作为候选' if p['direct_climb_candidate'] else '超过门限'} | {p['actual_outcome']} |\n"
        saved=json.loads((OUT/'results/four-platforms/result.json').read_text())
        attempts=[]
        for mapping in saved['projection_maps']:
            label='本层图' if mapping['mode']=='base' else '叠加图'
            for index,attempt in enumerate(mapping.get('previous_attempts',[]),1):
                attempts.append(dict(label=f'{label}／前次尝试{index}',limit=attempt['expansion_limit'],
                    status=attempt['status'],statistics=attempt['statistics']))
            attempts.append(dict(label=label+('／扩大预算重试' if mapping.get('previous_attempts') else ''),
                limit=mapping.get('expansion_limit',saved['config']['maximum_expansions']),
                status=mapping['status'],statistics=mapping['statistics']))
        four+='\n| 全程地图／尝试 | A*调用次数 | 累计节点展开 | 每次展开预算 | 搜索与检查 | 实际结果 |\n| --- | ---: | ---: | ---: | ---: | --- |\n'
        for attempt in attempts:
            stats=attempt['statistics'];searches=stats['searches']
            state='找到路线' if attempt['status']=='success' else ('预算耗尽' if searches[-1]['expansions']>=attempt['limit'] else '未找到路线')
            four+=f"| {attempt['label']} | {len(searches)} | {sum(s['expansions'] for s in searches):,} | {attempt['limit']:,} | {stats['elapsed_s']:.2f} s | {state} |\n"
        calls=sum(len(a['statistics']['searches']) for a in attempts)
        total_s=sum(a['statistics']['elapsed_s'] for a in attempts)
        setup_s=saved['statistics']['surface_setup_s']+saved['statistics'].get('retry_setup_s',0.)
        four+=f'\n本场景合计调用 **{calls} 次A***，搜索与路线检查共 **{total_s:.2f} s**；读取、分层和重试准备另约 **{setup_s:.2f} s**。预算指每次A*允许的最大节点展开次数，耗尽表示该次提前停止，不证明无路。拟合、结果导出、独立核查和浏览器不计入以上规划时间。\n'
        previous=sum(m['previous_attempt_elapsed_s'] for m in data['planning_maps'])
        if previous:
            four+=f'\n叠加搜索首次22,000节点预算耗尽，耗时{previous:.2f} s；只扩大该图预算重跑，未改A*、代价或约束。首次失败完整保存在previous_attempts中，未包含在上表最后一次搜索计时内。\n'
            four+='\n复现时在首次规划失败后、核查前运行 `python -m fsgp_bgk_repro.retry_layer_projection --scene four-platforms --map all-platforms-overlay --budget 66000`。详见step.md第29节；该命令只重试此图，不增加局部查询。\n'
        for m in data['planning_maps']:
            if m['a_star_searches']>1:
                four+=f"\n{m['map_id']} 的最终阶段包含 {m['a_star_searches']} 次A*、合计 {m['total_expansions']:,} 次节点展开：滚动检查不合格的边被排除后再搜索。最后一次单独展开{m['expansions']:,}次，表中给的是两次累计数。该历史119秒记录没有分项计时；2026-11-25的独立计时补测见下方记录。\n"
        if not data['full_route_found']:
            four+='\n本次没有返回完整路线，因此上表不把搜索树节点或理想走法写成已走过的平台结果。达到展开预算表示有限搜索未找到路线，不证明所有运动均不可行。页面显示拟合场景、已验证初始姿态和起终点。\n'
        four+='''
40 cm 为用户设定的跨层规划门限，未标定为盲走能力。跨层结果若存在，仍只是边界两端姿态可行的连接候选，未验证抬腿动作或动态执行。

[42482 五平台全程结果](http://127.0.0.1:42482/surface-planner/?scene=four-platforms)。默认显示蓝色拟合面，勾选“透视上层”查看下方。[原四平台实验独立留档](http://127.0.0.1:42482/surface-planner/history/before-five-platform-chain-20261124/?scene=four-platforms)，不再作为本页局部选项。
'''
    profiling='';profile_path=OUT/'profiling/four-platforms-20261125/profile.json'
    if profile_path.exists():
        p=json.loads(profile_path.read_text())
        assert p['verification']['passed'] and p['saved_result_sha256']==sha(OUT/'results/four-platforms/result.json')
        profiling=f'''

## 分项计时补测（2026-11-25）

复用同一场景、曲面、状态、约束和66,000次展开预算，单独重复最终成功叠加阶段。原路线与拟合数据保持不变。带计时器实测 **{p['wall_s']:.2f} s**，原历史阶段为119.08 s；运行环境和计时器开销影响绝对值，不能直接作为算法加速倍数。

下表为互不重复的独占时间，子调用已从父项扣除。原始点检索涉及支撑及碰撞，不能全归为碰撞；A*其余项包括邻接、缓存和记录，不能等同于堆操作。

| 计算内容 | 时间 | 占本轮规划 |
| --- | ---: | ---: |
'''
        for item in p['categories']:
            profiling+=f"| {item['label']} | {item['exclusive_s']:.3f} s | {item['percent']:.2f}% |\n"
        profiling+='\n| 调用顺序 | 内容 | 时间 |\n| --- | --- | ---: |\n'
        phase_names=dict(search='A*搜索与节点检查',validation='路线采样检查')
        for k,item in enumerate(p['intervals'],1):
            profiling+=f"| {k} | {phase_names[item['phase']]} | {item['elapsed_s']:.3f} s |\n"
        profiling+='''
本轮结果与原136个路线节点、1,302帧的几何、关节、阶段及代价一致。首次搜索承担大部分新姿态求解；第二次从头重建A*队列，但复用姿态和曲面查询缓存，仅耗约2.56 s。不能根据29,730次重复展开就认为再次支付了首轮的IK和碰撞开销。曲面函数是预先拟合的，本轮曲面查询不包含重新拟合。

[分项报告与口径](http://127.0.0.1:42482/surface-planner/profiling/four-platforms-20261125/report.md) · [耗时图](http://127.0.0.1:42482/surface-planner/profiling/four-platforms-20261125/breakdown.png) · [JSON与函数调用次数](http://127.0.0.1:42482/surface-planner/profiling/four-platforms-20261125/profile.json)。
'''
    report=method+'\n'.join(tables)+underpass+four+profiling+footer
    (ROOT/'surface-planning-v1.md').write_text(report);(OUT/'report.md').write_text(report)
    (OUT/'layer-choice.md').write_text(report)
    path=ROOT/'result.md';text=path.read_text();begin='<!-- layer-projection:start -->';end='<!-- layer-projection:end -->'
    section=begin+'\n\n<a id="layer-projection-planning"></a>\n\n## 13. 分层滚动与可跨层投影叠加（实验 2026-11-24；计时补测 2026-11-25）\n\n'+method[method.index('当前顺序：'):]+ '\n'.join(tables)+underpass+four+profiling+footer+'\n'+end+'\n'
    if begin in text:text=text[:text.index(begin)]+section+text[text.index(end)+len(end):]
    else:text=text.rstrip()+'\n\n'+section
    path.write_text(text.rstrip()+'\n')
    return summary


if __name__=='__main__':run()
