"""A short measured explanation of binary headroom and the two A* alternatives."""
import json
from pathlib import Path
from .scene_fit_catalog import WEB
from .assessment_export import sha


def run():
    root=Path(__file__).resolve().parents[1];folder=WEB/'surface-planner/results/layer-choice'
    r=json.loads((folder/'result.json').read_text());audit=json.loads((folder/'audit.json').read_text())
    if r.get('schema_version',1)>=2:
        from .layer_projection_report import run as report_projection
        return report_projection()
    fit=json.loads((WEB/'convex-assessment/qcircle-adaptive/layer-choice/data.json').read_text())
    assert audit['passed'] and audit['result_sha256']==sha(folder/'result.json')
    rows=[]
    for route,a in zip(r['routes'],audit['routes']):
        rows.append(dict(id=route['case_id'],label=route['label'],cost=route['cost'],length_m=a['route_length_m'],
            planning_s=route['planning_statistics']['elapsed_s'],frames=a['frames'],
            change_actions=sum(p['phase']=='prepare' and any(p.get('change_flag',[])) for p in route['frames']),
            minimum_zmp_margin_m=a['minimum_zmp_margin_m'],minimum_ceiling_gap_m=a['minimum_observed_ceiling_gap_m']))
    lines=['## 11. 上方二值通行、向上选层与降身／上桥对比（2026-11-22）','',
        '[打开同一场景的两条路线](http://127.0.0.1:42482/surface-planner/?scene=layer-choice)。在“路径”中切换压低穿下／逐腿上桥；勾选“上方通行 0/1”查看搜索位置的绿点 1、红点 0。','',
        '### 如何做','',
        '1. **判断上方。** 在查询 XY 上，收集凸包覆盖此处的有效函数并按高度排序。相对当前支撑面，取最近的真正上层作顶界；当前脚已踩住的台阶属于支撑，不误当头顶。上层即使原始点较稀疏，仍用其有效拟合面约束净空。',
        '2. **给出 0/1。** 当前最低机身中心候选为 32 cm，加机身半高 7 cm、安全余量 2.5 cm，所需最低净空 41.5 cm。局部净空不足则为 0，直接否决这个下方支撑候选；足够或没有更上层拟合面则为 1，继续检查原始点支持、IK、轮子和 ZMP。1 仅表示上层条件允许，不代表机器人已完整可通行。',
        '3. **继续向上看。** 不因某一层为 0 就结束：继续枚举更高函数，记录父子高度关系。上层要有净空、原始支持且单次高度切换≤盲走保守门限 4 cm，才标为可切换候选；完整姿态及过渡仍需通过检查。超过能力的层不能靠一个 flag 跳上去。',
        '4. **分别规划。** 同一对起终点，分别运行下层 A* 和必须经过上桥区域的上层 A*。下层路线限制地面支撑；上层路线必须真正踩上桥面再到终点，不能碰一下低台阶就算上桥。两条都可行时比较同一工程代价，选较小者；只有一条可行则只选它。',
        '5. **逐腿换层。** 换层前先在同层完成朝向对齐；换层保持朝向，四足按左前→右前→左后→右后依次重新放置。每个单腿阶段只有对应的 change flag 为真，其他三足承担支撑并检查 ZMP。原先“脚目标高差>4 cm”的 lift flag 单独保留，二者含义不同。','',
        '### 测试场景','',
        '12×3 m 平地通道，两侧有高墙，桥板横跨全宽；桥底 42 cm、桥面 48 cm。侧方没有绕过桥板的地面路线。下方通道需要把机身中心降到 32 cm；上方从两端各 12 级台阶抵达桥面，每级高 4 cm、踏面长 32 cm。没有把已测的 4 cm 单级能力放大成直接爬 48 cm。全部曲面从同一原始 XYZ 拟合，不使用部件标签规划。','',
        '| 方案 | 路长 | A* 工程代价 | 规划及过渡检查 | 逐腿 change 动作 | 最小 ZMP 裕量 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for a in rows:
        lines.append(f"| {a['label']} | {a['length_m']:.2f} m | {a['cost']:.2f} | {a['planning_s']:.2f} s | {a['change_actions']} | {a['minimum_zmp_margin_m']*100:.2f} cm |")
    lines += ['',f"当前代价选择 **{next(a['label'] for a in rows if a['id']==r['recommended_case_id'])}**。单足代价仅含坡度、平滑性、RMSE、最大误差及不确定性，权重依次为 0.30／0.20／0.15／0.10／0.10，合计 0.85，不重新缩放；上方占用已删除，净空仍由上方 0/1 和碰撞检查判断。路径代价还综合距离、IK、高度、转向和换层足点数，没有秒的单位。上桥的额外换层代价为每只需切换的脚 0.08，与下层使用同一公式。计时为 CPU 单线程一次运行，复用曲面，不含拟合、索引读取、立面修正和网页；不等于机器人执行时长。",'',
        '能力来源需要分清：4 cm 来自既有盲走模型的上台阶仿真测试（51 次总记录中的盲走部分）；32 cm 最低中心候选与 41.5 cm 净空来自当前 IK／机身几何设置。该盲走 actor 只有速度指令，未测出可控降身能力；下降暂共用保守高度门限，也尚未独立标定。网页两条轨迹均未由 actor 跟踪执行。','',
        f"拟合统计单独计算：{fit['raw_points']:,} 个原始点、{fit['centers']:,} 个候选中心、{fit['actual_solve_count']:,} 次实际主求解、{fit['valid']:,} 个有效且显示的函数。主求解包含自适应半径尝试，不等于函数数；拟合、指标与显示几何合计 {fit['timings']['total_compute_ms']/1000:.2f} s。",'',
        '独立逐帧核查同时检查目标接触点及 FK 实际接触点的原凸包和原始点≤3 cm，检查 IK≤12 mm、轮心原始点距离≥7.8 cm、机身余量≥2.5 cm及两套接触点支撑多边形的 ZMP 裕量≥1 cm，另验证上层到达、侧绕禁止、四足 change 顺序及五项代价。当前 113 项 Python 测试通过。本轮只重算规划，原有 61 组共 915 个拟合数据文件不变；地形首页仍为 15 场景、61 组（含森林对照），六项诊断指标保留。含占用代价的旧规划保存在 surface-planner/history/before-no-occupancy-cost-20261122/。','',
        '[逐帧核查](http://127.0.0.1:42482/surface-planner/results/layer-choice/audit.json) · [局部高度关系与搜索树](http://127.0.0.1:42482/surface-planner/results/layer-choice/layer-tree.json) · [网页核查](http://127.0.0.1:42482/surface-planner/layer-choice-browser-audit.json) · [简明全流程](http://127.0.0.1:42482/surface-planner/workflow.md)。']
    text='\n'.join(lines)+'\n';(WEB/'surface-planner/layer-choice.md').write_text('# 二值上方通行与两种路线\n\n'+text)
    path=root/'result.md';old=path.read_text();begin='<!-- layer-choice:start -->';end='<!-- layer-choice:end -->';section=begin+'\n\n<a id="layer-choice-planning"></a>\n\n'+text+'\n'+end+'\n\n'
    if begin in old:old=old[:old.index(begin)]+section+old[old.index(end)+len(end):].lstrip('\n')
    else:old=old.replace('## 11. 历史实验归档',section+'## 12. 历史实验归档')
    path.write_text(old);(WEB/'surface-planner/layer-choice-summary.json').write_text(json.dumps(dict(routes=rows,recommended=r['recommended_case_id']),ensure_ascii=False,indent=2)+'\n')
    return rows


if __name__=='__main__':run()
