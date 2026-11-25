"""Time the unchanged saved projection query without overwriting its results.

Nested spans report exclusive time, so an IK or tree call inside a pose/query is
counted once. Instrumentation overhead remains in the measured wall time.
"""
import argparse
from collections import defaultdict
from dataclasses import replace
from functools import wraps
import json
from pathlib import Path
import time

import numpy as np

from . import surface_astar as shared
from .assessment_export import finite_json,sha
from .layer_projection_planner import Config,LayerGraph,LayerSurfaceMap,ProjectionPlanner,RESULT,SurfaceRobot

LABELS={
    'surface_query':'曲面查询、凸包、法向与高度层筛选',
    'raw_tree':'原始点KD树检索（支撑及碰撞）',
    'foot_selection':'四足候选选择其余',
    'ik':'四腿IK与解析正运动学',
    'body_collision':'机身盒碰撞其余',
    'stability':'模型重心与准静态支撑裕量',
    'pose':'姿态组合、机身高度筛选其余',
    'layer_connections':'跨层候选及交接匹配其余',
    'search_other':'A*邻接、队列、缓存及记录其余',
    'rolling_other':'滚动与降身采样调度其余',
    'validation_other':'路线验证调度其余',
    'control_other':'规划控制与统计其余',
}


class Spans:
    def __init__(self):
        self.stack=[];self.phase='control';self.values=defaultdict(lambda:dict(exclusive_ns=0,inclusive_ns=0,calls=0))
        self.intervals=[];self.root_ns=None

    def wrap(self,function,category,phase=None,name=None):
        label=name or function.__qualname__
        @wraps(function)
        def timed(*args,**kwargs):
            previous=self.phase
            if phase is not None:self.phase=phase
            current=self.phase;frame=[0];self.stack.append(frame);start=time.perf_counter_ns()
            try:return function(*args,**kwargs)
            finally:
                elapsed=time.perf_counter_ns()-start;self.stack.pop()
                key=(current,category,label);value=self.values[key]
                value['exclusive_ns']+=elapsed-frame[0];value['inclusive_ns']+=elapsed;value['calls']+=1
                if self.stack:self.stack[-1][0]+=elapsed
                else:self.root_ns=elapsed
                if phase is not None:self.intervals.append(dict(phase=current,function=label,elapsed_s=elapsed/1e9))
                self.phase=previous
        return timed

    def report(self):
        total=self.root_ns/1e9;by_category=defaultdict(float);by_phase=defaultdict(float);functions=[]
        for (phase,category,name),v in self.values.items():
            seconds=v['exclusive_ns']/1e9;by_category[category]+=seconds;by_phase[phase]+=seconds
            functions.append(dict(phase=phase,category=category,function=name,calls=v['calls'],
                exclusive_s=seconds,inclusive_s=v['inclusive_ns']/1e9))
        assert abs(sum(by_category.values())-total)<1e-6
        categories=[dict(id=k,label=LABELS[k],exclusive_s=v,percent=100*v/total)
            for k,v in sorted(by_category.items(),key=lambda p:-p[1])]
        return dict(wall_s=total,categories=categories,
            phases=[dict(phase=k,elapsed_s=v,percent=100*v/total) for k,v in by_phase.items()],
            intervals=self.intervals,functions=sorted(functions,key=lambda v:-v['exclusive_s']))


class TreeProbe:
    def __init__(self,tree,spans,name):
        self.tree=tree
        self.query=spans.wrap(tree.query,'raw_tree',name=name+'.query')
        self.query_ball_point=spans.wrap(tree.query_ball_point,'raw_tree',name=name+'.query_ball_point')

    def __getattr__(self,name):return getattr(self.tree,name)


def run(scene,map_id,output):
    result_path=RESULT/scene/'result.json';saved=json.loads(result_path.read_text());before=sha(result_path)
    mapping=next(m for m in saved['projection_maps'] if m['id']==map_id)
    expected=next(r for r in saved['routes'] if r['map_id']==map_id)
    spec=next(s for s in saved['test_cases'] if s['id']==mapping['case_id'])
    cfg=replace(Config(**saved['config']),maximum_expansions=mapping.get('expansion_limit',saved['config']['maximum_expansions']))
    start=time.perf_counter();surface=LayerSurfaceMap(scene,cfg);surface_s=time.perf_counter()-start
    start=time.perf_counter();graph=LayerGraph(surface);surface.graph=graph;graph_s=time.perf_counter()-start
    start=time.perf_counter();robot=SurfaceRobot();robot_s=time.perf_counter()-start
    assert surface.hashes==saved['original_files_sha256']
    spans=Spans();patched=[]
    def patch(owner,name,category,phase=None):
        original=getattr(owner,name);patched.append((owner,name,original))
        setattr(owner,name,spans.wrap(original,category,phase))
    patch(LayerSurfaceMap,'query','surface_query')
    patch(shared.SurfaceMap,'near','surface_query')
    patch(shared.SurfaceMap,'contacts','foot_selection')
    patch(shared.Planner,'evaluate','pose');patch(shared.Planner,'fixed_pose','pose')
    patch(SurfaceRobot,'ik','ik');patch(SurfaceRobot,'com','stability')
    patch(shared,'support_margin','stability')
    patch(shared.SurfaceMap,'body_free','body_collision')
    patch(ProjectionPlanner,'macro_targets','layer_connections')
    patch(ProjectionPlanner,'edge_kind','layer_connections');patch(LayerGraph,'match','layer_connections')
    patch(ProjectionPlanner,'rolling_edge','rolling_other')
    patch(ProjectionPlanner,'search_projection','search_other','search')
    patch(ProjectionPlanner,'validate_projection','validation_other','validation')
    patch(ProjectionPlanner,'plan_projection','control_other')
    surface.raw=TreeProbe(surface.raw,spans,'raw');surface.rawxy=TreeProbe(surface.rawxy,spans,'rawxy')
    try:
        planner=ProjectionPlanner(surface,robot,cfg,graph,set(mapping['allowed_layers']))
        route,statistics=planner.plan_projection(spec)
    finally:
        for owner,name,original in reversed(patched):setattr(owner,name,original)
    assert route is not None
    profile=spans.report();profile.update(date='2026-11-25',scene=scene,map_id=map_id,
        scope='One instrumented repeat of the final successful overlay query, including its rolling-edge replan; earlier exhausted attempts not repeated.',
        accounting='Exclusive nested spans. Raw KD-tree calls are separated from their query/collision parents. Instrumentation overhead is included, not subtracted.',
        setup=dict(input_models_and_trees_s=surface_s,layer_graph_s=graph_s,robot_model_s=robot_s),
        historical_success_s=mapping['statistics']['elapsed_s'],planner_internal_wall_s=statistics['elapsed_s'],
        node_evaluations=statistics['node_evaluations'],invalid_rolling_edges=statistics['invalid_edges'],
        searches=[{k:v for k,v in s.items() if k!='tree'} for s in statistics['searches']],
        route=dict(nodes=len(route['nodes']),frames=len(route['frames']),crossings=len(route['events']),cost=route['cost']),
        source_sha256={**saved['sources_sha256'],'profile_layer_projection.py':sha(Path(__file__))},
        input_sha256=surface.hashes,saved_result_sha256=before,
        reproduction=f'python -m fsgp_bgk_repro.profile_layer_projection --scene {scene} --map {map_id}')
    assert len(route['nodes'])==len(expected['nodes']) and len(route['frames'])==len(expected['frames'])
    assert [tuple(p['key']) for p in route['nodes']]==[tuple(p['key']) for p in expected['nodes']]
    assert abs(route['cost']-expected['cost'])<1e-10
    for key in ('body','feet','normals','joints','owners'):
        np.testing.assert_allclose([f[key] for f in route['frames']],[f[key] for f in expected['frames']],rtol=0,atol=1e-12)
    assert [f['phase'] for f in route['frames']]==[f['phase'] for f in expected['frames']]
    assert sha(result_path)==before and all(sha(surface.folder/n)==h for n,h in surface.hashes.items())
    assert all(sha(Path(__file__).parent/n)==h for n,h in saved['sources_sha256'].items())
    profile['verification']=dict(passed=True,same_route_nodes=True,same_1302_frame_geometry=True,
        same_frame_phases=True,same_cost=True,saved_result_unchanged=True,fit_inputs_unchanged=True,
        original_runtime_sources_unchanged=True,exclusive_spans_sum_to_wall=True)
    output.mkdir(parents=True,exist_ok=True)
    (output/'profile.json').write_text(json.dumps(finite_json(profile),ensure_ascii=False,indent=2)+'\n')
    write_report(profile,output)
    print(json.dumps({k:profile[k] for k in ('wall_s','setup','phases','categories','verification')},ensure_ascii=False,indent=2),flush=True)
    return profile


def write_report(p,output):
    lines=['# 五平台规划耗时分布（2026-11-25）','',
        '复用原拟合数据，对最终成功的叠加图做一轮带计时器的重复实验。没有重复前两轮预算耗尽的尝试，也未修改规划器或原路线。',
        f"历史成功阶段为{p['historical_success_s']:.2f} s；本次带计时器实测{p['wall_s']:.2f} s。运行环境波动与计时器开销均可能影响绝对值，不能直接当作加速或减速倍数。",'',
        '## 按处理阶段','', '| 阶段 | 时间 | 占规划时间 |','| --- | ---: | ---: |']
    labels=dict(control='规划控制和记录',search='A*搜索及候选节点检查',validation='候选路线的滚动采样检查')
    for v in p['phases']:lines.append(f"| {labels[v['phase']]} | {v['elapsed_s']:.3f} s | {v['percent']:.2f}% |")
    lines+=['','## 按具体计算内容','',
        '下表为互不重复的独占时间：子调用已经从父项扣除。原始点KD树检索包含支撑、轮子和机身邻域查询，不能全部视作碰撞耗时。', '',
        '| 计算项 | 独占时间 | 占规划时间 |','| --- | ---: | ---: |']
    for v in p['categories']:lines.append(f"| {v['label']} | {v['exclusive_s']:.3f} s | {v['percent']:.2f}% |")
    lines+=['','## 每次搜索与路线检查','', '| 调用顺序 | 内容 | 时间 |','| --- | --- | ---: |']
    for k,v in enumerate(p['intervals'],1):lines.append(f"| {k} | {labels[v['phase']]} | {v['elapsed_s']:.3f} s |")
    lines+=['','## 准备阶段与验证','']
    setup=p['setup'];lines.append(f"模型读入／原始点与曲面搜索树准备{setup['input_models_and_trees_s']:.3f} s；连续分层和边界连接{setup['layer_graph_s']:.3f} s；机器人模型准备{setup['robot_model_s']:.3f} s。以上不计入规划百分比。")
    lines+=[f"新评估{p['node_evaluations']:,}个姿态候选；A*累计展开{sum(s['expansions'] for s in p['searches']):,}次；排除{p['invalid_rolling_edges']}条失败滚动边。返回{p['route']['nodes']}个路线节点、{p['route']['frames']}帧及{p['route']['crossings']}处跨层候选。",'',
        '逐帧机身、四足、法向、关节和函数归属均与原结果一致（1e−12容差），节点序列、阶段和代价一致；原拟合文件、原结果和规划器源码SHA256不变。跨层动作仍未验证。','',
        '[完整计时数据及复现命令](profile.json) · [当前工作流程](../../workflow.md) · [规划页面](../../?scene=four-platforms)']
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    rows=p['categories'];fig,ax=plt.subplots(figsize=(11,6));y=np.arange(len(rows))
    bars=ax.barh(y,[v['exclusive_s'] for v in rows],color='#408fc0')
    ax.set_yticks(y,[v['label'] for v in rows]);ax.invert_yaxis();ax.set_xlabel('独占时间（秒）')
    ax.set_title(f"五平台成功叠加规划：分项耗时（带计时器，总计{p['wall_s']:.2f}秒）")
    maximum=max(v['exclusive_s'] for v in rows);ax.set_xlim(0,maximum*1.30)
    for bar,v in zip(bars,rows):ax.text(bar.get_width()+maximum*.012,bar.get_y()+bar.get_height()/2,f"{v['exclusive_s']:.2f}s / {v['percent']:.1f}%",va='center',fontsize=9)
    ax.spines[['top','right']].set_visible(False);fig.tight_layout();fig.savefig(output/'breakdown.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scene',default='four-platforms')
    parser.add_argument('--map',default='all-platforms-overlay');parser.add_argument('--output',type=Path,default=RESULT.parent/'profiling/four-platforms-20261125')
    args=parser.parse_args();run(args.scene,args.map,args.output)
