"""Inspect recorded local execution and write a compact benchmark report."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def write_report(output):
    output = Path(output)
    results = json.loads((output/'results.json').read_text())
    protocol = json.loads((output/'protocol.json').read_text())
    config = protocol['config']
    audits, durations = [], []
    for result in results:
        name = result['name']
        history = json.loads((output/f'{name}.json').read_text())['records']
        with np.load(output/f'{name}.npz') as data:
            executed, start, goal, half = (data[k] for k in ('executed', 'start', 'goal', 'half'))
        translation = np.linalg.norm(np.diff(executed[:, :3], axis=0), axis=1)
        rotation = Rotation.from_euler('xyz', executed[:, 3:])
        angles = (rotation[:-1].inv()*rotation[1:]).magnitude()
        max_motion, max_proposal_angle = 0., 0.
        previous_end = 0
        for record in history:
            assert record['start_index'] == previous_end
            current = executed[previous_end]
            np.testing.assert_allclose(current[:3], record['sensed_center'], atol=1e-10)
            durations.append(record['seconds'])
            if 'local_reachability_route' in record:
                route = np.array(record['local_reachability_route'])
                assert np.linalg.norm(route[:, :3]-current[:3], axis=1).max() <= config['horizon']+1e-8
            for attempt in record.get('attempts', []):
                initial, final = np.array(attempt['initial']), np.array(attempt['final'])
                np.testing.assert_allclose(initial[0], current, atol=1e-10)
                np.testing.assert_allclose(final[[0, -1]], initial[[0, -1]], atol=1e-10)
                assert np.linalg.norm(initial[-1, :3]-current[:3]) <= config['horizon']+1e-8
                max_motion = max(max_motion, attempt['maximum_translation_step'])
                max_proposal_angle = max(max_proposal_angle, attempt['maximum_rotation_step'])
                # The sensed cloud must cover the whole body, including its
                # collision activation region, at every archived local state.
                for state in attempt.get('states', []):
                    poses = np.array(state['poses'])
                    reach = np.linalg.norm(poses[:, :3]-current[:3], axis=1).max()+np.linalg.norm(half)+.055
                    assert reach <= config['sensing_radius']+1e-8
            if 'end_index' in record:
                endpoint = record['end_index']
                segment = executed[previous_end:endpoint+1]
                length = np.linalg.norm(np.diff(segment[:, :3], axis=0), axis=1).sum()
                assert length <= config['advance']+1e-8
                previous_end = endpoint
        np.testing.assert_allclose(executed[0], start, atol=1e-10)
        assert previous_end == len(executed)-1
        assert translation.max(initial=0) <= config['command_translation']+1e-8
        assert angles.max(initial=0) <= config['command_rotation']+1e-8
        assert max_motion <= .01+1e-8 and max_proposal_angle <= np.radians(2)+1e-8
        assert result['goal_reached'] == (np.linalg.norm(executed[-1, :3]-goal[:3]) < config['goal_tolerance'] and
                                         (rotation[-1].inv()*Rotation.from_euler('xyz', goal[3:])).magnitude() < np.radians(.5))
        audits.append(dict(name=name, commands=len(translation), replans=len(history),
                           maximum_command_translation_m=float(translation.max(initial=0)),
                           maximum_command_rotation_rad=float(angles.max(initial=0)),
                           maximum_integration_translation_m=max_motion,
                           maximum_integration_rotation_rad=max_proposal_angle,
                           immutable_prefix_indices=True, local_horizon_only=True,
                           archived_states_inside_sensing_sphere=True))
    summary = dict(tasks=len(results), reached=sum(r['goal_reached'] for r in results),
                   planning_successes=sum(r['planning_success'] for r in results),
                   safely_stopped=sum(not r['goal_reached'] and r['continuous_lower_bound_against_surfaces_m']>=0 for r in results),
                   cycle_median_ms=float(np.median(durations)*1000), cycle_p95_ms=float(np.percentile(durations, 95)*1000),
                   cycle_max_ms=float(max(durations)*1000))
    (output/'motion_validation.json').write_text(json.dumps(dict(summary=summary, cases=audits), indent=2))
    imported = sum(bool(r['params'].get('source_repository')) for r in results)
    text = ['# 从起点开始的局部滚动规划', '',
            f'{len(results)} 个原始点云任务（{len(results)-imported} 个合成场景、{imported} 个开源世界局部任务）。端点在原任务附近按点云净空确定，未按规划结果筛选任务。', '',
            '## 执行流程', '',
            '- 已走路径固定。每轮查询当前位置 1.4 m 点云；局部位置/姿态网格搜索只覆盖 0.65 m，选择短期目标。',
            f"- {config['local_nodes']} 节点直连短轨迹，以射线虚拟力/力矩、牛顿—欧拉和 SO(3) 更新松弛最多 {config['relaxation_steps']} 步；前段不安全时缩短至首条可达边重试。",
            '- 仅提交通过全局部点云连续扫掠检查的前 0.10 m，拆成 ≤10 mm / ≤2° 的几何指令；然后从实际推进位置重规划。',
            '- 全局终点进入局部范围后才连接；没有预计算或输入完整避障路线。局部网格前端对绕行有贡献。',
            '- 运行后用独立密集表面做 SO(3) 插值检查。成功要求到达终点且连续无碰撞。未到达的安全前缀保留并计为失败。', '',
            '## 结果', '',
            f"到达终点 {summary['reached']}/{summary['tasks']}；到达且连续无碰撞 {summary['planning_successes']}/{summary['tasks']}；未到达但已走段连续无碰撞 {summary['safely_stopped']}。", '',
            '| 场景 | 停止原因 | 轮数 | 距终点/m | 已走段连续间隙下界/mm | 规划耗时/s |',
            '|---|---|---:|---:|---:|---:|']
    status = dict(goal_reached='到达', stalled='停滞', no_local_frontier='无局部前沿',
                  unsafe_prefix_rejected='拒绝不安全前段', replan_limit='轮数上限')
    for r in results:
        text.append(f"| {r['label']} | {status.get(r['status'], r['status'])} | {r['replans']} | {r['goal_error_m']:.3f} | {r['continuous_lower_bound_against_surfaces_m']*1000:.1f} | {r['planning_seconds']:.2f} |")
    text += ['', '## 耗时与适用范围', '',
             f"单轮墙钟耗时中位数 {summary['cycle_median_ms']:.0f} ms，P95 {summary['cycle_p95_ms']:.0f} ms，最大 {summary['cycle_max_ms']:.0f} ms（4 进程并行运行时统计，包含局部搜索与松弛）。尚未达到实机实时控制要求。", '',
             '使用离线多次扫描的局部截取，未模拟移动传感器新增回波/遮挡。虚拟速度在每段求解开始时重置；deltaT 为虚拟积分时间，几何指令无物理执行时刻。', '',
             '## 可核验记录', '',
             '- 每场景 `.json`：各轮局部初值、最终短轨迹、短期目标、局部可达搜索路径、恢复尝试、虚拟位姿/速度/加速度抽样及耗时。',
             '- 每场景 `.npz`：实际执行前缀、选定起终点、独立密集表面验证结果。',
             '- `motion_validation.json`：推进分段上限、积分上限、固定局部端点、已走段连续索引、局部范围及感知范围核验。',
             '- `protocol.json`、源输入原运行及 `source/`：配置、输入来源、实现快照。', '']
    if (output/'endpoint_geometry_validation.json').exists():
        text += ['开源世界的 16 个起终点另外通过完整原世界几何核验：20 mm 膨胀机身不与三角面相交，中心不在任何原始物体实体内。该核验未反馈给规划器。', '']
    (output/'REPORT.md').write_text('\n'.join(text))
    return summary
