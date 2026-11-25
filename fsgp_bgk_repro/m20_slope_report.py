"""Export plots and replay saved MuJoCo states for the M20 slope experiment."""
import argparse
import html
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from .m20_slope_capability import OUTPUT, make_model


def replay(folder, trial, filename):
    from PIL import Image, ImageDraw
    saved = np.load(folder/trial['trace_file'])
    model = make_model(trial['angle_deg'], trial['friction'], trial['physics_dt_s'])
    model.vis.global_.offwidth, model.vis.global_.offheight = 640, 360
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [1.6, 0., trial['ramp_rise_m']/2+.25]
    camera.distance, camera.azimuth, camera.elevation = 6.4, 95., -17.
    option = mujoco.MjvOption()
    option.geomgroup[1] = 0
    renderer = mujoco.Renderer(model, height=360, width=640)
    command = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-vcodec', 'rawvideo',
               '-pix_fmt', 'rgb24', '-s', '640x360', '-r', '25', '-i', '-', '-an',
               '-c:v', 'libx264', '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p',
               '-movflags', '+faststart', str(folder/filename)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for index in range(0, len(saved['qpos']), 2):
            data.qpos[:] = saved['qpos'][index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 640, 42), fill=(15, 23, 42))
            draw.text((12, 6), f"M20 model_{trial['checkpoint_iteration']} | {trial['angle_deg']:g} deg | command {trial['command_speed_m_s']:g} m/s", fill='white')
            draw.text((12, 23), f"t={saved['trace'][index,0]:.2f}s  x={data.qpos[0]:.2f}m  | {'PASS' if trial['success'] else 'FAIL'}: {trial['stop_reason']}", fill='white')
            process.stdin.write(np.asarray(frame).tobytes())
            if index == (len(saved['qpos'])-1)//2*2:
                frame.save(folder/filename.replace('.mp4', '.png'))
    finally:
        process.stdin.close()
        process.wait()
        renderer.close()
    if process.returncode:
        raise RuntimeError(f'ffmpeg failed: {process.returncode}')
    print(f'Replayed {filename}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--videos', action='store_true')
    args = parser.parse_args()
    folder = args.output
    report = json.loads((folder/'report.json').read_text())
    rows = [s for s in report['summaries'] if s['friction'] == 1 and s['physics_dt_s'] == .005]
    standard = [s for s in rows if s['speed_m_s'] == .4]
    colors = {14999: '#2563eb', 19999: '#dc2626'}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for iteration in [14999, 19999]:
        subset = [s for s in standard if s['checkpoint_iteration'] == iteration]
        axes[0].plot([s['angle_deg'] for s in subset],
                     [s['successes']/s['trials'] for s in subset], 'o-', color=colors[iteration],
                     label=f'model_{iteration}')
        axes[1].plot([s['angle_deg'] for s in subset], [s['mean_maximum_x_m'] for s in subset],
                     'o-', color=colors[iteration], label=f'model_{iteration}')
    axes[0].set(xlabel='Ramp angle (degrees)', ylabel='Passing initial poses / tested poses',
                title='0.4 m/s command, friction = 1.0', ylim=(-.05, 1.08))
    axes[1].set(xlabel='Ramp angle (degrees)', ylabel='Mean maximum base x (m)',
                title='Progress including failed trials')
    axes[1].axhline(2.8, color='#64748b', linestyle='--', label='Crest x = 2.8 m')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(folder/'slope-results.png', dpi=160)
    plt.close(fig)
    best = []
    for iteration in [14999, 19999]:
        for speed in sorted({s['speed_m_s'] for s in rows}):
            passing = [s for s in rows if s['checkpoint_iteration'] == iteration
                       and s['speed_m_s'] == speed and s['all_passed']]
            if passing:
                best.append(max(passing, key=lambda s: s['angle_deg']))
    selected = []
    for iteration in [14999, 19999]:
        edge = max(s['angle_deg'] for s in standard if s['checkpoint_iteration'] == iteration and s['all_passed'])
        for angle in [edge, edge+1]:
            selected.extend(s for s in standard if s['checkpoint_iteration'] == iteration and s['angle_deg'] == angle)
    video_trials = []
    edge = max(s['angle_deg'] for s in standard if s['checkpoint_iteration'] == 19999 and s['all_passed'])
    for angle in [edge, edge+1]:
        trial = next(r for r in report['trials'] if r['checkpoint_iteration'] == 19999 and r['angle_deg'] == angle
                     and r['seed'] == 0 and r['command_speed_m_s'] == .4 and r['friction'] == 1
                     and r['physics_dt_s'] == .005)
        filename = f'model_19999-{angle:g}deg.mp4'
        if args.videos:
            replay(folder, trial, filename)
        video_trials.append((trial, filename))
    def table_row(s):
        return (f"<tr><td>model_{s['checkpoint_iteration']}.pt</td><td>{s['speed_m_s']:g}</td>"
                f"<td>{s['angle_deg']:g}°</td><td>{s['rise_m']:.3f} m</td>"
                f"<td>{s['successes']}/{s['trials']}</td></tr>")
    head = '<tr><th>Checkpoint</th><th>指令速度 m/s</th><th>坡度</th><th>2 m 水平坡段高差</th><th>通过数/初始姿态数</th></tr>'
    videos = ''.join(f'<figure><video controls preload="metadata" src="{name}" poster="{name.replace(".mp4", ".png")}"></video>'
                     f'<figcaption>19999 · {r["angle_deg"]:g}° · {"通过" if r["success"] else "未通过"} · '
                     f'0.4 m/s · seed 0；从已保存的仿真状态逐帧回放，实际速度。</figcaption></figure>'
                     for r, name in video_trials)
    env = ('<p>同一个 M20 模型，57 维盲走观测、16 维动作；没有地形观测。'
           '平地起步 x=−0.65 m，0.5 s 后发送前进指令；坡段 x=0.8–2.8 m，宽 4 m，无入口台阶，顶部接平地。'
           '默认物理频率 200 Hz、控制频率 50 Hz、摩擦系数 μ=1.0。'
           'seed 0 为标准初始位姿，其余只扰动 yaw ±0.025 rad、横向 ±0.015 m、纵向 ±0.05 m。'
           '速度 0.2 m/s 限时 60 s，其余限时 30 s。</p>'
           '<p>通过要求：四轮全部越过坡顶 0.10 m，机身越过坡顶 0.55 m，且在顶部保持直立前进 0.5 s。'
           '翻倒、机身持续触地、机身偏离中心线 0.75 m 或超时均未通过。'
           '本实验只增加小幅初始姿态扰动，未开启观测噪声、质量随机化和执行延迟。</p>'
           '<p>环境复核：本机 GPU 正常；本次没有运行原始 IsaacLab 测试。'
           '找到的 Isaac Sim 5.1 独立安装中，isaaclab、isaaclab_rl、rsl_rl、gymnasium 和 rl_training 未配置到 Python 环境。'
           '当前报告全部为 MuJoCo 移植测量；NumPy 策略已与从原始 checkpoint 加载的 PyTorch 前向输出对照。</p>')
    checks = [s for s in report['summaries'] if s['friction'] != 1 or s['physics_dt_s'] != .005]
    check_table = ''.join('<tr>' + ''.join(f'<td>{value}</td>' for value in [s['checkpoint_iteration'],s['speed_m_s'],s['angle_deg'],s['friction'],s['physics_dt_s'],f"{s['successes']}/{s['trials']}"]) + '</tr>' for s in checks)
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>M20 盲走 checkpoint 爬坡实测</title><style>
body{{font:16px/1.7 system-ui,sans-serif;max-width:1080px;margin:32px auto;padding:0 20px;color:#172033;background:#f8fafc}}
table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:9px;border:1px solid #dbe2ea;text-align:left}}
img,video{{width:100%;border-radius:8px}}figure{{margin:20px 0}}figcaption{{font-size:14px;color:#475569}}code{{word-break:break-all}}a{{color:#1d4ed8}}
</style><h1>M20 盲走 checkpoint 爬坡实测</h1>
<p>2026-11-20 · 共 {report['trial_count']} 次仿真 · MuJoCo {html.escape(report['simulator_version'])}。
这是固定仿真条件下的测量，尚未在原 Isaac/PhysX 训练环境和实体机器人上复核。</p>
<h2>0.4 m/s 的临界坡度</h2><table>{head}{''.join(table_row(s) for s in selected)}</table>
<p>表中“通过数”只指本轮初始姿态样本；10/10 不能当作野外可靠性保证。临界点外的未测角度没有保证。</p>
<img src="slope-results.png" alt="两个 checkpoint 的坡度通过比例及最大前进位置">
<h2>速度对比：各条件下已测试、全部通过的最高坡度</h2><table>{head}{''.join(table_row(s) for s in best)}</table>
<p>不同速度的最高通过角度只在各自采样网格内成立；主结果 0.4 m/s 采用 1° 网格、每点 10 组初始姿态。</p>
<p>策略通过率可以随坡度非单调变化；某个角度全部通过，不等于它以下所有角度及全部初始条件都有保证。</p>
<h2>真实仿真状态回放</h2>{videos}<h2>测试方法</h2>{env}
<h2>摩擦与物理步长复核</h2><table><tr><th>迭代</th><th>速度</th><th>坡度</th><th>μ</th><th>物理步长 s</th><th>通过</th></tr>{check_table}</table>
<p>步长复核保持控制频率为 50 Hz；摩擦复核同时设置机器人与地形的接触系数。</p>
<p>坡高计算 h=2 tan(θ)。该数值是整段坡的高差，不能当作能直接跨越的台阶高度，也不表示最大可爬升海拔。</p>
<p>原始记录与元数据：<a href="report.json">report.json</a>；<a href="actor_validation.json">与 PyTorch 的推理一致性</a>；
<a href="evaluator_source.py">本轮测试脚本快照</a>。JSON 中每次试验均链接其 NPZ 轨迹，包含时间、机身/轮子位置、关节位置和动作。</p>
</html>'''
    (folder/'index.html').write_text(document)
    print(json.dumps(dict(trials=report['trial_count'], primary=selected, speed_comparison=best),ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
