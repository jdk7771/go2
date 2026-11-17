"""Publish compute and browser timings, with their distinct measurement scopes."""
import html
import json
import re
from .benchmark_assessment_timing import TIMING, STAGES, VIEW_NAMES, VIEW_STAGES

STAGE_NAMES=dict(select_centers_ms='选中心',window_query_ms='建树／筛点',fit_and_covariance_ms='主拟合＋协方差',
    assembly_ms='组合',display_patch_geometry_ms='面片几何',three_jump_methods_ms='A/B/C',
    clearance_ms='净空',local_costs_ms='局部评分',body_50x80_ms='机身评估')


def table(headers,rows):
    return '| '+' | '.join(headers)+' |\n|'+ '|'.join(['---']+['---:']*(len(headers)-1))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)+'\n'


def main():
    cpu=json.loads((TIMING/'compute.json').read_text())
    browser=json.loads((TIMING/'browser.json').read_text()) if (TIMING/'browser.json').exists() else None
    if browser:
        assert {c['id'] for c in cpu['scenes']}=={c['id'] for c in browser['scenes']}, 'Compute/browser scene scopes differ or a benchmark is incomplete'
    view_rows=[]
    text='''# 17 场景与每个可视化窗口的耗时记录

实验日期：2026-11-17（Asia/Shanghai）。所有场景仍在 http://127.0.0.1:42482/。

这是**整幅场景点云**的时间，不是一个 20 cm 面片的时间，也不是摄像机图像帧率。每场景按相同 20×20 cm 拟合和 50×80 cm 机身参数顺序重复 3 次；记录中位数及观测到的最小／最大值。原始 XYZ 与发布文件哈希一致，计算结果逐元素与发布模型核对一致；没有更改几何结果来获得速度。

## 1. 整个场景从 XYZ 到全部评估

单位为秒。算法总时间包括选中心、搜索、主拟合和协方差、组合、全部面片显示几何、A/B/C 跳变、净空、落足与机身评估。点云读取／解析场景重新生成时间单列；它会受到操作系统缓存影响，没有清除文件缓存。

“计算调用墙钟”还包括输入类型转换和最终输出数组打包；不包括点云准备与结果写文件。算法阶段计时也不包含文件导出、网络和浏览器绘制。

'''
    environment=cpu['environment']
    text+=f"计算环境：{environment['cpu']}，Python {environment['python']}，NumPy {environment['numpy']}，BLAS 单线程。\n\n"
    rows=[]
    for c in cpu['scenes']:
        s=c['summary'];t=s['total_assessment_ms']
        rows.append([c['name'],f"{c['counts']['raw_points']:,}",f"{c['counts']['valid']:,}",
            f"{t['median_ms']/1000:.3f}",f"{t['min_ms']/1000:.3f}–{t['max_ms']/1000:.3f}",
            f"{s['source_prepare_ms']['median_ms']/1000:.3f}",f"{s['compute_call_wall_ms']['median_ms']/1000:.3f}"])
    text+=table(['场景','原始点','有效函数','算法总计 s','最短–最长 s','点云准备 s','计算调用墙钟 s'],rows)
    text+='''quick 没有有效函数，沙漠没有可用机身综合结果；这些场景耗时少，不能解释为已经快速完成了有效稳定性评估。各场景的实际求解数、有效数和双侧额外求解数见 compute-summary.csv。

## 2. 每个算法阶段的实测时间

单位为毫秒，每格为同一场景 3 次中位数；各阶段分别取中位数，相加不一定等于总时间中位数。误差与协方差随主拟合产生，面片内最大 σ 随面片几何产生；A/B/C 在同一个阶段中共同计算。

'''
    text+=table(['场景',*[STAGE_NAMES[k]+' ms' for k in STAGES]],[[c['name'],*[f"{c['summary'][k]['median_ms']:.1f}" for k in STAGES]] for c in cpu['scenes']])
    text+='''## 3. 每张图从 XYZ 开始，需要哪些计算

下表单位为秒，是**同次运行内该图所依赖的实测阶段之和，再取 3 次中位数**。这是依赖阶段的合计估算，不是将 8 个独立程序从头运行得到的墙钟测量。不含点云准备、浏览器绘制和结果写文件。共享阶段重复出现在不同列，不能将这 8 列相加。

当前 Python 入口一次计算完整套结果。下表用于分析共享依赖；页面展开一个窗口时只读取和绘制已有结果。

原始点云图不需要拟合，因此其算法项为 0；它仍然需要读数据与绘制。独立面与误差图共享拟合和面片几何；净空、跳变、落足和机身图还依赖各自指标。组合面使用拟合中心之间的三角网，独立曲面显示几何不是其必要依赖。

'''
    for c in cpu['scenes']:
        row=[c['name']]
        for key,stages in VIEW_STAGES.items():
            values=sorted(sum(run[k] for k in stages) for run in c['runs'])
            n=len(values);m=values[n//2] if n%2 else (values[n//2-1]+values[n//2])/2
            row.append(f'{m/1000:.3f}')
        view_rows.append(row)
    text+=table(['场景',*[VIEW_NAMES[k]+' s' for k in VIEW_STAGES]],view_rows)
    text+='''## 4. 浏览器每张图的加载与出图

网页展示的是预先算好的文件，切换 A/B/C 或颜色不会在后台重新拟合。下面这组时间与前面的算法时间分开。

测量模式仅把初始窗口全部收起，页面代码与数据保持不变；每次只打开一个窗口。浏览器 HTTP 缓存禁用，操作系统文件缓存未清理。每场景 3 次新的页面导航。

- **读入全部结果数据**：从导航开始，到全部场景数据读入、解码和页面指标准备完成，尚未创建三维窗口。不是默认三个窗口同时打开时的首页总加载时间。
- **首次打开窗口**：从展开窗口开始，到两次动画帧后执行 gl.finish 完成队列中的绘制；包括 WebGL 上下文初始化、几何和颜色、布局、调度、实际渲染。不是屏幕扫描出像素的物理延迟。
- **重建并绘制**：已创建窗口后，重建其几何／颜色并执行绘制、gl.finish；每次导航取 3 样本，共 9 样本。包括 JS 与渲染，不含网络、Python 算法和 PNG 编码。

首次开图包含帧调度等开销，不能用 1000/首次开图毫秒换算为持续帧率。机身窗口比其他窗口更宽，CSV 记录了实际画布尺寸。

'''
    if browser:
        environment=browser['environment'];renderer='；'.join(environment['webgl_renderers'])
        text+=f"环境：无头 Chrome {environment['chrome_version']}，页面 1440×1000，设备缩放 1；WebGL：`{renderer}`。\n\n"
        text+='**这里若显示 SwiftShader，表示软件渲染，不代表桌面浏览器的硬件 GPU 速度；实际交互浏览器的时间可能不同。**\n\n'
        text+='首次打开各窗口，中位数，单位毫秒：\n\n'
        text+=table(['场景','读入全部数据 ms',*[VIEW_NAMES[k]+' ms' for k in VIEW_STAGES]],
            [[c['name'],f"{c['load_without_3d']['median_ms']:.1f}",*[f"{c['views'][k]['first_open_gpu_complete']['median_ms']:.1f}" for k in VIEW_STAGES]] for c in browser['scenes']])
        text+='重建几何／颜色并绘制，中位数，单位毫秒：\n\n'
        text+=table(['场景',*[VIEW_NAMES[k]+' ms' for k in VIEW_STAGES]],
            [[c['name'],*[f"{c['views'][k]['rebuild_gpu_complete']['median_ms']:.1f}" for k in VIEW_STAGES]] for c in browser['scenes']])
        assert browser['passed']
    else:text+='浏览器计时尚未生成。\n\n'
    text+='''## 5. 原始记录与重复命令

- [计算逐次记录 CSV](compute-runs.csv)、[逐场景计算汇总 CSV](compute-summary.csv)、[完整计算 JSON](compute.json)。
- [逐图计算依赖 CSV](per-view-compute.csv)；其中明示 measurement_kind 为依赖阶段求和。
- [浏览器逐次 CSV](browser-runs.csv)、[浏览器逐图汇总 CSV](browser-summary.csv)、[完整浏览器 JSON](browser.json)。

```bash
cd /home/jiang/snap/go2
# 依次运行，避免 Python 测速与浏览器软件渲染竞争 CPU。
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.benchmark_assessment_timing --repeats 3
node fsgp_bgk_repro/benchmark_assessment_browser.cjs --repeats 3
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -m fsgp_bgk_repro.publish_assessment_timing
```

更改测试范围可以使用 Python 的 `--scenes hill table`，浏览器的 `--scenes hill,table`。两者会发布本轮选择的测试范围；默认是全部 17 场景，不会覆盖原有模型与场景数值文件。
'''
    (TIMING/'report.md').write_text(text)
    # The in-app dashboard presents the same records; no extra server/port.
    headers=[];body=[];in_code=False
    def inline(text):
        escaped=html.escape(text)
        escaped=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'<a href="\2">\1</a>',escaped)
        escaped=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',escaped)
        escaped=re.sub(r'`([^`]+)`',r'<code>\1</code>',escaped)
        return escaped
    for line in text.splitlines():
        if line.startswith('```'):
            if in_code:body.append('</code></pre></details>')
            else:body.append('<details><summary>复现指令</summary><pre><code>')
            in_code=not in_code;continue
        if in_code:body.append(html.escape(line)+'\n');continue
        if line.startswith('|'):
            cells=[cell.strip() for cell in line.strip().strip('|').split('|')]
            if all(cell.replace(':','').replace('-','')=='' for cell in cells):continue
            if not headers:
                headers=cells;body.append('<div class="scroll"><table><thead><tr>'+''.join('<th>'+html.escape(c)+'</th>' for c in cells)+'</tr></thead><tbody>')
            else:body.append('<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>')
        else:
            if headers:body.append('</tbody></table></div>');headers=[]
            if not line:continue
            if line.startswith('# '):body.append('<h1>'+html.escape(line[2:])+'</h1>')
            elif line.startswith('## '):body.append('<h2>'+html.escape(line[3:])+'</h2>')
            else:body.append('<p>'+inline(line)+'</p>')
    page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>逐场景与逐图耗时 · 42482</title><link rel="icon" href="data:,"><style>body{font:15px/1.75 system-ui,sans-serif;color:#243e33;background:#f1f5f2;margin:0}main{max-width:1500px;margin:auto;padding:24px}h1{font-size:28px}h2{margin-top:32px;font-size:21px}a{color:#196b4f}.scroll{overflow-x:auto;background:white;border:1px solid #d6e0d9;border-radius:9px;margin:16px 0}table{border-collapse:collapse;white-space:nowrap;font-size:13px;width:100%}td,th{border-bottom:1px solid #dce5df;padding:9px 12px;text-align:right}td:first-child,th:first-child{text-align:left;position:sticky;left:0;background:white}th{background:#e4eee7}nav{display:flex;flex-wrap:wrap;gap:15px}@media(max-width:700px){main{padding:16px 12px}h1{font-size:23px}}</style></head><body><main><nav><a href="../../">全部场景</a><a href="../">20 cm 评估</a><a href="report.md">完整 Markdown 记录</a><a href="compute-summary.csv" download>计算汇总 CSV</a><a href="per-view-compute.csv" download>逐图计算 CSV</a><a href="browser-summary.csv" download>逐图浏览器 CSV</a></nav>'''+''.join(body)+'</main></body></html>'
    page=page.replace('</style>','pre{overflow:auto;background:#e7eee9;border-radius:8px;padding:12px}p{overflow-wrap:anywhere}</style>')
    (TIMING/'index.html').write_text(page)
    short='''## 11. 新增：17 场景、逐图计时记录

最新的逐场景 3 次计算、每图依赖阶段、浏览器逐窗口测量，见 [同端口耗时页面](http://127.0.0.1:42482/assessment/timing/) 与 [完整计时说明](fsgp_bgk_repro/outputs/visualization/assessment/timing/report.md)。算法和浏览器计时分别记录，不相加为独立每图的算法工作量。

下表为整幅场景点云的算法总耗时，单位秒。包括全部评估与显示几何，不含点云准备、文件导出和浏览器绘制。quick 等无有效拟合的快速返回不代表有效评估完成。

'''
    short+=table(['场景','3 次中位数 s','最短–最长 s'],[[c['name'],f"{c['summary']['total_assessment_ms']['median_ms']/1000:.3f}",f"{c['summary']['total_assessment_ms']['min_ms']/1000:.3f}–{c['summary']['total_assessment_ms']['max_ms']/1000:.3f}"] for c in cpu['scenes']])
    short+='逐窗口出图数据明确记录实际 WebGL 设备；本次无头 Chrome 的 SwiftShader 是软件渲染，不能当作桌面硬件 GPU 性能。原始逐次记录与依赖计算口径均保存在上述链接中。\n'
    (TIMING/'step-section.md').write_text(short)
    print('Published',len(cpu['scenes']),'compute scenes,',len(browser['scenes']) if browser else 0,'browser scenes')


if __name__=='__main__':main()
