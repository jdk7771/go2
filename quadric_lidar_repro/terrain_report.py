"""Source-backed local report for end-to-end official terrain tests."""

import csv
from html import escape
from pathlib import Path
import shutil

import numpy as np

LABELS={"hill":"Hill","forest":"Forest","quick":"Quick","desert":"Desert", "mountain":"Mountain","snow":"Snow","vocano":"Vocano (upstream spelling)"}
COLORS={"Quadric":"#24749e","Quadric-general":"#47906f","Local-quadratic":"#d38837","FSGP125":"#8169a7","FSGP500":"#b36481"}


def number(value,digits=3):
    return "N/A" if value is None else f"{value:.{digits}f}"


def create_report(output,protocol,metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output=Path(output);summary=metrics["summary"]
    scenes=[s["scene"] for s in protocol["scenes"]]
    lookup={(r["scene"],r["method"]):r for r in summary}
    with (output/"per_frame.csv").open() as stream:frames=list(csv.DictReader(stream))
    methods=["Quadric","Local-quadratic","FSGP125","FSGP500"]
    fig,axes=plt.subplots(1,2,figsize=(14,4.8),constrained_layout=True)
    x=np.arange(len(scenes))
    for i,method in enumerate(methods):
        axes[0].bar(x+(i-1.5)*.19,[lookup[s,method]["median_runtime_ms"] for s in scenes],
                    width=.18,color=COLORS[method],label=method)
    bottom=np.zeros(len(scenes))
    for key,label,color in [("normal_knn_ms","KNN + PCA normals","#758c9e"),
                            ("region_growing_ms","Geometry segmentation","#93b6ca"),
                            ("fitting_ms","Quadric fitting","#3884a7"),("evaluation_ms","Grid geometry","#b3d8bf")]:
        values=np.array([lookup[s,"Quadric"][key] for s in scenes])
        axes[1].bar(x,values,bottom=bottom,color=color,label=label);bottom+=values
    axes[0].set_title("XYZ input to 625 grid outputs; median time")
    axes[1].set_title("Analytic branch: segmentation is included")
    for ax in axes:
        ax.set_xticks(x,[LABELS[s] for s in scenes],rotation=20,ha="right",fontsize=8)
        ax.set_ylabel("CPU time / ms");ax.grid(axis="y",alpha=.2);ax.legend(fontsize=8)
    fig.savefig(output/"timing.png",dpi=165);plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(14,4.8),constrained_layout=True)
    for i,method in enumerate(methods):
        for ax,key,factor in [(axes[0],"height_mae_m",100),(axes[1],"slope_mae_deg",1)]:
            values=[lookup[s,method][key] for s in scenes]
            ax.bar(x+(i-1.5)*.19,[np.nan if v is None else factor*v for v in values],width=.18,color=COLORS[method],label=method)
    axes[0].set(ylabel="Height MAE / cm",title="Against dense clean-cloud geometric reference")
    axes[1].set(ylabel="Slope angle MAE / degree",title="Same currently supported grid cells")
    for ax in axes:
        ax.set_xticks(x,[LABELS[s] for s in scenes],rotation=20,ha="right",fontsize=8)
        ax.grid(axis="y",alpha=.2);ax.legend(fontsize=8)
    fig.savefig(output/"errors.png",dpi=165);plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(14,4.8),constrained_layout=True)
    for i,method in enumerate(["Quadric","Quadric-general","Local-quadratic"]):
        axes[0].bar(x+(i-1)*.24,[100*lookup[s,method]["mean_coverage"] for s in scenes],width=.23,color=COLORS[method],label=method)
    plane=[];quadric=[];distribution=[]
    for scene in scenes:
        selected=[r for r in frames if r["scene"]==scene and r["method"]=="Quadric"]
        for out,key in [(plane,"plane_patches"),(quadric,"quadric_patches"),(distribution,"distribution_patches")]:
            out.append(float(np.mean([float(r[key]) for r in selected])))
    axes[1].bar(x,plane,label="Plane",color="#8baabd")
    axes[1].bar(x,quadric,bottom=plane,label="General quadric",color="#d69a45")
    axes[1].bar(x,distribution,bottom=np.array(plane)+quadric,label="Distribution",color="#966c9c")
    axes[0].set(ylabel="Valid grid cells / %",ylim=(0,105),title="Missing and failed fits remain unknown")
    axes[1].set(ylabel="Mean patch count",title="Default plane gate retained")
    for ax in axes:
        ax.set_xticks(x,[LABELS[s] for s in scenes],rotation=20,ha="right",fontsize=8)
        ax.grid(axis="y",alpha=.2);ax.legend(fontsize=8)
    fig.savefig(output/"coverage.png",dpi=165);plt.close(fig)

    scene_images=[]
    for info in protocol["scenes"]:
        scene=info["scene"];frame=protocol["frames"]-1
        with np.load(output/"frames"/f"{scene}_{frame:03}.npz") as data:
            points=data["scan"];labels=data["labels"];xy=data["xy"];shape=tuple(data["shape"])
            extent=[xy[:,0].min()-.1,xy[:,0].max()+.1,xy[:,1].min()-.1,xy[:,1].max()+.1]
            fig=plt.figure(figsize=(15,8),constrained_layout=True)
            raw=Path(info["source"]["local_path"]).read_bytes();end=raw.index(b"DATA binary\n")+len(b"DATA binary\n")
            fields=next(line.split()[1:] for line in raw[:end].decode().splitlines() if line.startswith("FIELDS"))
            full=np.frombuffer(raw[end:],dtype="<f4").reshape(-1,len(fields))[:,:3]
            keep=np.linspace(0,len(full)-1,min(7000,len(full)),dtype=int)
            ax=fig.add_subplot(2,4,1,projection="3d")
            ax.scatter(*full[keep].T,c=full[keep,2],cmap="terrain",s=1)
            ax.set_title(f"Official {scene}.pcd: {len(full)} points",fontsize=9)
            ax.set(xlabel="x / m",ylabel="y / m",zlabel="z / m");ax.view_init(elev=35,azim=-120)
            ax=fig.add_subplot(2,4,2,projection="3d")
            colors=plt.cm.tab20((labels%20)/19);colors[labels<0]=[.7,.7,.7,1]
            ax.scatter(*points.T,c=colors,s=2)
            ax.set_title(f"Observed {len(points)} XYZ points; automatic patches",fontsize=9)
            ax.set(xlabel="x / m",ylabel="y / m",zlabel="z / m");ax.view_init(elev=35,azim=-120)
            ref=np.where(data["reference_valid"],data["reference_height"],np.nan)
            q=np.where(data["Quadric_valid"],data["Quadric_height"],np.nan)
            gp=np.where(data["FSGP500_valid"],data["FSGP500_height"],np.nan)
            finite=np.concatenate([v[np.isfinite(v)] for v in [ref,q,gp]])
            lo,hi=np.quantile(finite,[.01,.99])
            maps=[(3,ref,"Clean geometric height reference","terrain",lo,hi,"height / m"),
                  (4,q,"Segmentation + quadric height","terrain",lo,hi,"height / m"),
                  (5,gp,"FSGP M=500 height (CPU reference)","terrain",lo,hi,"height / m")]
            common=data["common"]
            qe=np.where(common,100*(q-ref),np.nan);ge=np.where(common,100*(gp-ref),np.nan)
            errors=np.concatenate([v[np.isfinite(v)] for v in [qe,ge]])
            maximum=max(float(np.quantile(np.abs(errors),.98)) if len(errors) else 1,1)
            maps.extend([(6,qe,"Quadric height error","coolwarm",-maximum,maximum,"error / cm"),
                         (7,ge,"FSGP M=500 height error","coolwarm",-maximum,maximum,"error / cm")])
            curvature=np.where(data["Quadric_valid"],np.max(np.abs(data["Quadric_principal"]),axis=1),np.nan)
            finite=curvature[np.isfinite(curvature)]
            upper=max(float(np.quantile(finite,.95)) if len(finite) else .1,.1)
            maps.append((8,curvature,"Analytic max |k|; white = unknown","viridis",0,upper,"curvature / m^-1"))
            for index,array,title,cmap,vmin,vmax,label in maps:
                ax=fig.add_subplot(2,4,index)
                image=ax.imshow(array.reshape(shape),origin="lower",extent=extent,cmap=cmap,vmin=vmin,vmax=vmax)
                ax.set_title(title,fontsize=9);ax.set(xlabel="x / m",ylabel="y / m")
                fig.colorbar(image,ax=ax,shrink=.7,label=label)
            fig.suptitle(f"{LABELS[scene]} | frame {frame+1}/{protocol['frames']} | 15% random dropout, 15mm added z-noise",fontsize=12)
            name=scene+".png";fig.savefig(output/name,dpi=155);plt.close(fig);scene_images.append(name)

    lines=["# 作者公开地形点云：从分割开始的完整拟合测试","",
           "已确认论文为高飞参与的 FSGP-BGK（IROS 2025）。本实验使用官方仓库原始点云，未用理想球面替代地形。","",
           "主要结果：计入法向和分割后，本地解析方案在七个场景都慢于局部高度二次拟合，与FSGP125 CPU参考接近。",
           "它相对几何参考的曲率差异也普遍更大，先前理想面片上的速度与精度优势没有延续到这条完整地形流水线。","",
           "解析分支完整流程：输入XYZ → 裁剪局部窗口 → KNN/PCA法向 → 空间与法向约束的自动分割 → Taubin拟合 → 栅格高度/法向/坡度/主曲率。",
           "FSGP原方法直接提取局部特征并做GP回归，不需要先做这种二次曲面面片分割。","",
           "## 数据范围与计时口径","",
           f"上游固定提交：{protocol['scenes'][0]['source']['url'].split('/')[5]}。七份原始点云均检查Git blob SHA1、文件大小和SHA256。",
           "仓库包含丘陵、森林以及quick、沙漠、山地、雪地、火山示例。这些示例不等于论文完整2500组评测；",
           "公路、室内和废墟原始批测集与通行代价标签没有包含在该仓库发布中，本次没有伪造这些缺失实验。","",
           f"每个场景{protocol['frames']}个移动窗口；5×5m地图、0.2m栅格，共625个查询位置。局部观测先以0.06m保留代表点，最多2500点，",
           "然后随机丢弃15%、增加15mm高程噪声。后者是沿用的本地诊断噪声，不是论文已声明参数；随机丢点不是射线遮挡仿真。",
           f"单线程CPU，每种方法预热1次后交错重复{protocol['repeats']}次计时；包含输入处理、法向、分割、拟合和地图计算，不含PCD读盘、参考生成与绘图。",
           "FSGP125/500为本地CPU公式参考，另含不确定性与代价输出；没有运行作者GPU程序，也没有在此实验加入BGK时序融合。","",
           "局部二次对照使用64个3D近邻，直接拟合重力对齐的高度函数z=f(x,y)，不预分割；",
           "它与前一轮任意切平面坐标下的局部多项式基线实现不同，本轮所有方法均固定625个地图查询。","",
           "|场景|原始点数|局部输入平均点数|原文件法向参考|",
           "|---|---:|---:|---|"]
    for info in protocol["scenes"]:
        lines.append(f"|{info['scene']}|{info['raw_points']}|{lookup[info['scene'],'Quadric']['mean_scan_points']:.0f}|{info['reference_normals']}|")
    lines += ["","## 从XYZ开始的速度","",
              "所有方法读取完全相同的局部观测并查询同样625个栅格。统计是逐帧中位数再取中位数；各阶段不含参考计算。","",
              "|场景|分割+拟合+几何(ms)|局部二次拟合(ms)|FSGP125(ms)|FSGP500(ms)|KNN法向(ms)|区域分割(ms)|拟合(ms)|几何地图(ms)|",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for scene in scenes:
        q=lookup[scene,"Quadric"]
        values=[lookup[scene,m]["median_runtime_ms"] for m in methods]+[q[k] for k in ["normal_knn_ms","region_growing_ms","fitting_ms","evaluation_ms"]]
        lines.append("|"+scene+"|"+"|".join(number(v) for v in values)+"|")
    lines += ["","![完整耗时和分阶段耗时](timing.png)","","## 效果与覆盖率","",
              "参照来自完整干净点云的局部几何：六个文件附带单位法向，森林文件没有法向，使用完整干净点云估计PCA法向。",
              "高度由近邻切平面给出，几何曲率由干净法向变化局部拟合得到。因此这里是几何参考差异，不能称为论文代价真值误差。",
              "主对照在参考与四种方法共同有效的当前栅格上评估，丢失/拟合失败位置保留未知；每种方法的完整覆盖率另列。",
              "FSGP的无量纲PCA曲率指标没有与单位m⁻¹的主曲率直接比较。","",
              "|场景|方法|高度MAE(cm)|坡度MAE(°)|几何曲率参考RMSE(m⁻¹)|有效栅格比例|共同评估格子·帧|",
              "|---|---|---:|---:|---:|---:|---:|"]
    for r in summary:
        if r["method"]=="Quadric-general":continue
        lines.append(f"|{r['scene']}|{r['method']}|{number(None if r['height_mae_m'] is None else 100*r['height_mae_m'])}|"
                     f"{number(r['slope_mae_deg'])}|{number(r['curvature_rmse_per_m'])}|{r['mean_coverage']:.1%}|{r['evaluation_cell_frames']}|")
    lines += ["","![高度与坡度参考差异](errors.png)","","![覆盖率和面片类型](coverage.png)","",
              "默认平面阈值会让部分缓弯面输出零曲率。Quadric-general为关闭平面门限的单独敏感性对照，",
              "其原始误差、失败比例和逐次耗时保留在CSV/JSON中；它采用自己的参考交集，不能与主表直接作精度排名。","",
              "## 完整原始PCD输入：不设点数上限","",
              "下面从原始PCD的全部XYZ在内存中开始，计时内先裁剪局部窗口；不做0.06m稀疏化、2500点上限、随机丢点或额外噪声。",
              "局部二次方法和FSGP也包含各自所需的在线裁剪和建树。输入全图点数与实际局部点数分开列出。","",
              "|场景|输入全图点数|局部实际点数|完整解析分支(ms)|局部二次(ms)|FSGP125(ms)|",
              "|---|---:|---:|---:|---:|---:|"]
    for r in metrics["dense_input_latencies"]:
        lines.append(f"|{r['scene']}|{r['input_points']}|{r['local_points']}|"+"|".join(number(r['methods'][m]['median_ms']) for m in methods[:3])+"|")
    lines += ["","## 场景效果","",
              "每幅图先显示原始地图与带噪观测的自动分割，再显示相同窗口的地图结果。灰色点没有形成有效面片，白格为未知。",
              "高度色标使用1–99百分位、误差色标使用98百分位、曲率色标使用95百分位；超出色标的数值截断，CSV/NPZ保留完整数值。"]
    for name in scene_images:lines += ["",f"![{name[:-4]}场景]("+name+")"]
    lines += ["","## 复查与来源","",
              "17项单元测试覆盖曲率闭式公式、有限差分、刚体变换、上下层分割、根选择与完整平面流水线。",
              "每帧NPZ包含原始局部输入、源点索引、分割标签、参考与各方法预测，可重新计算误差；protocol.json记录全部参数与源码哈希。",
              "本次是二次曲面方案在高飞团队公开地形上的独立测试，不能称为论文原版GPU性能复现或完整2500场景复现。","",
              "论文：https://arxiv.org/html/2503.04134v2 。官方仓库：https://github.com/ZJU-FAST-Lab/FSGP_BGK 。",""]
    (output/"REPORT.md").write_text("\n".join(lines))
    body=[];table=[]
    def flush():
        if not table:return
        body.append('<div class="scroll"><table>')
        for i,line in enumerate(table):
            if i==1:continue
            tag="th" if i==0 else "td"
            body.append("<tr>"+"".join(f"<{tag}>"+escape(cell)+f"</{tag}>" for cell in line.strip("|").split("|"))+"</tr>")
        body.append("</table></div>");table.clear()
    for line in lines:
        if line.startswith("|"):table.append(line);continue
        flush()
        if line.startswith("# "):body.append("<h1>"+escape(line[2:])+"</h1>")
        elif line.startswith("## "):body.append("<h2>"+escape(line[3:])+"</h2>")
        elif line.startswith("!["):
            alt,name=line[2:].split("](",1);name=name[:-1]
            body.append(f'<a href="{escape(name)}"><img src="{escape(name)}" alt="{escape(alt)}" loading="lazy"></a>')
        elif line:body.append("<p>"+escape(line)+"</p>")
    flush()
    nav='<nav><a href="summary.csv">汇总CSV</a><a href="per_frame.csv">逐帧CSV</a><a href="metrics.json">逐次耗时JSON</a><a href="protocol.json">数据与协议</a><a href="REPORT.md">报告</a></nav>'
    html='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>官方地形：从分割开始的曲面拟合测试</title><style>html{overflow-wrap:anywhere}body{margin:0;background:#f2f5f7;color:#243445;font:15px/1.75 system-ui,sans-serif}main{box-sizing:border-box;width:100%;max-width:1380px;margin:auto;padding:30px;background:white}h1{font-size:28px}h2{margin-top:36px;border-top:1px solid #dde4e9;padding-top:20px}p{margin:8px 0}nav{display:flex;flex-wrap:wrap;gap:18px}a{color:#146d98}img{width:100%;height:auto;margin:20px 0}.scroll{max-width:100%;overflow:auto;margin:20px 0}table{border-collapse:collapse;white-space:nowrap;font-size:13px;width:100%}th,td{padding:10px;border-bottom:1px solid #dce3e8;text-align:right}th{background:#edf3f8}td:first-child,th:first-child{text-align:left}@media(max-width:600px){main{padding:16px}h1{font-size:23px}}</style><main>'+nav+"".join(body)+'</main></html>'
    (output/"index.html").write_text(html)
    viewer=ROOT_VIEWER()/"quadric-terrain";viewer.mkdir(parents=True,exist_ok=True)
    for name in ["index.html","REPORT.md","protocol.json","metrics.json","summary.csv","per_frame.csv","timing.png","errors.png","coverage.png",*scene_images]:
        shutil.copy2(output/name,viewer/name)


def ROOT_VIEWER():
    return Path(__file__).resolve().parents[1]/"fsgp_bgk_repro/outputs/visualization"
