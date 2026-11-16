"""Paper feature sampling -> FSGP surface, with explicit quadratic controls.

The FSGP paper does not segment geometric surface patches or fit quadrics.
This diagnostic follows its feature/non-feature partition and sampling; both
quadratic regressions are separately labelled local controls on the SAME M
retained observations. Existing homepage scans/models are reconstructed exactly.
"""

import argparse
from dataclasses import replace
import csv
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import cKDTree

from .core import Config, SparseTerrainGP, TraversabilityMapper, extract_features, interpolate_features
from .experiment import sample_scan

ROOT = Path(__file__).resolve().parent
NAMES = {"hill":"丘陵", "flat":"平地", "ramp":"斜坡", "steps":"台阶", "quick":"Quick 地形",
         "table":"大桌子", "flat_ramp":"平地—斜坡—高处平地"}
COUNTS = (50, 125, 500)
SIZE = 101


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean_json(value):
    if isinstance(value, np.ndarray):
        return clean_json(value.tolist())
    if isinstance(value, (np.floating, float)):
        return round(float(value), 7) if np.isfinite(value) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {key:clean_json(v) for key,v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean_json(v) for v in value]
    return value


def truth_height(xy):
    return .3*np.clip(np.asarray(xy)[:, 0]+1., 0., 2.)


def quadratic_fit(points):
    origin = points[:, :2].mean(axis=0)
    scale = max(float(np.sqrt(np.mean(np.sum((points[:, :2]-origin)**2, axis=1)))), 1e-8)
    u,v = ((points[:, :2]-origin)/scale).T
    design = np.column_stack((u*u, u*v, v*v, u, v, np.ones(len(u))))
    coefficient, _, rank, _ = np.linalg.lstsq(design, points[:, 2], rcond=None)
    physical = coefficient/np.array([scale**2]*3+[scale]*2+[1.])
    return {"origin":origin, "coefficients":physical, "rank":int(rank)}


def quadratic_predict(model, xy):
    u,v = (xy-model["origin"]).T
    return np.column_stack((u*u, u*v, v*v, u, v, np.ones(len(u))))@model["coefficients"]


def local_quadratic(points, xy, neighbors=20):
    """Height graph least squares on K retained 3D neighbours, no patch labels.

    Query Z is anchored by the nearest retained XY observation. Degenerate XY
    designs stay unknown; the method cannot represent a vertical surface.
    """
    _, nearest = cKDTree(points[:, :2]).query(xy)
    anchor = points[nearest, 2]
    _, ids = cKDTree(points).query(np.column_stack((xy, anchor)), k=min(neighbors,len(points)))
    local = points[ids]
    u,v = (local[:, :, :2]-xy[:, None, :]).transpose(2,0,1)
    scale = np.maximum(np.sqrt(np.mean(u*u+v*v, axis=1)), 1e-10)
    u,v = u/scale[:,None], v/scale[:,None]
    design = np.stack((np.ones_like(u), u,v,u*u,u*v,v*v), axis=2)
    gram = np.einsum("nki,nkj->nij",design,design)
    spectrum = np.linalg.eigvalsh(gram)
    valid = spectrum[:,0] > np.maximum(spectrum[:,-1],1e-30)*1e-10
    rhs = np.einsum("nki,nk->ni",design,local[:,:,2]-anchor[:,None])
    ridge = 1e-10*np.trace(gram,axis1=1,axis2=2)/6
    coefficient = np.linalg.solve(gram+ridge[:,None,None]*np.eye(6),rhs)
    height = anchor+coefficient[:,0]
    valid &= np.isfinite(height)
    return np.where(valid,height,np.nan), valid


def feature_partition(features, config):
    mask = (features[:,3]>config.curvature_threshold) | (features[:,4]>config.gradient_threshold)
    non_feature = np.flatnonzero(~mask)
    _, representatives = np.unique(np.floor(features[non_feature,:3]/config.voxel_size).astype(np.int64),axis=0,return_index=True)
    representatives = non_feature[representatives]
    candidates = np.concatenate((np.flatnonzero(mask),representatives))
    return mask, representatives, candidates


def metrics(prediction, target, mask):
    keep = mask & np.isfinite(prediction) & np.isfinite(target)
    error = prediction[keep]-target[keep]
    return {"n":int(keep.sum()), "mae_m":float(np.abs(error).mean()),
            "rmse_m":float(np.sqrt(np.mean(error**2))), "p95_m":float(np.percentile(np.abs(error),95))} if len(error) else None


def export_case(scene, frame, scan, saved, fused, config, source):
    features = extract_features(scan, config.neighbors)
    feature, voxel_ids, candidates = feature_partition(features, config)
    inducing = saved["inducing"]
    distance, retained_ids = cKDTree(scan).query(inducing[:,:3])
    assert distance.max() < 1e-12
    assert np.isin(retained_ids, candidates).all(), "Retained point does not follow paper sampling"
    np.testing.assert_allclose(inducing[:,3:5],features[retained_ids,3:5],atol=1e-12,rtol=1e-10)
    gp = SparseTerrainGP(config).fit(inducing[:,[0,1,3,4]],inducing[:,2])
    test, _ = interpolate_features(features,saved["xy"],config.interpolation_neighbors)
    coarse_height, coarse_variance, _ = gp.predict(test)
    reconstruction_error = float(np.max(np.abs(coarse_height-saved["height"])))
    variance_error = float(np.max(np.abs(coarse_variance-saved["variance"])))
    assert reconstruction_error < 1e-8 and variance_error < 1e-8
    axis_x = np.linspace(saved["xy"][:,0].min(),saved["xy"][:,0].max(),SIZE)
    axis_y = np.linspace(saved["xy"][:,1].min(),saved["xy"][:,1].max(),SIZE)
    x,y = np.meshgrid(axis_x,axis_y)
    xy = np.column_stack((x.ravel(),y.ravel()))
    test, nearest_distance = interpolate_features(features,xy,config.interpolation_neighbors)
    height, variance, _ = gp.predict(test)
    supported = nearest_distance<=config.support_radius
    gp_valid = supported & (variance<=config.max_variance)
    model = quadratic_fit(inducing[:,:3])
    whole_height = quadratic_predict(model,xy)
    local_height, local_valid = local_quadratic(inducing[:,:3],xy,config.neighbors)
    local_valid &= supported
    common = gp_valid & local_valid & np.isfinite(whole_height)
    point_height, _, _ = gp.predict(features[:,[0,1,3,4]])
    point_global = quadratic_predict(model,scan[:,:2])
    point_local, point_local_valid = local_quadratic(inducing[:,:3],scan[:,:2],config.neighbors)
    point_mask = point_local_valid & np.isfinite(point_global)
    reconstruction = {name:metrics(value,scan[:,2],point_mask) for name,value in
                      [("gp",point_height),("local",point_local),("global",point_global)]}
    exact = truth_height(xy) if scene=="flat_ramp" else None
    analytic_errors = None
    if exact is not None:
        seams = np.minimum(np.abs(xy[:,0]+1),np.abs(xy[:,0]-1))<=.2
        regions = {"all":common,"seams":common&seams,"flat_low":common&(xy[:,0]<-1.2),
                   "ramp":common&(np.abs(xy[:,0])<.8),"flat_high":common&(xy[:,0]>1.2)}
        analytic_errors = {region:{name:metrics(value,exact,mask) for name,value in
                          [("gp",height),("local",local_height),("global",whole_height)]} for region,mask in regions.items()}
    return {"scene":scene,"label":NAMES[scene],"frame":frame,"count":len(inducing),
            "points":scan.ravel(), "feature_mask":feature.astype(int),"voxel_ids":voxel_ids,
            "inducing_ids":retained_ids,"inducing":inducing[:,:3].ravel(),
            "partition":{"input":len(scan),"feature":int(feature.sum()),"non_feature":int((~feature).sum()),
                         "voxel_representatives":len(voxel_ids),"candidates":len(candidates),"retained":len(inducing)},
            "x":axis_x,"y":axis_y,"gp_height":np.where(gp_valid,height,np.nan),
            "gp_sigma":np.sqrt(variance),"quadratic_height":np.where(supported,whole_height,np.nan),
            "local_height":np.where(local_valid,local_height,np.nan),"truth":exact,
            "quadratic":model,"reconstruction":reconstruction,"analytic_errors":analytic_errors,
            "cost_shape":saved["shape"],"coarse_xy":saved["xy"],
            "fsgp_cost":saved["cost"].ravel(),"bgk_cost":fused["cost"].ravel(),
            "bounds":[scan.min(axis=0),scan.max(axis=0)],"coverage":{"gp":float(gp_valid.mean()),
            "local":float(local_valid.mean()),"common":float(common.mean())},
            "source":source,"grid_reconstruction_max_error_m":reconstruction_error,
            "variance_reconstruction_max_error":variance_error}


def calibration():
    rng = np.random.default_rng(904)
    xy = rng.uniform([-4.,-3.],[6.,5.],(160,2))
    z = .13*(xy[:,0]-1)**2-.06*(xy[:,0]-1)*(xy[:,1]+.3)+.08*(xy[:,1]+.3)**2+.17*xy[:,0]-.11*xy[:,1]+.4
    points = np.column_stack((xy,z))
    model = quadratic_fit(points)
    error = float(np.max(np.abs(quadratic_predict(model,xy)-z)))
    local, valid = local_quadratic(points,xy)
    local_error = float(np.max(np.abs(local[valid]-z[valid])))
    assert error<1e-11 and local_error<1e-6 and valid.all()
    return {"global_quadratic_exact_recovery_max_m":error,"local_quadratic_exact_recovery_max_m":local_error}


def figures(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    font=FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family":font.get_name(),"axes.unicode_minus":False})
    points=np.asarray(data["points"]).reshape(-1,3)
    x,y=np.meshgrid(data["x"],data["y"])
    fig=plt.figure(figsize=(15,5.8),facecolor="#f5f8fc")
    axes=[fig.add_subplot(1,3,i+1,projection="3d") for i in range(3)]
    feature=np.asarray(data["feature_mask"],dtype=bool)
    axes[0].scatter(*points[~feature].T,c="#a1afbd",s=2,depthshade=False)
    axes[0].scatter(*points[feature].T,c="#d39235",s=3,depthshade=False)
    retained=np.asarray(data["inducing"]).reshape(-1,3)
    axes[0].scatter(*retained.T,c="#be403e",s=8,depthshade=False)
    for ax,field,color in [(axes[1],"gp_height","#3286d0"),(axes[2],"local_height","#dc9440")]:
        z=np.array([np.nan if v is None else v for v in data[field]]).reshape(SIZE,SIZE)
        ax.plot_surface(x,y,z,rstride=1,cstride=1,color=color,linewidth=0,antialiased=False,alpha=1.)
    values=np.concatenate([points[:,2]]+[np.array([np.nan if v is None else v for v in data[k]]) for k in ["gp_height","local_height"]])
    lo,hi=np.nanmin(values),np.nanmax(values)
    titles=["论文特征筛选与采样","论文 FSGP 高程回归（CPU 公式参考）","局部二次函数对照（相同 M 点）"]
    for ax,title in zip(axes,titles):
        ax.set_title(title,fontsize=11,pad=12); ax.set_xlabel("X (m)");ax.set_ylabel("Y (m)");ax.set_zlabel("Z (m)")
        ax.set_xlim(data["x"][0],data["x"][-1]);ax.set_ylim(data["y"][0],data["y"][-1]);ax.set_zlim(lo-.04,hi+.04)
        ax.set_box_aspect((np.ptp(data["x"]),np.ptp(data["y"]),max(hi-lo,.4)))
        ax.view_init(elev=28,azim=-63);ax.tick_params(labelsize=7)
    fig.suptitle(f"{NAMES[data['scene']]} · 第 {data['frame']+1} 帧 · M={data['count']} · 相同输入 / 相同视角",fontsize=14)
    fig.text(.5,.025,"橙点：特征点；灰点：非特征点；红点：最终保留点。二次函数为单独对照；本论文没有逐面片二次曲面分割。",ha="center",fontsize=9)
    fig.subplots_adjust(left=.015,right=.975,top=.85,bottom=.15,wspace=.10)
    fig.savefig(output/f"{data['scene']}-surfaces.png",dpi=170,facecolor=fig.get_facecolor(),bbox_inches="tight",pad_inches=.12);plt.close(fig)
    row=int(np.argmin(np.abs(np.asarray(data["y"])-.1)))
    y0=data["y"][row]
    band=np.abs(points[:,1]-y0)<=.1
    fig,ax=plt.subplots(figsize=(12,4.6),constrained_layout=True)
    ax.scatter(points[band,0],points[band,2],s=13,c="#687b8b",alpha=.65,label="输入点（剖面 ±0.10 m 带）")
    for key,color,label,style in [("gp_height","#247abb","FSGP 回归","-"),("local_height","#d48b29","局部二次对照","-"),("quadratic_height","#9778b9","全窗单个二次函数对照","--")]:
        z=np.array([np.nan if v is None else v for v in data[key]]).reshape(SIZE,SIZE)[row]
        ax.plot(data["x"],z,c=color,lw=2,label=label,ls=style)
    if data["truth"] is not None:
        ax.plot(data["x"],np.asarray(data["truth"]).reshape(SIZE,SIZE)[row],c="#23384b",lw=1.5,ls=":",label="组合地形解析真值")
        for edge in [-1.,1.]:ax.axvline(edge,color="#b8c3cc",ls=":",lw=1)
    ax.set(xlabel="X (m)",ylabel="Z (m)",title=f"{NAMES[data['scene']]} · y={y0:.3f} m 剖面 · M={data['count']}",xlim=(data["x"][0],data["x"][-1]))
    ax.grid(alpha=.18);ax.legend(fontsize=9,ncol=2,loc="best")
    fig.savefig(output/f"{data['scene']}-profile.png",dpi=170);plt.close(fig)


def home_entry(home):
    if not home.exists():return
    text=home.read_text()
    if 'id="fsgp-fit-entry"' in text:return
    banner=('<nav id="fsgp-fit-entry" style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:12px;'
            'margin-bottom:14px;padding:17px 20px;background:#e8f4ec;border:1px solid #8ebca1;border-radius:10px;font:14px system-ui;color:#28553a">'
            '<strong>FSGP 论文流程：特征筛选、GP 曲面与二次函数对照 · 新增平地—斜坡组合</strong>'
            '<a href="./fsgp-fit/?v=1" style="padding:9px 14px;background:#2c7551;color:white;border-radius:6px;text-decoration:none">打开拟合曲面 / 剖面曲线 →</a></nav>')
    home.write_text(text.replace("<body>","<body>\n"+banner,1))


def write_report(cases, settings, output):
    pooled=[]; rows=[]
    for case in cases:
        if case["scene"]!="flat_ramp":continue
        for region,methods in case["analytic_errors"].items():
            for method,error in methods.items():
                rows.append({"frame":case["frame"]+1,"count":case["count"],"region":region,"method":method,**error})
    for count in COUNTS:
        for region in ["all","seams","flat_low","ramp","flat_high"]:
            for method in ["gp","local","global"]:
                items=[r for r in rows if r["count"]==count and r["region"]==region and r["method"]==method]
                n=sum(r["n"] for r in items)
                pooled.append({"count":count,"region":region,"method":method,"n":n,
                               "mae_m":sum(r["n"]*r["mae_m"] for r in items)/n,
                               "rmse_m":np.sqrt(sum(r["n"]*r["rmse_m"]**2 for r in items)/n)})
    (output/"combined_summary.json").write_text(json.dumps(clean_json(pooled),indent=2))
    with (output/"combined_per_frame.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    labels={"gp":"FSGP CPU 公式参考","local":"局部二次函数对照","global":"全窗单个二次函数对照"}
    lines=["# FSGP 论文点云处理与拟合曲面/曲线", "",
      "原首页六个场景（平地、斜坡、台阶、丘陵、quick、桌子）全部接入，M=50/125/500，沿用第8帧原始观测与保留点。",
      "新增平地—斜坡—高处平地，3个M各8帧；总计42组。新旧场景均不进行逐面片几何分割。", "",
      "## 方法对应", "",
      "论文第III-A节：3D KNN=20计算κ=λmin/Σλ与平均绝对高差g；κ>0.03或g>0.03m为特征点。",
      "非特征点取0.1m体素代表点，和特征点合并，超出M时随机保留M点。页面显示各阶段点数。",
      "第III-B节用(x,y,κ,g)预测z；查询处的κ/g来自5近邻IDW。没有曲面区域生长或二次面片分割。",
      "原代码使用RQ（有理二次）核，它不是二次多项式回归。BGK融合代价/方差，并不融合高程曲面。", "",
      "本次GP严格复用已有CPU公式参考的模型和固定超参数（长度尺度0.6，alpha=10，噪声方差0.0004），没有为这些图调参。",
      "它按式(6)/(7)对保留观测条件化，不是作者GPyTorch GPU训练程序，也不是完整变分SGP。",
      "训练PCA只拟合一次，未追加作者代码中的225个恒高虚拟点；详见上层README。不能将本结果称为论文GPU性能或完整实验复现。", "",
      "## 二次函数对照", "",
      "两个对照都只拟合同一组M个保留点，没有读取地形真值或为桌子分离高度层。",
      "- 全窗：z=a*u²+b*u*v+c*v²+d*u+e*v+f，u/v以保留点XY均值为原点；归一化坐标求最小二乘后转换回米。",
      "- 局部：每个查询XY以最近保留点的Z为锚，找20个保留点的3D近邻，拟合局部二次高度函数；XY设计退化处未知。",
      "这两种拟合都属于额外对照，不能称为FSGP论文的二次拟合算法。两个对照和GP的支持范围、邻域含义不同；没有测试速度或宣称普遍精度优势。", "",
      "## 新增组合地形真值", "",
      "z=0.3*clip(x+1,0,2)：低平地 x≤−1；斜坡 −1<x<1，坡度0.3约16.70°；高平地 x≥1，高0.6m。",
      "两个接缝高度连续、坡度不连续。完整场景[-4,4]×[-4,4]m，0.08m规则点；5×5m移动窗口中心x由−0.8到0.8m，8帧间隔0.1s。",
      "每帧最多2500点，然后15%随机丢点、Z噪声标准差0.015m，非LiDAR射线遮挡。三个M共用同一帧观测。",
      "采用101×101查询格；三方法在同一有效格交集比较解析高程真值。以下MAE/RMSE按8帧全部有效格数汇总，单位cm。", "",
      "| M | 方法 | 高程 MAE cm | 高程 RMSE cm | 接缝±0.2m MAE cm | 有效格总数 |", "|---:|---|---:|---:|---:|---:|"]
    for count in COUNTS:
        for method in ["gp","local","global"]:
            item=next(p for p in pooled if p["count"]==count and p["region"]=="all" and p["method"]==method)
            seam=next(p for p in pooled if p["count"]==count and p["region"]=="seams" and p["method"]==method)
            lines.append(f"| {count} | {labels[method]} | {item['mae_m']*100:.3f} | {item['rmse_m']*100:.3f} | {seam['mae_m']*100:.3f} | {item['n']} |")
    lines += ["", "同一物理位置在不同移动帧会被重复观测；以上计数是逐帧查询样本总数，不是独立场景数量。", "",
      "## 旧场景指标与多层限制", "",
      "旧六个场景仅显示输入点重建误差，包含训练点与参与特征计算的点，而且输入带有噪声；不能作为独立测试误差或论文官方真值。",
      "桌子包含地面、桌底、桌面和桌腿，一张z=f(x,y)高度函数不能表达同一XY下的全部高度层。GP和二次对照都会出现混合或外插，未掩盖该结果。", "",
      "## 查看和校验", "",
      "页面：http://127.0.0.1:42482/fsgp-fit/ ，原首页顶部也有入口。", "",
      "三个三维视图同步旋转；曲面均为实面。移动Y剖面滑块查看输入点、GP、局部二次和全窗二次函数；组合场景同时显示解析真值。",
      "静态图固定为M=125、第8帧，交互视图按当前选项重新载入对应数据。灰点是剖面±0.1m带内输入点，不是精确同一Y上的真值点。",
      "所有密集曲面/剖面均重新查询已拟合模型，不是将原25×25高程图插值后冒充模型输出。",
      "原映射源码哈希保持不变；42组保留点均属于特征/体素候选并集，重建原625格GP均值/方差误差<1e-8。",
      "已知二次函数校验全窗和局部对照可恢复精确曲面。audit.json记录校验和输入哈希。显示JSON坐标保留7位小数，完整精度输入保留在NPZ中。", "",
      "论文：https://arxiv.org/html/2503.04134v2#S3", "官方仓库：https://github.com/ZJU-FAST-Lab/FSGP_BGK", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/regression_fits")
    parser.add_argument("--publish",type=Path,default=ROOT/"outputs/visualization/fsgp-fit")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"cases").mkdir(exist_ok=True)
    protocol=json.loads((ROOT/"outputs/cpu/protocol.json").read_text())
    for name,digest in protocol["implementation_sha256"].items():assert sha(ROOT/name)==digest,name
    config=Config(**protocol["config"])
    audits=[];manifest=[];cases=[]
    for scene in ["hill","flat","ramp","steps","quick","table"]:
        run=ROOT/("outputs/table_cpu" if scene=="table" else "outputs/cpu")
        for count in COUNTS:
            evidence=run/f"{scene}_m{count}_evidence.npz"; fitted=run/f"{scene}_m{count}_fsgp.npz"; bgk=run/f"{scene}_m{count}_bgk.npz"
            with np.load(evidence) as d:scan=d["points"]
            with np.load(fitted) as d:saved={k:d[k] for k in d.files}
            with np.load(bgk) as d:fused={k:d[k] for k in d.files}
            source={"evidence_path":str(evidence.relative_to(ROOT.parent)),"evidence_sha256":sha(evidence),"saved_gp_sha256":sha(fitted)}
            case=export_case(scene,7,scan,saved,fused,replace(config,inducing_points=count),source)
            cases.append(case)
        manifest.append({"id":scene,"name":NAMES[scene],"frames":[7]})
        print(f"Reconstructed homepage scene {scene}",flush=True)
    axis=np.arange(-4,4.001,.08);x,y=np.meshgrid(axis,axis);xy=np.column_stack((x.ravel(),y.ravel()))
    clean=np.column_stack((xy,truth_height(xy)))
    np.savez_compressed(args.output/"flat_ramp_clean.npz",points=clean)
    rng=np.random.default_rng(306)
    mappers={count:TraversabilityMapper(replace(config,inducing_points=count)) for count in COUNTS}
    for frame in range(8):
        center=(-.8+1.6*frame/7,0.)
        scan=sample_scan(clean,center,config,rng)
        np.savez_compressed(args.output/f"flat_ramp_scan_{frame:03d}.npz",points=scan,center=center)
        for count in COUNTS:
            mapper=mappers[count]
            saved=mapper.assess(scan,center,timestamp=frame*.1);fused=mapper.fuse(saved)
            np.savez_compressed(args.output/f"flat_ramp_m{count}_{frame:03d}.npz",**{key:saved[key] for key in ["xy","height","variance","inducing","observed","cost"]},bgk_cost=fused["cost"])
            source={"clean_points_sha256":sha(args.output/"flat_ramp_clean.npz"),"scan_sha256":sha(args.output/f"flat_ramp_scan_{frame:03d}.npz"),"center":center}
            cases.append(export_case("flat_ramp",frame,scan,saved,fused,replace(config,inducing_points=count),source))
        print(f"Tested combined flat/ramp frame {frame+1}/8",flush=True)
    manifest.append({"id":"flat_ramp","name":NAMES["flat_ramp"],"frames":list(range(8))})
    for case in cases:
        cleaned=clean_json(case)
        path=args.output/"cases"/f"{case['scene']}_m{case['count']}_{case['frame']:03d}.json"
        path.write_text(json.dumps(cleaned,ensure_ascii=False,separators=(",",":"),allow_nan=False))
        audits.append({"case":path.stem,"sha256":sha(path),"partition":case["partition"],
                       "grid_reconstruction_max_error_m":case["grid_reconstruction_max_error_m"],
                       "variance_reconstruction_max_error":case["variance_reconstruction_max_error"],
                       "analytic_errors":case["analytic_errors"]})
        if case["count"]==125 and case["frame"]==7:figures(cleaned,args.output)
    settings={"scenes":manifest,"counts":COUNTS,"default_scene":"flat_ramp","default_frame":7,"default_count":125,
              "config":config.as_dict(),"source_implementation_sha256":protocol["implementation_sha256"],
              "paper_url":"https://arxiv.org/html/2503.04134v2","paper_code":"https://github.com/ZJU-FAST-Lab/FSGP_BGK",
              "method":"KNN feature extraction, threshold feature/non-feature partition, voxel representatives for non-features, union and random cap M. No geometric region-growing segmentation.",
              "quadratic_controls":"Global z=a*u^2+b*u*v+c*v^2+d*u+e*v+f; local quadratic uses 20 nearest retained 3D points anchored at nearest XY height. Both fit the same M retained points as GP; neither is a paper method.",
              "scope":"Equation-based CPU reference, not author GPU code or the complete 2500-scene benchmark. Dense plots evaluate the fitted model anew; they are not interpolation of saved coarse height grids.",
              "sensor":"Existing homepage final-frame scans preserved. New combined scene: cap 2500, random dropout 15%, added Z noise std 0.015m, 8 moving windows at 0.1s; random dropout is not ray-traced occlusion.",
              "combined_scene":"z=0.3*clip(x+1,0,2): low flat x<=-1, ramp -1<x<1 (16.70 degrees), high flat z=0.6m x>=1; no height step at seams."}
    (args.output/"manifest.json").write_text(json.dumps(clean_json(settings),ensure_ascii=False,indent=2))
    (args.output/"audit.json").write_text(json.dumps(clean_json({"passed":True,"calibration":calibration(),"cases":audits,
         "exporter_sha256":sha(Path(__file__)),"checks":["Frozen original mapping sources match hashes","Each retained observation belongs to the paper feature/voxel candidate union","Every saved GP height and variance reconstructed within 1e-8","Same retained XYZ used by both quadratic controls","Combined analytic truth evaluated on a common supported grid"]}),indent=2))
    summary=[{"frame":c["frame"],"count":c["count"],"errors":c["analytic_errors"]} for c in cases if c["scene"]=="flat_ramp"]
    (args.output/"combined_metrics.json").write_text(json.dumps(clean_json(summary),indent=2))
    write_report(cases,settings,args.output)
    for path in (ROOT/"visualization/fits").iterdir():
        if path.is_file():shutil.copy2(path,args.output/path.name)
    vendor=ROOT.parent/"m20_3d_planner/visualization/assets/vendor"
    (args.output/"vendor").mkdir(exist_ok=True)
    for name in ["three.module.min.js","OrbitControls.js","LICENSE.txt","README.md"]:shutil.copy2(vendor/name,args.output/"vendor"/name)
    if args.publish:
        args.publish.mkdir(parents=True,exist_ok=True);shutil.copytree(args.output,args.publish,dirs_exist_ok=True);home_entry(args.publish.parent/"index.html")
    print(f"Exported {len(cases)} verified fits to {args.publish}",flush=True)


if __name__=="__main__":main()
