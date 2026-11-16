"""Time the exact local quadratic used by the fit viewer, including KNN.

Frozen homepage observations and retained points are reused for post-sampling
timings. A separate raw-XYZ experiment runs feature extraction and new,
deterministic feature/voxel sampling for BOTH methods on each timed call.
"""

import argparse
from dataclasses import replace
import hashlib
import inspect
import json
from pathlib import Path
import shutil
from time import perf_counter_ns

import numpy as np
from scipy.spatial import cKDTree

from .core import Config, SparseTerrainGP, extract_features, interpolate_features, select_inducing
from .experiment import environment
from .visualize_regression import local_quadratic

ROOT=Path(__file__).resolve().parent
NAMES={"flat_ramp":"平地—斜坡组合","hill":"丘陵","flat":"平地","ramp":"斜坡","steps":"台阶","quick":"quick","table":"桌子"}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def statistics(samples):
    return {"median_ms":float(np.median(samples)),"p95_ms":float(np.percentile(samples,95)),
            "minimum_ms":float(np.min(samples)),"samples_ms":samples}


def time_tasks(tasks,repeats,warmups):
    for function in tasks.values():
        for _ in range(warmups):function()
    samples={key:[] for key in tasks}
    rng=np.random.default_rng(461)
    names=list(tasks)
    for _ in range(repeats):
        for index in rng.permutation(len(names)):
            key=names[index];start=perf_counter_ns();result=tasks[key]()
            samples[key].append((perf_counter_ns()-start)/1e6)
            del result
    return {key:statistics(values) for key,values in samples.items()}


def gp_mean(gp,query,batch_size=512):
    """Same conditional mean, omitting the variance solve and derivatives."""
    values=[]
    for start in range(0,len(query),batch_size):
        transformed=(query[start:start+batch_size]-gp.input_mean)@gp.rotation
        values.append(gp.height_mean+gp.kernel(transformed,gp.train_x)@gp.coefficients)
    return np.concatenate(values)


def staged_local(points,xy,neighbors=20):
    """Equivalent formula, split only for diagnosing stage costs."""
    t0=perf_counter_ns()
    _,nearest=cKDTree(points[:,:2]).query(xy)
    anchor=points[nearest,2]
    _,ids=cKDTree(points).query(np.column_stack((xy,anchor)),k=min(neighbors,len(points)))
    t1=perf_counter_ns()
    local=points[ids]
    u,v=(local[:,:,:2]-xy[:,None,:]).transpose(2,0,1)
    scale=np.maximum(np.sqrt(np.mean(u*u+v*v,axis=1)),1e-10)
    u,v=u/scale[:,None],v/scale[:,None]
    design=np.stack((np.ones_like(u),u,v,u*u,u*v,v*v),axis=2)
    gram=np.einsum("nki,nkj->nij",design,design)
    t2=perf_counter_ns()
    spectrum=np.linalg.eigvalsh(gram)
    valid=spectrum[:,0]>np.maximum(spectrum[:,-1],1e-30)*1e-10
    rhs=np.einsum("nki,nk->ni",design,local[:,:,2]-anchor[:,None])
    t3=perf_counter_ns()
    ridge=1e-10*np.trace(gram,axis1=1,axis2=2)/6
    coefficient=np.linalg.solve(gram+ridge[:,None,None]*np.eye(6),rhs)
    height=anchor+coefficient[:,0]
    valid &= np.isfinite(height)
    t4=perf_counter_ns()
    return np.where(valid,height,np.nan),valid,{"knn_ms":(t1-t0)/1e6,
        "normalization_and_gram_ms":(t2-t1)/1e6,"rank_and_rhs_ms":(t3-t2)/1e6,
        "solve_and_output_ms":(t4-t3)/1e6,"instrumented_total_ms":(t4-t0)/1e6}


def load_case(scene,count):
    if scene=="flat_ramp":
        folder=ROOT/"outputs/regression_fits"
        evidence=folder/"flat_ramp_scan_007.npz";fitted=folder/f"flat_ramp_m{count}_007.npz"
    else:
        folder=ROOT/("outputs/table_cpu" if scene=="table" else "outputs/cpu")
        evidence=folder/f"{scene}_m{count}_evidence.npz";fitted=folder/f"{scene}_m{count}_fsgp.npz"
    with np.load(evidence) as d:scan=d["points"]
    with np.load(fitted) as d:saved={k:d[k] for k in d.files}
    return scan,saved,{"scan":str(evidence.relative_to(ROOT.parent)),"scan_sha256":sha(evidence),
        "model":str(fitted.relative_to(ROOT.parent)),"model_sha256":sha(fitted)}


def queries(saved,dense):
    if not dense:return saved["xy"]
    x=np.linspace(saved["xy"][:,0].min(),saved["xy"][:,0].max(),101)
    y=np.linspace(saved["xy"][:,1].min(),saved["xy"][:,1].max(),101)
    a,b=np.meshgrid(x,y);return np.column_stack((a.ravel(),b.ravel()))


def ready_tasks(scan,features,retained,xy,config):
    def local():return local_quadratic(retained[:,:3],xy,config.neighbors)
    def supported_local():
        distance,_=cKDTree(scan[:,:2]).query(xy)
        height,valid=local();return height,valid&(distance<=config.support_radius)
    def gp_complete():
        gp=SparseTerrainGP(config).fit(retained[:,[0,1,3,4]],retained[:,2])
        test,distance=interpolate_features(features,xy,config.interpolation_neighbors)
        height,variance,gradient=gp.predict(test)
        return height,variance,gradient,(distance<=config.support_radius)&(variance<=config.max_variance)
    def gp_height():
        gp=SparseTerrainGP(config).fit(retained[:,[0,1,3,4]],retained[:,2])
        test,distance=interpolate_features(features,xy,config.interpolation_neighbors)
        return gp_mean(gp,test),distance<=config.support_radius
    return {"local_function":local,"local_with_support":supported_local,
            "gp_full_outputs":gp_complete,"gp_height_only":gp_height}


def raw_tasks(scan,xy,config):
    def prepare():
        features=extract_features(scan,config.neighbors)
        retained=select_inducing(features,config,np.random.default_rng(config.seed),feature_based=True)
        return features,retained
    def local():
        _,retained=prepare()
        distance,_=cKDTree(scan[:,:2]).query(xy)
        height,valid=local_quadratic(retained[:,:3],xy,config.neighbors)
        return height,valid&(distance<=config.support_radius)
    def gp_complete():
        features,retained=prepare()
        return ready_tasks(scan,features,retained,xy,config)["gp_full_outputs"]()
    def gp_height():
        features,retained=prepare()
        return ready_tasks(scan,features,retained,xy,config)["gp_height_only"]()
    return {"local_from_xyz":local,"gp_full_from_xyz":gp_complete,"gp_height_from_xyz":gp_height,"preprocessing_only":prepare}


def report(result,output):
    lines=["# 当前局部二次函数的速度实测", "",
      "直接计时拟合可视化中调用的local_quadratic()，不修改拟合方法或参数。CPU、单线程BLAS；预热后重复计时，算法按固定随机顺序交错运行。",
      f"硬件：{result['environment']['processor']}；Python {result['environment']['python'].split()[0]}；每项 {result['repeats']} 次，预热 {result['warmups']} 次。", "",
      "## 组合场景，第8帧，M=125，K=20", "",
      "以下为中位数 / P95，单位ms。输入已选点，但仍计入每次建树、KNN搜索、拟合与查询。GP还计入模型构建及查询处的5近邻特征插值。", "",
      "| 查询点数 | 局部二次函数本身 | 加原始观测支持判断 | GP 仅高程 | GP 高程+方差+梯度 |",
      "|---:|---:|---:|---:|---:|"]
    for row in result["ready"]:
        if row["scene"]!="flat_ramp" or row["count"]!=125:continue
        values=[f"{row['timing'][key]['median_ms']:.3f} / {row['timing'][key]['p95_ms']:.3f}" for key in
                ["local_function","local_with_support","gp_height_only","gp_full_outputs"]]
        lines.append(f"| {row['query_points']} | "+" | ".join(values)+" |")
    lines += ["", "GP仅高程项是本次额外的等价均值查询：与现有GP均值数值一致，省略方差求解、梯度及方差门限；没有修改现有映射实现。",
      "局部二次只输出高程及退化标记，不输出GP不确定性或曲率。因此不能将它与GP完整输出的全部时间差解释为同等功能加速。", "",
      "## 局部二次阶段分解", "",
      "分解使用与原函数逐点数值一致的插桩版本；总耗时取上表未插桩的原函数。各项中位数不要求相加等于总中位数。", "",
      "| 查询点数 | 两棵树与邻域搜索 | 坐标归一化/Gram构造 | 退化检查/右端项 | 6×6求解及输出 |", "|---:|---:|---:|---:|---:|"]
    for row in result["stages"]:
        values=[f"{row['timing'][key]['median_ms']:.3f}" for key in
                ["knn_ms","normalization_and_gram_ms","rank_and_rhs_ms","solve_and_output_ms"]]
        lines.append(f"| {row['query_points']} | "+" | ".join(values)+" |")
    lines += ["", "## 原始XYZ到高程：7场景，M=125，625查询", "",
      "每次计时重新计算完整原始观测的20近邻κ/g、特征/体素采样、M点选择、模型/邻域和查询支持。",
      "本组每次以seed=7重新随机选点，两种方法共用同一新选点；它是单独的原始XYZ计时组，不冒充旧可视化第8帧保留点的随机状态。",
      "不含PCD/NPZ读取、图像渲染、通行代价与BGK融合。GP完整项额外输出方差和梯度。", "",
      "| 场景 | 原始观测点 | 公共特征/采样 ms | 局部二次完整 ms | GP仅高程 ms | GP完整输出 ms |",
      "|---|---:|---:|---:|---:|---:|"]
    for row in result["raw"]:
        values=[f"{row['timing'][key]['median_ms']:.3f}" for key in
                ["preprocessing_only","local_from_xyz","gp_height_from_xyz","gp_full_from_xyz"]]
        lines.append(f"| {NAMES[row['scene']]} | {row['scan_points']} | "+" | ".join(values)+" |")
    lines += ["", "## 七场景与M敏感性：已选点后的完整局部二次查询", "",
      "本表局部二次包括原始观测支持判断，GP仅高程与GP完整输出含训练和IDW。全部使用冻结第8帧相同的保留XYZ。", "",
      "| 场景 | M | 查询点数 | 局部二次 ms | GP仅高程 ms | GP完整输出 ms |", "|---|---:|---:|---:|---:|---:|"]
    for row in result["ready"]:
        values=[f"{row['timing'][key]['median_ms']:.3f}" for key in ["local_with_support","gp_height_only","gp_full_outputs"]]
        lines.append(f"| {NAMES[row['scene']]} | {row['count']} | {row['query_points']} | "+" | ".join(values)+" |")
    lines += ["", "## 可复核证据", "",
      "metrics.json保留所有逐次计时、P95、环境、输入与模型SHA256、查询网格和实现快照哈希。source/保存此次计时的源码。",
      "插桩局部拟合与原函数的高程/有效掩码一致；GP仅均值与完整输出均值一致；原始XYZ两方法的选点一致。",
      "不使用这里的耗时推断完整机器人系统频率，也不与论文GPU时间直接比较。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats",type=int,default=15);parser.add_argument("--warmups",type=int,default=3)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/regression_speed")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"source").mkdir(exist_ok=True)
    files=["core.py","visualize_regression.py","benchmark_regression_speed.py"]
    for name in files:shutil.copy2(ROOT/name,args.output/"source"/name)
    config=Config(**json.loads((ROOT/"outputs/cpu/protocol.json").read_text())["config"])
    result={"environment":environment(),"repeats":args.repeats,"warmups":args.warmups,
            "source_sha256":{name:sha(ROOT/name) for name in files},
            "local_function_sha256":hashlib.sha256(inspect.getsource(local_quadratic).encode()).hexdigest(),
            "config":config.as_dict(),"ready":[],"raw":[],"stages":[],"checks":[]}
    for scene in ["flat_ramp","hill","flat","ramp","steps","quick","table"]:
        counts=[125,50,500] if scene=="flat_ramp" else [125]
        for count in counts:
            scan,saved,source=load_case(scene,count);c=replace(config,inducing_points=count)
            features=extract_features(scan,c.neighbors);retained=saved["inducing"]
            for dense in [False,True]:
                xy=queries(saved,dense);tasks=ready_tasks(scan,features,retained,xy,c)
                expected,expected_valid=tasks["local_function"]()
                staged,valid,_=staged_local(retained[:,:3],xy,c.neighbors)
                assert np.array_equal(valid,expected_valid)
                np.testing.assert_allclose(staged,expected,atol=1e-12,rtol=0,equal_nan=True)
                gp_full=tasks["gp_full_outputs"]()[0];gp_just_height=tasks["gp_height_only"]()[0]
                np.testing.assert_allclose(gp_full,gp_just_height,atol=1e-10,rtol=1e-10)
                row={"scene":scene,"frame":7,"count":count,"scan_points":len(scan),"query_points":len(xy),
                     "source":source,"query_sha256":hashlib.sha256(xy.tobytes()).hexdigest(),
                     "timing":time_tasks(tasks,args.repeats,args.warmups)}
                result["ready"].append(row)
                if scene=="flat_ramp" and count==125:
                    samples={key:[] for key in ["knn_ms","normalization_and_gram_ms","rank_and_rhs_ms","solve_and_output_ms","instrumented_total_ms"]}
                    for _ in range(args.repeats):
                        _,_,timing=staged_local(retained[:,:3],xy,c.neighbors)
                        for key,value in timing.items():samples[key].append(value)
                    result["stages"].append({"query_points":len(xy),"timing":{key:statistics(values) for key,values in samples.items()}})
            if count==125:
                xy=queries(saved,False);tasks=raw_tasks(scan,xy,c)
                f,r=tasks["preprocessing_only"]();f2,r2=tasks["preprocessing_only"]()
                assert np.array_equal(r,r2)
                direct=local_quadratic(r[:,:3],xy,c.neighbors)
                np.testing.assert_allclose(tasks["local_from_xyz"]()[0],direct[0],atol=1e-12,equal_nan=True)
                np.testing.assert_allclose(tasks["gp_full_from_xyz"]()[0],tasks["gp_height_from_xyz"]()[0],atol=1e-10)
                result["raw"].append({"scene":scene,"count":125,"scan_points":len(scan),"query_points":len(xy),
                    "retained_sha256":hashlib.sha256(r.tobytes()).hexdigest(),"timing":time_tasks(tasks,args.repeats,args.warmups)})
            print(f"Measured {scene}, M={count}, queries 625/10201",flush=True)
    result["checks"]=["Instrumented local predictions and valid masks match the original function",
                       "Mean-only GP agrees with complete GP to 1e-10",
                       "Raw-XYZ paths use identical deterministic feature/voxel samples",
                       "Data loading and rendering excluded; all functions use single-thread BLAS"]
    (args.output/"metrics.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    report(result,args.output)
    destination=ROOT/"outputs/visualization/fsgp-fit/speed"
    destination.mkdir(parents=True,exist_ok=True);shutil.copytree(args.output,destination,dirs_exist_ok=True)
    print(f"Wrote {args.output/'REPORT.md'}",flush=True)


if __name__=="__main__":main()
