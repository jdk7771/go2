"""Analytic display completion of enclosed hill gaps smaller than 9 cm².

Coefficients, observations, fitting domains and metrics are unchanged. Whole
connected gaps are measured in the exact XY union, then separately rendered
with heights from individual neighboring quadratic functions. Larger gaps and
the exterior remain open. The overlay is not a new fit or new observation.
"""
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial import cKDTree

from .assessment_export import sha,finite_json
from .convex_surface_assessment import OUT,FIELDS,PROFILES,inside
from .polygon_topology import PolygonTopology
from .terrain_assessment import basis

REPORT=OUT/'hill-small-gaps'
AREA_LIMIT_M2=9e-4
AREA_EPSILON_M2=1e-12


def is_small_gap(area):return AREA_EPSILON_M2<area<AREA_LIMIT_M2


def subdivide(triangles,maximum_edge=.04):
    pending=np.asarray(triangles,float).reshape(-1,3,2);accepted=[]
    while len(pending):
        length=np.linalg.norm(np.roll(pending,-1,axis=1)-pending,axis=2)
        split=length.max(1)>maximum_edge+1e-12
        if (~split).any():accepted.append(pending[~split])
        p=pending[split]
        if not len(p):break
        edge=np.argmax(length[split],axis=1);row=np.arange(len(p))
        a,b,c=p[row,edge],p[row,(edge+1)%3],p[row,(edge+2)%3];middle=(a+b)/2
        pending=np.r_[np.stack((a,middle,c),axis=1),np.stack((middle,b,c),axis=1)]
    return np.concatenate(accepted) if accepted else np.empty((0,3,2))


def load_models(profile,scene='hill'):
    folder=OUT/profile/scene;meta=json.loads((folder/'data.json').read_text())
    model=np.fromfile(folder/'models.bin','<f8').reshape(-1,len(FIELDS))
    active=np.flatnonzero(model[:,FIELDS.index('valid')]>0)
    xy=np.fromfile(folder/'polygons.bin','<f8').reshape(-1,2);offset=np.fromfile(folder/'polygon-offsets.bin','<i4')
    polygons=[xy[offset[i]:offset[i+1]]+model[i,:2] for i in active]
    return folder,meta,model,active,polygons


def boundary_compatible(triangles,candidates,model,polygon_xy,offsets,tolerance=.04):
    """Require an owner's height to agree with observed hulls at gap vertices.

    Interior vertices have no boundary support and impose no check. At a
    multi-level boundary at least one observed surface must match; surfaces
    at other heights must not veto a valid continuation on the same level.
    """
    delta=triangles[:,None,:,:]-model[candidates,None,:2]
    heights=model[candidates,None,2]+np.einsum('nkvi,nki->nkv',basis(delta),model[candidates,3:9])
    supported=np.zeros(heights.shape,bool)
    order=np.argsort(candidates.ravel());parents,starts,counts=np.unique(candidates.ravel()[order],return_index=True,return_counts=True)
    for parent,start,count in zip(parents,starts,counts):
        row,col=np.divmod(order[start:start+count],candidates.shape[1])
        q=triangles[row].reshape(-1,2)-model[parent,:2]
        supported[row,col]=inside(q,polygon_xy[offsets[parent]:offsets[parent+1]]).reshape(-1,3)
    any_support=supported.any(1)
    good=np.ones(candidates.shape,bool)
    for col in range(candidates.shape[1]):
        difference=np.where(supported,np.abs(heights[:,col,None,:]-heights),np.inf).min(1)
        good[:,col]=np.all(~any_support|(difference<=tolerance),axis=1)
    return good


def build(profile,label,scene='hill',report=REPORT,check_boundary=False):
    started=time.perf_counter();folder,meta,model,active,polygons=load_models(profile,scene)
    base_hashes={name:sha(folder/name) for name in
        ('models.bin','radii.bin','polygons.bin','polygon-offsets.bin','neighbors.bin','neighbor-offsets.bin',
         'patches.bin','patch-faces.bin','patch-owners.bin','points.bin','covariance.bin','metric-samples.bin',
         'raw-obstacle-hits.bin','per-model.csv','data.json')}
    tree=cKDTree(model[active,:2]);radii=np.fromfile(folder/'radii.bin','<f4').astype(float)
    fallback=.15 if profile=='q15-n15' else .20
    radii=np.where(np.isfinite(radii),radii,fallback/np.sqrt(2))
    region_ids=[];all_triangles=[];regions=[]
    with PolygonTopology() as geo:
        union=geo.union(polygons);union_area=geo.area(union)
        gaps=[g for g in geo.enclosed_gaps(union) if g['area']>AREA_EPSILON_M2]
        small=[g for g in gaps if is_small_gap(g['area'])];large=[g for g in gaps if not is_small_gap(g['area'])]
        for index,region in enumerate(small):
            triangles=subdivide(geo.triangles(region))
            if not len(triangles):raise RuntimeError('A positive-area small gap could not be triangulated')
            all_triangles.append(triangles);region_ids.extend([index]*len(triangles));regions.append(region)
        triangles=np.concatenate(all_triangles) if all_triangles else np.empty((0,3,2))
        region_ids=np.asarray(region_ids,np.int32)
        _,near=tree.query(triangles.mean(1),k=min(12,len(active))) if len(triangles) else (None,np.empty((0,12),int))
        near=np.asarray(near).reshape(len(triangles),-1) if len(triangles) else np.empty((0,12),int)
        candidates=active[near]
        delta=triangles[:,None,:,:]-model[candidates,None,:2]
        reach=np.linalg.norm(delta,axis=3).max(2)
        score=reach/np.maximum(radii[candidates],1e-8)
        allowed=reach<=radii[candidates]+.04+1e-9
        if check_boundary and len(triangles):
            polygon_xy=np.fromfile(folder/'polygons.bin','<f8').reshape(-1,2)
            offsets=np.fromfile(folder/'polygon-offsets.bin','<i4')
            allowed&=boundary_compatible(triangles,candidates,model,polygon_xy,offsets)
        score[~allowed]=np.inf
        choice=np.argmin(score,axis=1) if len(score) else np.empty(0,int)
        good=np.isfinite(score[np.arange(len(score)),choice])
        # Never partially hide a connected region and claim it was completed.
        rejected=set(region_ids[~good].tolist());keep=np.array([i not in rejected for i in region_ids],bool)
        triangles=triangles[keep];region_ids=region_ids[keep]
        owners=candidates[np.arange(len(candidates)),choice][keep] if len(candidates) else np.empty(0,int)
        filled=[g for i,g in enumerate(regions) if i not in rejected]
        delta=triangles-model[owners,None,:2]
        z=model[owners,None,2]+np.einsum('nvi,ni->nv',basis(delta),model[owners,3:9])
        positions=np.concatenate((triangles,z[:,:,None]),axis=2).reshape(-1,3).astype('<f4')
        faces=np.arange(len(positions),dtype='<u4').reshape(-1,3)
        vertex_owners=np.repeat(owners,3).astype('<u4')
        final_union=geo.union_with(union,filled)
        after=[g for g in geo.enclosed_gaps(final_union) if g['area']>AREA_EPSILON_M2]
        after_large=sorted(g['area'] for g in after if not is_small_gap(g['area']))
        np.testing.assert_allclose(after_large,sorted(g['area'] for g in large),rtol=1e-8,atol=1e-10)
        added_area=sum(g['area'] for g in filled)
        np.testing.assert_allclose(geo.area(final_union)-union_area,added_area,rtol=1e-8,atol=1e-8)
        a,b=triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]
        triangle_area=float(np.sum(np.abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]))/2)
        np.testing.assert_allclose(triangle_area,added_area,rtol=1e-8,atol=1e-10)
        outside=np.linalg.norm(delta,axis=2)-radii[owners,None]
        info=dict(scene=scene,profile=profile,label=label,threshold_cm2=9.,strictly_less=True,
            raw_points=meta['raw_points'],valid_functions=meta['valid'],new_fits=0,
            original_xy_union_area_m2=union_area,display_xy_union_area_m2=geo.area(final_union),
            enclosed_gaps_before=len(gaps),small_gaps_before=len(small),filled_gaps=len(filled),
            filled_area_m2=added_area,large_gaps_retained=len(large),
            largest_retained_gap_cm2=max((g['area'] for g in large),default=0)*10000,
            unfilled_small_gaps=len(small)-len(filled),remaining_enclosed_gaps=len(after),
            overlay_triangles=len(faces),maximum_reach_beyond_parent_radius_m=float(np.maximum(outside,0).max(initial=0)),
            geometry=dict(method='GEOS exact union; whole connected enclosed gaps; constrained triangulation',
                height='Each display triangle evaluates one original neighboring quadratic function',
                whole_connected_area=True,large_gaps_unchanged=True,exterior_unchanged=True,maximum_edge_m=.04,
                boundary_height_check=check_boundary,boundary_height_tolerance_m=.04 if check_boundary else None,
                note='Display completion outside original per-function convex domains; no new observations or solved functions.'),
            files=dict(positions='positions.bin',faces='faces.bin',owners='owners.bin'),
            original_files_sha256=base_hashes,elapsed_ms=(time.perf_counter()-started)*1000)
    output=report/profile if report==REPORT else report/profile/scene
    output.mkdir(parents=True,exist_ok=True)
    positions.tofile(output/'positions.bin');faces.tofile(output/'faces.bin');vertex_owners.tofile(output/'owners.bin')
    (output/'regions.json').write_text(json.dumps(finite_json(dict(
        filled=[dict(area_m2=g['area'],exterior=g['exterior'],holes=g['holes']) for g in filled],
        retained_large_areas_m2=[g['area'] for g in large])),ensure_ascii=False)+'\n')
    (output/'data.json').write_text(json.dumps(finite_json(info),ensure_ascii=False,indent=2)+'\n')
    assert all(sha(folder/name)==h for name,h in base_hashes.items())
    print(f"{scene} / {profile}: filled {len(filled)}/{len(small)} small gaps, {added_area:.6f} m²; retained {len(large)} large gaps",flush=True)
    return info


def audit_saved(records,report=REPORT,manifest=Path('/tmp/go2-before-offset-ablation.json')):
    checked=[]
    for record in records:
        profile=record['profile'];scene=record['scene'];original=OUT/profile/scene
        folder=report/profile if report==REPORT else report/profile/scene
        assert all(sha(original/name)==digest for name,digest in record['original_files_sha256'].items())
        regions=json.loads((folder/'regions.json').read_text())
        assert len(regions['filled'])==record['filled_gaps']
        assert all(is_small_gap(g['area_m2']) for g in regions['filled'])
        assert all(a>=AREA_LIMIT_M2 for a in regions['retained_large_areas_m2'])
        v=np.fromfile(folder/'positions.bin','<f4').reshape(-1,3).astype(float)
        faces=np.fromfile(folder/'faces.bin','<u4').reshape(-1,3)
        owners=np.fromfile(folder/'owners.bin','<u4');m=np.fromfile(original/'models.bin','<f8').reshape(-1,len(FIELDS))
        assert np.isfinite(v).all() and np.all(m[owners,FIELDS.index('valid')]>0)
        assert np.all(owners[faces]==owners[faces[:,0]][:,None])
        predicted=m[owners,2]+np.einsum('ni,ni->n',basis(v[:,:2]-m[owners,:2]),m[owners,3:9])
        error=float(np.max(np.abs(v[:,2]-predicted),initial=0));assert error<5e-5
        checked.append(dict(scene=scene,profile=profile,original_files_verified=len(record['original_files_sha256']),
            filled_regions_under_threshold=True,large_gap_areas_unchanged=True,
            one_original_function_per_triangle=True,maximum_float32_height_error_m=error,new_fits=0))
    preservation=None
    if manifest.exists():
        snapshot=json.loads(manifest.read_text());changed=[p for p,digest in snapshot.items() if sha(Path(p))!=digest]
        assert not changed,changed;preservation=dict(files_verified=len(snapshot),changed_files=changed)
    result=dict(passed=True,records=checked,baseline_preservation=preservation)
    (report/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


def run():
    REPORT.mkdir(parents=True,exist_ok=True);records=[build(profile,label) for profile,label in PROFILES]
    audit_saved(records)
    summary=dict(date='2026-11-22',scene='hill',threshold_cm2=9.,raw_points_unchanged=True,
        coefficients_unchanged=True,metrics_unchanged=True,large_gaps_unchanged=True,records=records)
    (REPORT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    lines=['# 丘陵：小于 9 cm² 的连通小缝显示补全','','日期：2026-11-22。入口 http://127.0.0.1:42482/ 。','',
        '按照最终要求，大空档保留，不继续拟合；只把面积严格小于 9 cm² 的封闭连通间隙并入蓝色覆盖。按整个连通区域面积判断，连起来的大空档不会拆成小三角形后被填掉。外侧敞口不处理。','',
        '这是显示补全：原始 200,000 个 XYZ 点、中心、邻域、六系数、协方差、六项指标与原凸包域全部不变，不增加拟合求解。补缝三角形的 Z 来自相邻原有函数，按该函数解析求值，并沿用它的指标颜色。补缝显示范围不代表新增实测观测。','',
        '| 设置 | 原覆盖 XY 并集 | 补后显示并集 | 补全小缝数 | 补全面积 | 保留大空档数 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in records:
        lines.append(f"| {r['label']} | {r['original_xy_union_area_m2']:.6f} m² | {r['display_xy_union_area_m2']:.6f} m² | {r['filled_gaps']:,} | {r['filled_area_m2']:.6f} m² | {r['large_gaps_retained']:,} |")
    lines+=['','面积是 XY 投影并集，重叠只计一次，不是曲面三维面积。仅识别并处理被覆盖面包围的封闭空洞；大空档与外侧边界不被遮盖。小于 1e−12 m² 的数值碎片忽略。','',
        'GEOS 精确并集计算连通空档及面积，约束三角化仅用于画出已经选定的小缝，不参与判定面积。每个显示三角形独立归属一个已有函数。补缝与原凸包几何分开保存，基础拟合文件通过 SHA256 核对未改变。','',
        '[丘陵：蓝色覆盖](http://127.0.0.1:42482/?scene=hill&profile=qcircle-adaptive&coverage=1&gaps=1) · '
        '[原始覆盖](http://127.0.0.1:42482/?scene=hill&profile=qcircle-adaptive&coverage=1&gaps=0) · [机器可读结果](summary.json) · [几何与数据核查](audit.json)','',
        '页面勾选“补齐小缝（<9 cm²）”查看效果。该显示开关只在丘陵可用，四种基础拟合设置分别计算。指标图中真正的低值区域仍显示白色；本操作消除的是覆盖模式下的小缝背景。','',
        '复现：在原丘陵四设置结果存在时，使用系统 Python 运行 `python -m fsgp_bgk_repro.hill_small_gaps`，然后运行 `python -m fsgp_bgk_repro.publish_metric_viewer`；本机需要 PYTHONNOUSERSITE=1，系统 GEOS C 库，不需要安装 Shapely。','']
    (REPORT/'report.md').write_text('\n'.join(lines));return summary


if __name__=='__main__':run()
