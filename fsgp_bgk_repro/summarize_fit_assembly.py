"""Count failed fits and valid local models unused by the published meshes.

Model counts and triangle counts are separate. A valid model can appear in
retained triangles while some of its other candidate connections are rejected.
This is a read-only diagnostic of the published scene results.
"""
import csv
import html
import json
from pathlib import Path

import numpy as np
from scipy.spatial import Delaunay, QhullError

from .scene_fit_catalog import OUT, file_sha
from .table_local_surfaces import evaluate_models

REPORT = OUT / "fit-assembly-stats"


def summarize_scene(meta):
    directory = OUT / meta["id"]
    with np.load(directory / "results.npz", allow_pickle=False) as data:
        points, coefficient = data["points"], data["coefficients"]
        height, rmse, rank = data["height"], data["rmse"], data["full_rank"]
        labels, faces = data["labels"], data["triangles"]
        n = len(points)
        solved = data["neighbor_ids"][:, 0] >= 0
        valid = np.isfinite(height)
        used = np.zeros(n, dtype=bool)
        used[np.unique(faces)] = True
        assert not np.any(used & ~valid)
        normal = np.column_stack((-coefficient[:, 1], -coefficient[:, 2], np.ones(n)))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        rejected_incidence = np.zeros(n, dtype=np.int32)
        candidate_count = rejected_count = 0
        reasons = {key: 0 for key in ("edge_length", "model_agreement", "normal_angle", "centroid_agreement")}
        candidate_xy_area = rejected_xy_area = 0.
        for component in meta["components"]:
            ids = np.flatnonzero(labels == component["id"])
            try:
                tri = Delaunay(points[ids, :2])
            except QhullError:
                assert component["triangle_count"] == 0
                continue
            candidates = ids[tri.simplices]
            count = len(candidates)
            passing = {key: np.ones(count, bool) for key in reasons}
            for a, b in ((0, 1), (1, 2), (2, 0)):
                left, right = candidates[:, a], candidates[:, b]
                delta = np.column_stack((points[left, :2]-points[right, :2], height[left]-height[right]))
                passing["edge_length"] &= np.linalg.norm(delta, axis=1) <= meta["settings"]["connect_radius_m"]
                passing["normal_angle"] &= np.einsum("ni,ni->n", normal[left], normal[right]) >= np.cos(np.deg2rad(meta["settings"]["max_normal_angle_deg"]))
                for source, target in ((left, right), (right, left)):
                    prediction = evaluate_models(points, coefficient, source, points[target, :2])
                    passing["model_agreement"] &= np.abs(prediction-height[target]) <= meta["settings"]["max_model_disagreement_m"]
            xy = points[candidates, :2]
            center_xy, center_z = xy.mean(axis=1), height[candidates].mean(axis=1)
            for corner in range(3):
                prediction = evaluate_models(points, coefficient, candidates[:, corner], center_xy)
                passing["centroid_agreement"] &= np.abs(prediction-center_z) <= meta["settings"]["max_model_disagreement_m"]
            keep = np.ones(count, bool)
            for key, passed in passing.items():
                reasons[key] += int(np.sum(keep & ~passed))
                keep &= passed
            start, end = component["triangle_start"], component["triangle_start"]+component["triangle_count"]
            np.testing.assert_array_equal(candidates[keep], faces[start:end])
            rejected_incidence += np.bincount(candidates[~keep].ravel(), minlength=n).astype(np.int32)
            candidate_count += count
            rejected_count += int((~keep).sum())
            area = .5*np.abs(np.cross(xy[:, 1]-xy[:, 0], xy[:, 2]-xy[:, 0]))
            candidate_xy_area += float(area.sum())
            rejected_xy_area += float(area[~keep].sum())
        # These are disjoint counts of LOCAL FUNCTIONS / fitting centers.
        counts = {
            "centers": n,
            "not_solved_insufficient": int((~solved).sum()),
            "solved_but_invalid": int((solved & ~valid).sum()),
            "no_valid_fit": int((~valid).sum()),
            "valid": int(valid.sum()),
            "valid_unused": int((valid & ~used).sum()),
            "valid_unused_small_component": int((valid & ~used & (labels < 0)).sum()),
            "valid_unused_major_component": int((valid & ~used & (labels >= 0)).sum()),
            "valid_used": int(used.sum()),
            "valid_used_with_rejected_adjacent_faces": int((used & (rejected_incidence > 0)).sum()),
            "valid_used_without_rejected_adjacent_faces": int((used & (rejected_incidence == 0)).sum()),
        }
        counts["fit_invalid_rank_first"] = int((solved & ~rank).sum())
        shifted = np.abs(coefficient[:, 0]) > meta["settings"]["seed_height_gate_m"]
        counts["fit_invalid_height_shift_after_rank"] = int((solved & rank & shifted).sum())
        counts["fit_invalid_rmse_after_rank_shift"] = int((solved & rank & ~shifted & (rmse > meta["settings"]["max_fit_rmse_m"])).sum())
        assert counts["centers"] == counts["no_valid_fit"]+counts["valid_unused"]+counts["valid_used"]
        assert counts["no_valid_fit"] == counts["not_solved_insufficient"]+counts["solved_but_invalid"]
        assert counts["solved_but_invalid"] == sum(counts[k] for k in ("fit_invalid_rank_first", "fit_invalid_height_shift_after_rank", "fit_invalid_rmse_after_rank_shift"))
        assert counts["valid_used"] == counts["valid_used_with_rejected_adjacent_faces"]+counts["valid_used_without_rejected_adjacent_faces"]
        assert counts["valid_unused"] == counts["valid_unused_small_component"]+counts["valid_unused_major_component"]
        assert counts["valid"] == meta["valid"]
        assert candidate_count == rejected_count+len(faces)
        assert sum(reasons.values()) == rejected_count
    return dict(id=meta["id"], name=meta["name"], counts=counts,
                percentages_of_centers={key: counts[key]/n*100 for key in ("no_valid_fit", "valid_unused", "valid_used")},
                triangles=dict(candidates=candidate_count, kept=len(faces), rejected=rejected_count,
                               first_failure_reasons=reasons),
                projected_area=dict(candidate_xy_hulls_m2=candidate_xy_area,
                                    rejected_xy_m2=rejected_xy_area,
                                    note="Per-component XY hull area; not actual missing 3D area or image pixels."),
                result_sha256=file_sha(directory / "results.npz"))


def export():
    REPORT.mkdir(parents=True, exist_ok=True)
    catalog = json.loads((OUT / "catalog.json").read_text())
    records = [summarize_scene(meta) for meta in catalog["scenes"]]
    result = {"scene_count":len(records), "unit":"fitting center / local quadratic function",
              "definitions":{
                  "no_valid_fit":"No valid model: insufficient neighbors or solved but failed rank/height/RMSE checks. Does not mean a mathematical fit is impossible.",
                  "valid_unused":"Valid local model whose seed appears in zero retained triangles. Includes small-component filtering and all adjacent faces rejected.",
                  "valid_used":"Valid model whose seed appears in at least one retained triangle; some other adjacent candidate triangles may still be rejected.",
                  "partial_rejection":"valid_used_with_rejected_adjacent_faces is a subset of valid_used, not a fourth additive category.",
                  "triangles":"Counted separately from local functions. First-failure order: edge length, model disagreement, normal angle, centroid disagreement.",
                  "scope":"Current complete input clouds and 0.12 m seed selection; scene counts overlap in source geometry and are not independent samples."},
              "scenes":records, "checks":{"partitions_sum_to_all_centers":True,"published_triangle_arrays_exactly_reproduced":True,"fit_outputs_unchanged":True}}
    (REPORT / "data.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    fields = ["scene", "centers", "no_valid_fit", "not_solved_insufficient", "solved_but_invalid", "valid_unused", "valid_used", "valid_used_with_rejected_adjacent_faces", "candidate_triangles", "rejected_triangles"]
    with (REPORT / "summary.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in records:
            c=row["counts"]
            writer.writerow({"scene":row["id"], **{k:c[k] for k in fields if k in c},
                             "candidate_triangles":row["triangles"]["candidates"], "rejected_triangles":row["triangles"]["rejected"]})
    rows=[]
    for row in records:
        c=row["counts"];p=row["percentages_of_centers"]
        cells=[html.escape(row["name"]),f'{c["centers"]:,}',f'{c["no_valid_fit"]:,} ({p["no_valid_fit"]:.2f}%)',
               f'{c["not_solved_insufficient"]:,} / {c["solved_but_invalid"]:,}',
               f'{c["valid_unused"]:,} ({p["valid_unused"]:.2f}%)',f'{c["valid_used"]:,} ({p["valid_used"]:.2f}%)',
               f'{c["valid_used_with_rejected_adjacent_faces"]:,}',
               f'{row["triangles"]["rejected"]:,} / {row["triangles"]["candidates"]:,}']
        rows.append('<tr id="'+row["id"]+'">'+''.join('<td>'+cell+'</td>' for cell in cells)+'</tr>')
    page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>拟合失败与组合缺口统计</title><link rel="icon" href="data:,"><style>
    body{font:14px/1.8 system-ui,sans-serif;color:#294459;background:#f1f5f8;margin:0}main{max-width:1450px;margin:auto;padding:28px 22px}h1{font-size:26px}h2{font-size:18px}a{color:#36799d}p{max-width:1100px}.table{overflow-x:auto;background:white;border:1px solid #d5e1e9;border-radius:8px}table{border-collapse:collapse;width:100%;white-space:nowrap;text-align:right;font-size:12px}th,td{padding:11px;border-bottom:1px solid #e1e9ef}th{background:#eaf1f6;color:#5e7d92}th:first-child,td:first-child{text-align:left}tr:target{background:#fff1cc}.note{font-size:12px;color:#71899b}.links{display:flex;gap:20px;flex-wrap:wrap}
    </style></head><body><main><a href="../../">← 42482 全场景拟合</a><h1>拟合失败，与拟合后未能成面，分别有多少？</h1>
    <p>统计单位统一为<strong>拟合中心 / 局部函数</strong>。前三个主要类别互斥，数量之和等于全部中心；三角形单独统计。</p>
    <p><strong>未得到有效拟合</strong>：邻点不足，或已经求解但未通过几何退化、偏移、残差检查。<strong>有效但完全未成面</strong>：函数有效，但中心没有出现在任何最终保留的三角形中。<strong>已参与成面</strong>：至少参与一个最终三角形，周边仍可能有部分连接被删除。</p>
    <div class="table"><table><thead><tr><th>场景</th><th>全部中心</th><th>未得到有效拟合</th><th>其中：未求解 / 求解后剔除</th><th>有效但完全未成面</th><th>已参与成面</th><th>其中仍有邻接缺面</th><th>删除 / 候选三角形</th></tr></thead><tbody>'''+''.join(rows)+'''</tbody></table></div>
    <p class="note">“其中仍有邻接缺面”包含在“已参与成面”中，不能再次相加。函数计数不能与三角形计数相加；这些比例也不是图像空白面积占比。小组件少于 30 个有效中心会被当前规则过滤；部分主要组件中的中心也可能因所有邻接三角形被删而未成面。</p>
    <h2>如何理解丘陵截图中的白块</h2><p>丘陵上既有有效函数完全没有进入最终网格，也有已经参与网格的函数，其部分邻接三角形未满足连接条件。因此仅统计“完全未成面”会漏掉部分白块。连接门限为距离 ≤0.35 m、双向预测高差 ≤2 cm、法向夹角 ≤15°，另外检查三角形重心的预测一致性。</p>
    <p class="note">本报告从当前发布的完整精度结果重新统计；重建的全部保留三角形与网页发布数组逐项相同，没有重新拟合或更改门限。不同场景可能复用同一原始地形，不把跨场景总数解释为独立样本。</p>
    <div class="links"><a href="./summary.csv" download>下载全部场景 CSV</a><a href="./data.json">详细统计与校验 JSON</a></div></main></body></html>'''
    (REPORT / "index.html").write_text(page)
    for row in records:
        print(json.dumps({"scene":row["id"], **row["counts"], "deleted_triangles":row["triangles"]["rejected"], "candidate_triangles":row["triangles"]["candidates"]}, ensure_ascii=False))
    return result


if __name__ == "__main__":
    export()
