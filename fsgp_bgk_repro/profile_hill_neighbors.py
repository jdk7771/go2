"""Read-only timing breakdown of the published hill's exact neighbor search.

The instrumented copy preserves query order and tie handling. Every measured
output is compared with the published neighbor array; production is unchanged.
"""

import argparse
import json
from time import perf_counter_ns as clock

import numpy as np
from scipy.spatial import cKDTree

from .experiment import environment
from .scene_fit_catalog import OUT, exact_neighbors, file_sha, source_cloud
from .table_local_surfaces import LocalConfig


def detailed_neighbors(points, seeds, config, batch_size=512):
    stages = dict.fromkeys(("build_tree", "batch_knn", "batch_height_filter",
                           "refine_radius", "refine_ball_query", "refine_filter",
                           "tie_sort_write"), 0)
    histogram = {}
    candidate_counts = np.zeros(len(seeds), dtype=np.int32)
    total_start = clock()
    start = clock()
    tree = cKDTree(points)
    stages["build_tree"] += clock() - start
    ids = np.full((len(seeds), config.neighbors), -1, dtype=np.int32)
    for start_index in range(0, len(seeds), batch_size):
        pending = np.arange(start_index, min(start_index + batch_size, len(seeds)))
        k = min(64, len(points))
        while len(pending):
            start = clock()
            distance, nearest = tree.query(seeds[pending], k=k,
                                          distance_upper_bound=config.radius_m + 1e-12)
            stages["batch_knn"] += clock() - start
            start = clock()
            if nearest.ndim == 1:
                nearest, distance = nearest[:, None], distance[:, None]
            exists = nearest < len(points)
            safe = np.minimum(nearest, len(points) - 1)
            eligible = exists & (np.abs(points[safe, 2] - seeds[pending, None, 2])
                                 <= config.seed_height_gate_m)
            enough = eligible.sum(axis=1) >= config.neighbors
            complete = ~exists[:, -1] | (k == len(points))
            stages["batch_height_filter"] += clock() - start
            item = histogram.setdefault(k, {"batch_calls": 0, "rows": 0, "accepted": 0})
            item["batch_calls"] += 1
            item["rows"] += len(pending)
            item["accepted"] += int(enough.sum())
            for row in np.flatnonzero(enough):
                start = clock()
                seed = pending[row]
                radius = distance[row, np.flatnonzero(eligible[row])[config.neighbors - 1]]
                stages["refine_radius"] += clock() - start
                start = clock()
                candidates = np.asarray(tree.query_ball_point(
                    seeds[seed], min(config.radius_m + 1e-12, radius + 1e-10)),
                    dtype=np.int32)
                stages["refine_ball_query"] += clock() - start
                start = clock()
                delta = points[candidates] - seeds[seed]
                d2 = np.einsum("ni,ni->n", delta, delta)
                take = ((np.abs(delta[:, 2]) <= config.seed_height_gate_m)
                        & (d2 <= config.radius_m**2 + 1e-14))
                selected, d2 = candidates[take], d2[take]
                stages["refine_filter"] += clock() - start
                start = clock()
                ids[seed] = selected[np.lexsort((selected, d2))[:config.neighbors]]
                stages["tie_sort_write"] += clock() - start
                candidate_counts[seed] = len(candidates)
            pending = pending[~enough & ~complete]
            k = min(len(points), k * 2)
    total = clock() - total_start
    stages["loop_allocation_and_instrumentation"] = total - sum(stages.values())
    summary = dict(query_histogram=histogram,
                   seeds_requiring_expansion=int(len(seeds) - histogram[64]["accepted"]),
                   per_seed_refinement_calls=int((candidate_counts > 0).sum()),
                   raw_refinement_candidates=dict(
                       median=float(np.median(candidate_counts[candidate_counts > 0])),
                       p95=float(np.percentile(candidate_counts[candidate_counts > 0], 95)),
                       maximum=int(candidate_counts.max())))
    return ids, dict(stages_ms={k: v / 1e6 for k, v in stages.items()},
                     total_ms=total / 1e6, counts=summary)


def coarse_neighbors(points, seeds, config, batch_size=512):
    """Time the entire per-center loop once per batch to minimize probe cost."""
    stages = dict.fromkeys(("build_tree", "batch_knn", "batch_height_filter",
                           "per_seed_refinement"), 0)
    total_start = clock()
    start = clock()
    tree = cKDTree(points)
    stages["build_tree"] += clock() - start
    ids = np.full((len(seeds), config.neighbors), -1, dtype=np.int32)
    for start_index in range(0, len(seeds), batch_size):
        pending = np.arange(start_index, min(start_index + batch_size, len(seeds)))
        k = min(64, len(points))
        while len(pending):
            start = clock()
            distance, nearest = tree.query(seeds[pending], k=k,
                                          distance_upper_bound=config.radius_m + 1e-12)
            stages["batch_knn"] += clock() - start
            start = clock()
            if nearest.ndim == 1:
                nearest, distance = nearest[:, None], distance[:, None]
            exists = nearest < len(points)
            safe = np.minimum(nearest, len(points) - 1)
            eligible = exists & (np.abs(points[safe, 2] - seeds[pending, None, 2])
                                 <= config.seed_height_gate_m)
            enough = eligible.sum(axis=1) >= config.neighbors
            complete = ~exists[:, -1] | (k == len(points))
            stages["batch_height_filter"] += clock() - start
            start = clock()
            for row in np.flatnonzero(enough):
                seed = pending[row]
                radius = distance[row, np.flatnonzero(eligible[row])[config.neighbors - 1]]
                candidates = np.asarray(tree.query_ball_point(
                    seeds[seed], min(config.radius_m + 1e-12, radius + 1e-10)),
                    dtype=np.int32)
                delta = points[candidates] - seeds[seed]
                d2 = np.einsum("ni,ni->n", delta, delta)
                take = ((np.abs(delta[:, 2]) <= config.seed_height_gate_m)
                        & (d2 <= config.radius_m**2 + 1e-14))
                candidates, d2 = candidates[take], d2[take]
                ids[seed] = candidates[np.lexsort((candidates, d2))[:config.neighbors]]
            stages["per_seed_refinement"] += clock() - start
            pending = pending[~enough & ~complete]
            k = min(len(points), k * 2)
    total = clock() - total_start
    stages["allocation_loop_and_instrumentation"] = total - sum(stages.values())
    return ids, dict(stages_ms={k: v / 1e6 for k, v in stages.items()}, total_ms=total / 1e6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=9)
    parser.add_argument("--warmups", type=int, default=2)
    args = parser.parse_args()
    if args.repeats < 3 or args.warmups < 1:
        parser.error("Use at least 3 repeats and 1 warmup")
    directory = OUT / "hill"
    reference_path = directory / "results.npz"
    before_sha = file_sha(reference_path)
    raw, source = source_cloud("hill")
    points = raw.astype(float)
    with np.load(reference_path, allow_pickle=False) as saved:
        seeds = points[saved["source_ids"]]
        expected = saved["neighbor_ids"].copy()
        np.testing.assert_array_equal(seeds, saved["points"])
    config = LocalConfig()
    for _ in range(args.warmups):
        np.testing.assert_array_equal(exact_neighbors(points, seeds, config), expected)
        np.testing.assert_array_equal(coarse_neighbors(points, seeds, config)[0], expected)
    baseline, profiles = [], []
    # Alternate order to reduce drift; correctness checks are outside timers.
    for repetition in range(args.repeats):
        order = ("baseline", "instrumented") if repetition % 2 == 0 else ("instrumented", "baseline")
        for name in order:
            if name == "baseline":
                start = clock()
                ids = exact_neighbors(points, seeds, config)
                elapsed = (clock() - start) / 1e6
                baseline.append(elapsed)
            else:
                ids, profile = coarse_neighbors(points, seeds, config)
                profiles.append(profile)
            np.testing.assert_array_equal(ids, expected)
    detailed = []
    for _ in range(3):
        ids, profile = detailed_neighbors(points, seeds, config)
        np.testing.assert_array_equal(ids, expected)
        detailed.append(profile)
    assert file_sha(reference_path) == before_sha
    representative = sorted(profiles, key=lambda row: row["total_ms"])[len(profiles) // 2]
    report = dict(
        scene="hill", raw_points=len(points), centers=len(seeds),
        repeats=args.repeats, warmups=args.warmups, environment=environment(), source=source,
        baseline_samples_ms=baseline, baseline_median_ms=float(np.median(baseline)),
        representative_instrumented_run=representative,
        instrumented_runs=profiles,
        detailed_instrumentation=dict(
            representative_run=sorted(detailed, key=lambda row: row["total_ms"])[1],
            runs=detailed,
            caution="Per-center timers significantly perturb runtime; use coarse timings for attribution, detailed runs mainly for deterministic operation counts."),
        methodology="Representative coarse breakdown is the run with median instrumented total; stages add to that run. Baseline times the unchanged production function, with alternating order. These are fresh measurements, not a retroactive split of the old 280.6 ms.",
        checks=dict(all_measured_neighbors_equal_published=True,
                    published_result_unchanged=True, result_sha256=before_sha),
    )
    output = directory / "neighbor-profile.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("instrumented_runs", "environment", "detailed_instrumentation")},
                     ensure_ascii=False, indent=2))
    print(json.dumps(report["detailed_instrumentation"]["representative_run"]["counts"], indent=2))
    print("Saved:", output)


if __name__ == "__main__":
    main()
