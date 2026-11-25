"""Configurable local polynomial assessment; engineering costs, not dynamics claims.

All calculations consume XYZ only. Units are metres/radians unless named.
Missing fits/unsupported directions remain NaN rather than zero risk.
"""
from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from scipy.integrate import quad
from scipy.spatial import cKDTree, ConvexHull, QhullError


@dataclass(frozen=True)
class AssessmentConfig:
    patch_size_m: float = .20
    seed_voxel_m: float = .10
    height_gate_m: float = .04
    minimum_points: int = 30
    noise_floor_m: float = .003
    max_rmse_m: float = .02
    error_uncertainty_limit_m: float = .02
    clearance_low_m: float = .10
    clearance_high_m: float = .60
    clearance_cell_m: float = .05
    jump_offset_m: float = .24
    jump_limit_m: float = .12
    sigma_multiplier: float = 2.
    slope_limit_deg: float = 45.
    roughness_limit_m: float = .01
    normal_change_limit_deg: float = 15.
    body_length_m: float = .80
    body_width_m: float = .50
    com_height_m: float = .30
    leg_height_residual_limit_m: float = .15
    body_minimum_coverage: float = .80
    center_offset_check: bool = True


def basis(xy):
    u, v = np.asarray(xy)[..., 0], np.asarray(xy)[..., 1]
    return np.stack((np.ones_like(u), u, v, u*u, u*v, v*v), axis=-1)


def evaluate(seeds, coefficients, ids, xy):
    ids = np.asarray(ids)
    phi = basis(np.asarray(xy)-seeds[ids, :2])
    return seeds[ids, 2]+np.sum(phi*coefficients[ids], axis=-1)


def variance(seeds, covariance, ids, xy):
    phi = basis(np.asarray(xy)-seeds[np.asarray(ids), :2])
    return np.maximum(0, np.einsum('...i,...ij,...j->...', phi, covariance[ids], phi))


def window_pools(points, seeds, config=AssessmentConfig()):
    scale = np.array([config.patch_size_m/2, config.patch_size_m/2, config.height_gate_m])
    tree = cKDTree(points/scale)
    # A scaled infinity-norm ball is the exact rectangular XYZ gate.
    pools = tree.query_ball_point(seeds/scale, 1.+1e-7, p=np.inf, return_sorted=True)
    return [np.asarray(p, dtype=np.int32) for p in pools]


def fit_pools(points, seeds, pools, config=AssessmentConfig()):
    n = len(seeds)
    result = dict(coefficient=np.full((n, 6), np.nan), height=np.full(n, np.nan),
                  rmse=np.full(n, np.nan), max_error=np.full(n, np.nan),
                  covariance=np.full((n, 6, 6), np.nan), noise_variance=np.full(n, np.nan),
                  counts=np.array([len(p) for p in pools], dtype=np.int32),
                  full_rank=np.zeros(n, bool), normal=np.full((n, 3), np.nan))
    active = np.flatnonzero(result['counts'] >= config.minimum_points)
    buckets = {}
    for i in active:
        buckets.setdefault(1 << int(np.ceil(np.log2(len(pools[i])))), []).append(i)
    for size, indices in buckets.items():
        batch_size = max(16, min(256, 65536//size))
        for start in range(0, len(indices), batch_size):
            ids = np.asarray(indices[start:start+batch_size])
            raw = np.zeros((len(ids), size), dtype=np.int32)
            weight = np.zeros(raw.shape)
            for j, i in enumerate(ids):
                raw[j, :len(pools[i])] = pools[i]
                weight[j, :len(pools[i])] = 1
            delta = points[raw]-seeds[ids, None, :]
            counts = result['counts'][ids]
            scale = np.maximum(np.sqrt(np.sum(np.sum(delta[:, :, :2]**2, axis=2)*weight, axis=1)/counts), 1e-8)
            a = basis(delta[:, :, :2]/scale[:, None, None])*weight[:, :, None]
            target = delta[:, :, 2]*weight
            gram = np.einsum('nki,nkj->nij', a, a)
            eig = np.linalg.eigvalsh(gram)
            rank = eig[:, 0] > np.maximum(eig[:, -1], 1e-30)*1e-10
            ridge = 1e-10*np.trace(gram, axis1=1, axis2=2)/6
            inverse = np.linalg.inv(gram+ridge[:, None, None]*np.eye(6))
            theta = np.einsum('nij,nj->ni', inverse, np.einsum('nki,nk->ni', a, target))
            residual = (np.einsum('nki,ni->nk', a, theta)-target)*weight
            rss = np.sum(residual**2, axis=1)
            rmse = np.sqrt(rss/counts)
            transform = 1/np.column_stack((np.ones(len(ids)), scale, scale, scale**2, scale**2, scale**2))
            coefficient = theta*transform
            # Finite-ridge sandwich covariance; residual df n-6 (ridge is 1e-10).
            noise = np.maximum(rss/(counts-6), config.noise_floor_m**2)
            cov = noise[:, None, None]*(inverse @ gram @ inverse)
            cov *= transform[:, :, None]*transform[:, None, :]
            cov = (cov+cov.transpose(0, 2, 1))/2
            center_ok = np.abs(coefficient[:, 0]) <= config.height_gate_m if config.center_offset_check else np.ones(len(ids), bool)
            valid = rank & center_ok & (rmse <= config.max_rmse_m)
            result['coefficient'][ids] = coefficient
            result['covariance'][ids] = cov
            result['noise_variance'][ids] = noise
            result['rmse'][ids] = rmse
            result['max_error'][ids] = np.max(np.abs(residual), axis=1)
            result['full_rank'][ids] = rank
            result['height'][ids[valid]] = seeds[ids[valid], 2]+coefficient[valid, 0]
    normal = np.column_stack((-result['coefficient'][:, 1:3], np.ones(n)))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    result['normal'] = normal
    return result


def patches(points, seeds, pools, fits, config=AssessmentConfig()):
    valid = np.flatnonzero(np.isfinite(fits['height']))
    axis = np.linspace(-config.patch_size_m/2, config.patch_size_m/2, 5)
    x, y = np.meshgrid(axis, axis)
    uv = np.column_stack((x.ravel(), y.ravel()))
    a = (np.arange(4)[:, None]*5+np.arange(4)).ravel()
    candidates = np.r_[np.column_stack((a, a+1, a+5)), np.column_stack((a+1, a+6, a+5))]
    positions, faces, owners = [], [], []
    std_max = np.full(len(seeds), np.nan)
    for i in valid:
        local = points[pools[i], :2]-seeds[i, :2]
        try:
            hull = ConvexHull(local)
        except QhullError:
            continue
        z = evaluate(seeds, fits['coefficient'], np.full(25, i), uv+seeds[i, :2])
        inside = np.all(uv @ hull.equations[:, :2].T+hull.equations[:, 2] <= 1e-7, axis=1)
        inside &= np.abs(z-seeds[i, 2]) <= config.height_gate_m+1e-7
        keep = candidates[np.all(inside[candidates], axis=1)]
        vertex_std = np.sqrt(variance(seeds, fits['covariance'], np.full(25, i), uv+seeds[i, :2]))
        std_max[i] = float(np.max(vertex_std[inside])) if np.any(inside) else np.sqrt(fits['covariance'][i, 0, 0])
        offset = len(positions)*25
        positions.append(np.column_stack((uv+seeds[i, :2], z)))
        faces.append(keep+offset)
        owners.append(np.full(25, i, dtype=np.uint32))
    return (np.asarray(positions, dtype='<f4').reshape(-1, 3),
            np.concatenate(faces).astype('<u4') if faces else np.empty((0, 3), dtype='<u4'),
            np.concatenate(owners).astype('<u4') if owners else np.empty(0, dtype='<u4'), std_max)


def candidate_pairs(seeds, fits, config=AssessmentConfig()):
    active = np.flatnonzero(np.isfinite(fits['height']))
    if len(active) < 2:
        return np.empty((0, 2), dtype=np.int32)
    tree = cKDTree(seeds[active, :2])
    pairs = []
    for direction in np.array([[1., 0], [-1., 0], [0, 1.], [0, -1.]]):
        target = seeds[active, :2]+config.jump_offset_m*direction
        dist, ix = tree.query(target, k=16, distance_upper_bound=.105)
        safe = active[np.minimum(ix, len(active)-1)]
        pred = evaluate(seeds, fits['coefficient'], active, target)
        cost = dist**2+.5*(fits['height'][safe]-pred[:, None])**2
        separation = np.linalg.norm(seeds[safe, :2]-seeds[active, None, :2], axis=2)
        cost[(ix >= len(active)) | (separation < .14)] = np.inf
        selected = np.argmin(cost, axis=1)
        keep = np.isfinite(cost[np.arange(len(active)), selected])
        pairs.append(np.column_stack((active[keep], safe[np.arange(len(active)), selected][keep])))
    return np.unique(np.sort(np.concatenate(pairs), axis=1), axis=0).astype(np.int32)


def jump_metrics(points, seeds, pools, fits, config=AssessmentConfig()):
    pairs = candidate_pairs(seeds, fits, config)
    n = len(seeds)
    columns = ['one_sided_m', 'bilateral_m', 'jump_sigma_m', 'significance', 'upper_m']
    output = {name:np.full(n, np.nan) for name in columns}
    records = np.full((len(pairs), 8), np.nan)
    known_directions = np.zeros(n, dtype=np.int32)
    side_solves = 0
    for start in range(0, len(pairs), 2048):
        edge = pairs[start:start+2048]
        left, right = edge.T
        midpoint = (seeds[left, :2]+seeds[right, :2])/2
        direction = seeds[right, :2]-seeds[left, :2]
        direction /= np.linalg.norm(direction, axis=1)[:, None]
        # A: each existing one-sided polynomial predicts the other anchor.
        a = np.maximum(np.abs(evaluate(seeds, fits['coefficient'], left, seeds[right, :2])-fits['height'][right]),
                       np.abs(evaluate(seeds, fits['coefficient'], right, seeds[left, :2])-fits['height'][left]))
        left_pool, right_pool = [], []
        for j, (i, k) in enumerate(edge):
            li, ri = pools[i], pools[k]
            li = li[np.sum((points[li, :2]-midpoint[j])*direction[j], axis=1) < -1e-8]
            ri = ri[np.sum((points[ri, :2]-midpoint[j])*direction[j], axis=1) > 1e-8]
            assert not np.intersect1d(li, ri).size
            left_pool.append(li);right_pool.append(ri)
        local_seeds = np.concatenate((seeds[left], seeds[right]))
        side = fit_pools(points, local_seeds, left_pool+right_pool, config)
        side_solves += int(np.isfinite(side['rmse']).sum())
        m = len(edge)
        li, ri = np.arange(m), np.arange(m)+m
        signed = evaluate(local_seeds, side['coefficient'], ri, midpoint)-evaluate(local_seeds, side['coefficient'], li, midpoint)
        sigma = np.sqrt(variance(local_seeds, side['covariance'], li, midpoint)
                        +variance(local_seeds, side['covariance'], ri, midpoint))
        valid = np.isfinite(side['height'][li]) & np.isfinite(side['height'][ri])
        b = np.where(valid, np.abs(signed), np.nan)
        sigma[~valid] = np.nan
        significance = b/np.sqrt(sigma**2+1e-12)
        upper = b+config.sigma_multiplier*sigma
        rows = np.column_stack((left, right, midpoint, a, b, sigma, upper))
        records[start:start+m] = rows
        for name, values in zip(columns, (a, b, sigma, significance, upper)):
            dest = output[name]
            for anchors in (left, right):
                # fmax preserves a known value when another direction is unknown.
                for i, value in zip(anchors, values):
                    dest[i] = np.fmax(dest[i], value)
        for anchors in (left[valid], right[valid]):
            np.add.at(known_directions, anchors, 1)
    output['known_directions'] = known_directions
    output['pairs'] = records
    output['solve_counts'] = dict(candidate_pairs=len(pairs), side_windows=2*len(pairs),
        actual_side_solves=side_solves, both_sides_valid_pairs=int(np.isfinite(records[:, 5]).sum()))
    return output


@lru_cache(maxsize=16)
def circular_column_areas(radius, cell_size):
    """Areas of square columns clipped to a circle, indexed x + cells*y."""
    cells = int(np.ceil(2*radius/cell_size-1e-12))
    edges = np.minimum(-radius+np.arange(cells+1)*cell_size, radius)
    areas = np.zeros(cells*cells)
    for iy in range(cells):
        y0, y1 = edges[iy:iy+2]
        breaks = []
        for yy in (y0, y1):
            if abs(yy) < radius:
                xx = np.sqrt(radius*radius-yy*yy)
                breaks.extend((-xx, xx))
        for ix in range(cells):
            x0, x1 = edges[ix:ix+2]
            def height(xx):
                bound = np.sqrt(max(0., radius*radius-xx*xx))
                return max(0., min(y1, bound)-max(y0, -bound))
            cuts = sorted({x0, x1, *(x for x in breaks if x0 < x < x1)})
            areas[ix+cells*iy] = sum(quad(height, a, b, epsabs=1e-13, epsrel=1e-11)[0]
                                     for a, b in zip(cuts[:-1], cuts[1:]))
    areas.setflags(write=False)
    return areas


def clearance_metrics(points, seeds, fits, patch_positions, patch_faces, config=AssessmentConfig()):
    """Sampled clearance evidence from raw returns and actual fitted surfaces."""
    active = np.flatnonzero(np.isfinite(fits['height']))
    circular = getattr(config, 'domain_shape', 'square') == 'circle'
    half = config.radius_m if circular else config.patch_size_m/2
    cells = int(np.ceil(2*half/config.clearance_cell_m-1e-12)) if circular else int(round(config.patch_size_m/config.clearance_cell_m))
    if cells < 1 or (not circular and not np.isclose(cells*config.clearance_cell_m, config.patch_size_m)):
        raise ValueError('Patch width must be an integer multiple of clearance cell size')
    areas = circular_column_areas(half, config.clearance_cell_m) if circular else None
    occupied_raw = np.zeros((len(seeds), cells*cells), bool)
    occupied_surface = np.zeros((len(seeds), cells*cells), bool)
    counts = np.zeros((len(seeds), 2), dtype=np.int32)
    # Only vertices used by a real fitted triangle are valid surface samples.
    samples = patch_positions[np.unique(patch_faces)].astype(float) if len(patch_faces) else np.empty((0, 3))
    for source_id, cloud in enumerate((points, samples)):
        if not len(cloud):
            continue
        tree = cKDTree(cloud[:, :2])
        dest = occupied_raw if source_id == 0 else occupied_surface
        for start in range(0, len(active), 256):
            ids = active[start:start+256]
            nearby = tree.query_ball_point(seeds[ids, :2], half+1e-7, p=2 if circular else np.inf, return_sorted=True)
            for i, neighbors in zip(ids, nearby):
                if not neighbors:
                    continue
                p = cloud[neighbors]
                z = evaluate(seeds, fits['coefficient'], np.full(len(p), i), p[:, :2])
                dz = p[:, 2]-z
                keep = (dz >= config.clearance_low_m-1e-7) & (dz <= config.clearance_high_m+1e-7)
                counts[i, source_id] = int(keep.sum())
                cell = np.floor((p[keep, :2]-seeds[i, :2]+half)/config.clearance_cell_m).astype(int)
                cell = np.clip(cell, 0, cells-1)
                dest[i, cell[:, 0]+cells*cell[:, 1]] = True
    occupied = occupied_raw | occupied_surface
    cost = occupied @ (areas/areas.sum()) if circular else np.mean(occupied, axis=1)
    cost[~np.isfinite(fits['height'])] = np.nan
    return dict(cost=cost, raw_hits=counts[:, 0], surface_hits=counts[:, 1],
                raw_occupied=occupied_raw, surface_occupied=occupied_surface,
                column_areas_m2=areas if circular else np.full(cells*cells,config.clearance_cell_m**2))


def local_metrics(seeds, fits, std_max, jumps, clearance, config=AssessmentConfig()):
    c = fits['coefficient']
    valid = np.isfinite(fits['height'])
    slope = np.rad2deg(np.arctan(np.linalg.norm(c[:, 1:3], axis=1)))
    change = np.zeros(len(seeds))
    half = config.patch_size_m/2
    if getattr(config, 'domain_shape', 'square') == 'circle':
        theta = np.arange(24)*2*np.pi/24
        offsets = config.radius_m*np.c_[np.cos(theta),np.sin(theta)]
    else:
        offsets = ((-half, -half), (-half, half), (half, -half), (half, half))
    for u, v in offsets:
        normal = np.column_stack((-(c[:, 1]+2*c[:, 3]*u+c[:, 4]*v),
                                   -(c[:, 2]+c[:, 4]*u+2*c[:, 5]*v), np.ones(len(c))))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        angle = np.rad2deg(np.arccos(np.clip(np.sum(normal*fits['normal'], axis=1), -1, 1)))
        change = np.maximum(change, angle)
    rough = np.maximum(np.clip(fits['rmse']/config.roughness_limit_m, 0, 1),
                       np.clip(change/config.normal_change_limit_deg, 0, 1))
    slope_cost = np.clip(slope/config.slope_limit_deg, 0, 1)
    jump_cost = np.clip(jumps['upper_m']/config.jump_limit_m, 0, 1)
    uncertainty = np.clip((fits['max_error']+config.sigma_multiplier*std_max)/config.error_uncertainty_limit_m, 0, 1)
    cost = .25*slope_cost+.20*rough+.25*jump_cost+.20*clearance['cost']+.10*uncertainty
    cost = np.where(slope >= config.slope_limit_deg, 1., cost)
    for values in (slope, change, rough, slope_cost, uncertainty, cost):
        values[~valid] = np.nan
    return dict(slope_deg=slope, normal_change_deg=change, roughness_cost=rough,
                slope_cost=slope_cost, error_uncertainty_cost=uncertainty, foot_cost=cost)


def choose_support(seeds, fits, query_xy, expected_z, max_candidates=16, config=AssessmentConfig()):
    active = np.flatnonzero(np.isfinite(fits['height']))
    result = np.full(query_xy.shape[:-1], -1, dtype=np.int32)
    height = np.full(result.shape, np.nan)
    if not len(active):
        return result, height
    tree = cKDTree(seeds[active, :2])
    flat, target = query_xy.reshape(-1, 2), expected_z.ravel()
    half = config.patch_size_m/2
    distance, index = tree.query(flat, k=max_candidates, distance_upper_bound=np.sqrt(2)*half+1e-8)
    safe = active[np.minimum(index, len(active)-1)]
    offset = flat[:, None, :]-seeds[safe, :2]
    prediction = evaluate(seeds, fits['coefficient'], safe, np.broadcast_to(flat[:, None, :], offset.shape))
    eligible = (index < len(active)) & np.all(np.abs(offset) <= half+1e-7, axis=2)
    # A query must be inside the fitted height band, not an unchecked long extrapolation.
    eligible &= np.abs(prediction-seeds[safe, 2]) <= config.height_gate_m+1e-7
    cost = distance**2+.5*(prediction-target[:, None])**2
    cost[~eligible] = np.inf
    winner = np.argmin(cost, axis=1)
    known = np.isfinite(cost[np.arange(len(flat)), winner])
    result.ravel()[known] = safe[np.arange(len(flat)), winner][known]
    height.ravel()[known] = prediction[np.arange(len(flat)), winner][known]
    return result, height


def body_metrics(seeds, fits, metrics, jumps, clearance, config=AssessmentConfig(), *, support_query=None):
    n = len(seeds)
    columns = ['risk', 'coverage', 'slope_deg', 'plane_rmse_m', 'support_margin_m',
               'max_jump_upper_m', 'max_clearance_cost', 'max_roughness_cost', 'max_uncertainty_cost']
    result = {k:np.full(n, np.nan) for k in columns}
    contacts = np.full((n, 4), -1, dtype=np.int32)
    contact_z = np.full((n, 4), np.nan)
    foot_xy = np.array([[-.35, -.20], [.35, -.20], [.35, .20], [-.35, .20]])
    x, y = np.meshgrid(np.linspace(-.4, .4, 5), np.linspace(-.25, .25, 5))
    grid_xy = np.column_stack((x.ravel(), y.ravel()))
    design = np.column_stack((np.ones(4), foot_xy))
    pinv = np.linalg.pinv(design)
    active = np.flatnonzero(np.isfinite(fits['height']))
    for start in range(0, len(active), 512):
        ids = active[start:start+512]
        offsets = np.r_[foot_xy, grid_xy]
        query = seeds[ids, None, :2]+offsets
        expected = fits['height'][ids, None]+np.sum(fits['coefficient'][ids, None, 1:3]*offsets, axis=2)
        if support_query is None:
            support, z = choose_support(seeds, fits, query, expected, config=config)
        else:
            support, z = support_query(query, expected)
        contacts[ids], contact_z[ids] = support[:, :4], z[:, :4]
        coverage = np.mean(support[:, 4:] >= 0, axis=1)
        result['coverage'][ids] = coverage
        foot_known = np.all(support[:, :4] >= 0, axis=1)
        good = foot_known & (coverage >= config.body_minimum_coverage)
        eligible_ids = ids[good]
        if not len(eligible_ids):
            continue
        support, zz = support[good], z[good, :4]
        plane = zz @ pinv.T
        residual = zz-plane @ design.T
        rmse = np.sqrt(np.mean(residual**2, axis=1))
        gx, gy = plane[:, 1], plane[:, 2]
        slope = np.rad2deg(np.arctan(np.hypot(gx, gy)))
        com_x = -config.com_height_m*gx/np.sqrt(1+gx*gx+gy*gy)
        com_y = -config.com_height_m*gy/np.sqrt(1+gx*gx+gy*gy)
        margin = np.minimum(.35-np.abs(com_x), .20-np.abs(com_y))
        safe = np.maximum(support, 0)
        known = support >= 0
        def maximum(values):
            a = values[safe].copy();a[~known] = np.nan
            finite = np.isfinite(a)
            maximum = np.max(np.where(finite, a, -np.inf), axis=1)
            maximum[~finite.any(axis=1)] = np.nan
            return maximum
        jump = maximum(jumps['upper_m'])
        clear = maximum(clearance['cost'])
        rough = maximum(metrics['roughness_cost'])
        uncertain = maximum(metrics['error_uncertainty_cost'])
        support_cost = np.clip((.10-margin)/.10, 0, 1)
        cost = (.20*np.clip(slope/config.slope_limit_deg, 0, 1)
                +.15*np.clip(rmse/.02, 0, 1)+.25*np.clip(jump/config.jump_limit_m, 0, 1)
                +.20*clear+.10*rough+.05*uncertain+.05*support_cost)
        hard = (slope >= config.slope_limit_deg) | (margin <= 0) | (np.max(np.abs(residual), axis=1) > config.leg_height_residual_limit_m)
        cost[hard] = 1.
        for key, value in zip(columns[0:1]+columns[2:], (cost, slope, rmse, margin, jump, clear, rough, uncertain)):
            result[key][eligible_ids] = value
    result.update(contact_ids=contacts, contact_z=contact_z, foot_offsets=foot_xy)
    return result
