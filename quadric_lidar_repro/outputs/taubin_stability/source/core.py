"""Paper-equation quadric surfaces, incremental moments and SE(3) registration.

Conventions: q=[x²,y²,z²,xy,yz,xz,x,y,z,1]; perturbations=[tx,ty,tz,rx,ry,rz].
Eq. (2)/(3) are squared-distance residuals, so LM minimizes their squares,
literally matching the paper's Jacobians (a quartic distance objective).
"""

from dataclasses import dataclass, asdict
import numpy as np
from scipy.linalg import eigh
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Config:
    alpha: float = 1.0
    beta: float = .1
    gamma: float = 1.9
    mse_threshold: float = .04
    planar_ratio: float = .01
    covariance_floor: float = .0001
    min_patch: int = 20
    mapping_min_patch: int = 50
    point_budget: int = 4000
    points_per_patch: int = 160
    association_samples: int = 48
    association_distance: float = 8.0
    max_residual_distance: float = 3.0
    outer_iterations: int = 3
    lm_iterations: int = 8
    max_range: float = 60.0
    normal_angle_deg: float = 35.0
    segmentation_angle_deg: float = 10.0
    map_radius: float = 70.0
    hash_resolution: float = 1.5
    association_normal_deg: float = 45.0
    robust_scale: float = 1.0
    seed: int = 7

    def as_dict(self):
        return asdict(self)


def basis(points):
    p = np.atleast_2d(points)
    x, y, z = p.T
    return np.column_stack((x*x, y*y, z*z, x*y, y*z, x*z, x, y, z, np.ones(len(p))))


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def se3_exp(xi):
    translation, omega = np.asarray(xi[:3]), np.asarray(xi[3:])
    theta = np.linalg.norm(omega)
    w = skew(omega)
    if theta < 1e-7:
        v = np.eye(3)+.5*w+w@w/6
    else:
        v = np.eye(3)+(1-np.cos(theta))/theta**2*w+(theta-np.sin(theta))/theta**3*(w@w)
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(omega).as_matrix()
    t[:3, 3] = v @ translation
    return t


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def polynomial_transform(rotation, translation):
    """Exact L such that q(Rp+t)=L q(p), including all cross terms."""
    r, t = rotation, translation
    result = np.zeros((10, 10))
    for out, (a, b) in enumerate(((0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (0, 2))):
        result[out, :3] = r[a]*r[b]
        result[out, 3:6] = [r[a, 0]*r[b, 1]+r[a, 1]*r[b, 0],
                            r[a, 1]*r[b, 2]+r[a, 2]*r[b, 1],
                            r[a, 0]*r[b, 2]+r[a, 2]*r[b, 0]]
        result[out, 6:9] = t[a]*r[b]+t[b]*r[a]
        result[out, 9] = t[a]*t[b]
    result[6:9, 6:9], result[6:9, 9], result[9, 9] = r, t, 1
    return result


DERIVATIVE = np.zeros((3, 10, 10))
for axis, terms in enumerate((((0, 6, 2), (3, 7, 1), (5, 8, 1), (6, 9, 1)),
                              ((1, 7, 2), (3, 6, 1), (4, 8, 1), (7, 9, 1)),
                              ((2, 8, 2), (4, 7, 1), (5, 6, 1), (8, 9, 1)))):
    for out, col, value in terms:
        DERIVATIVE[axis, out, col] = value


@dataclass
class Moments:
    origin: np.ndarray
    gram: np.ndarray
    count: int

    @classmethod
    def from_points(cls, points):
        origin = np.mean(points, axis=0)
        q = basis(points-origin)
        return cls(origin, q.T@q, len(points))

    @property
    def mean(self):
        return self.origin+self.gram[6:9, 9]/self.count

    @property
    def covariance(self):
        mu = self.gram[6:9, 9]/self.count
        return (self.gram[6:9, 6:9]/self.count-np.outer(mu, mu))

    def transformed(self, pose):
        origin = transform(self.origin[None, :], pose)[0]
        l = polynomial_transform(pose[:3, :3], np.zeros(3))
        return Moments(origin, l@self.gram@l.T, self.count)

    def merge(self, other):
        # The augmented moment Gram stores exactly the means and S/P/Q of
        # Eqs. (8)/(9). No past points are retained or revisited.
        l = polynomial_transform(np.eye(3), other.origin-self.origin)
        self.gram += l@other.gram@l.T
        self.gram = .5*(self.gram+self.gram.T)
        self.count += other.count


@dataclass
class Patch:
    moments: Moments
    coefficients: np.ndarray
    mse: float
    kind: str
    inverse_covariance: np.ndarray
    radius: float

    @property
    def center(self):
        return self.moments.mean

    @property
    def count(self):
        return self.moments.count


def fit_patch(moments, config):
    covariance = moments.covariance
    eigenvalues, vectors = np.linalg.eigh(.5*(covariance+covariance.T))
    eigenvalues = np.maximum(eigenvalues, 0)
    inverse = (vectors/np.maximum(eigenvalues, config.covariance_floor))@vectors.T
    mu = moments.mean-moments.origin
    centered = polynomial_transform(np.eye(3), -mu)
    gram = centered@moments.gram@centered.T/moments.count
    gram = .5*(gram+gram.T)
    coeff = np.zeros(10)
    kind = "plane"
    if eigenvalues[0] <= config.planar_ratio*max(eigenvalues[1], 1e-12):
        coeff[6:9] = vectors[:, 0]
        # Plane is fitted about the mean; coefficients below are returned in anchor coordinates.
    else:
        kind = "quadric"
        mean = gram[:9, 9]
        c = gram[:9, :9]-np.outer(mean, mean)
        g = sum(d@gram@d.T for d in DERIVATIVE)[:9, :9]
        try:
            _, solutions = eigh(.5*(c+c.T), .5*(g+g.T)+np.eye(9)*1e-9, check_finite=False)
            coeff[:9] = solutions[:, 0]
            coeff[9] = -mean@coeff[:9]
            normalization = np.sqrt(max(coeff[:9]@g@coeff[:9], 1e-30))
            coeff /= normalization
        except np.linalg.LinAlgError:
            coeff[:] = 0
            kind = "distribution"
    mse = max(float(coeff@gram@coeff), 0)
    if not np.isfinite(mse) or mse > config.mse_threshold or np.linalg.norm(coeff) < 1e-10:
        kind, coeff = "distribution", np.zeros(10)
    coeff = centered.T@coeff
    return Patch(moments, coeff, mse, kind, inverse, float(np.sqrt(eigenvalues[-1])))


def quadric_matrix(c):
    return np.array([[c[0], c[3]/2, c[5]/2], [c[3]/2, c[1], c[4]/2],
                     [c[5]/2, c[4]/2, c[2]]])


def distance_and_gradient(patch, points):
    """Eqs. (1)--(6): squared residual and its derivative wrt target points."""
    if patch.kind == "distribution":
        delta = points-patch.center
        weighted = delta@patch.inverse_covariance
        return np.sum(delta*weighted, axis=1), 2*weighted
    local = points-patch.moments.origin
    c = patch.coefficients
    a = quadric_matrix(c)
    f = np.einsum("ni,ij,nj->n", local, a, local)+local@c[6:9]+c[9]
    g = 2*local@a+c[6:9]
    h = np.maximum(np.sum(g*g, axis=1), 1e-12)
    residual = f*f/h
    gradient = 2*(f/h)[:, None]*g-4*(f*f/h**2)[:, None]*(g@a)
    return residual, gradient


def pose_jacobian(points, gradient):
    # gradient @ [I, -[p]x] == [gradient, p cross gradient]
    return np.column_stack((gradient, np.cross(points, gradient)))


def subset(points, count):
    if len(points) <= count:
        return points
    return points[np.linspace(0, len(points)-1, count, dtype=int)]


class PatchBank:
    """Vectorized patch-wise traversal; no nearest-neighbor point index."""
    def __init__(self, patches):
        self.patches = patches
        self.centers = np.array([p.center for p in patches]).reshape(-1, 3)
        self.origins = np.array([p.moments.origin for p in patches]).reshape(-1, 3)
        self.radii = np.array([p.radius for p in patches])
        self.inverse = np.array([p.inverse_covariance for p in patches]).reshape(-1, 3, 3)
        self.coefficients = np.array([p.coefficients for p in patches]).reshape(-1, 10)
        self.a = np.array([quadric_matrix(p.coefficients) for p in patches]).reshape(-1, 3, 3)
        self.distribution = np.array([p.kind=="distribution" for p in patches], dtype=bool)


def associate(source_points, target, config):
    """Eq. (7), with a deterministic sample and a documented center gate."""
    center = source_points.mean(axis=0)
    bank = target if isinstance(target, PatchBank) else PatchBank(target)
    if not len(bank.patches):
        return None
    p = subset(source_points, config.association_samples)
    distances = np.linalg.norm(center-bank.centers, axis=1)
    ids = np.flatnonzero(distances <= config.association_distance+2*bank.radii)
    if not len(ids):
        return None
    delta = p[None, :, :]-bank.centers[ids, None, :]
    mahalanobis = np.einsum("mni,mij,mnj->mn", delta, bank.inverse[ids], delta)
    local = p[None, :, :]-bank.origins[ids, None, :]
    c, a = bank.coefficients[ids], bank.a[ids]
    f = np.einsum("mni,mij,mnj->mn", local, a, local)+np.einsum("mni,mi->mn", local, c[:, 6:9])+c[:, 9, None]
    gradient = 2*np.einsum("mni,mij->mnj", local, a)+c[:, None, 6:9]
    d = f*f/np.maximum(np.sum(gradient*gradient, axis=2), 1e-12)
    d[bank.distribution[ids]] = mahalanobis[bank.distribution[ids]]
    weights = config.beta+config.gamma*np.exp(-np.minimum(mahalanobis, 1400))
    scores = np.mean(config.alpha*d/weights, axis=1)
    # Explicit stability extension: nearly planar source samples must agree in
    # normal direction with the matched surface, not merely its infinite extension.
    values, vectors = np.linalg.eigh(np.cov(p, rowvar=False)) if len(p)>=3 else (np.ones(3),np.eye(3))
    if len(p)>=3 and config.association_normal_deg < 90 and values[0] < .1*max(values[1], 1e-12):
        center_local = center-bank.origins[ids]
        n = 2*np.einsum('mi,mij->mj', center_local, a)+c[:, 6:9]
        norm = np.linalg.norm(n, axis=1)
        agreement = np.abs(n@vectors[:, 0])/np.maximum(norm, 1e-12)
        incompatible = (agreement < np.cos(np.radians(config.association_normal_deg))) & ~bank.distribution[ids] & (norm > 1e-8)
        scores[incompatible] = np.inf
    if not np.isfinite(scores).any():
        return None
    candidates = np.flatnonzero(scores <= scores.min()+1e-12)
    return int(ids[candidates[np.argmin(distances[ids[candidates]])]])


def select_source(patches, point_lists, config):
    order = sorted(range(len(patches)), key=lambda i: patches[i].count*np.linalg.norm(patches[i].center), reverse=True)
    selected, total = [], 0
    for index in order:
        if config.point_budget-total < 3:
            break
        p = subset(point_lists[index], min(config.points_per_patch, config.point_budget-total))
        if len(p):
            selected.append((index, p))
            total += len(p)
        if total >= config.point_budget:
            break
    return selected


def register(patches, point_lists, targets, initial, config):
    pose = initial.copy()
    selected = select_source(patches, point_lists, config)
    bank = PatchBank(targets)
    accepted, used = 0, 0
    damping = 1e-3
    for _ in range(config.outer_iterations):
        assignments = []
        for _, points in selected:
            world = transform(points, pose)
            match = associate(world, bank, config)
            if match is not None:
                d, _ = distance_and_gradient(targets[match], world)
                good = d <= (25 if targets[match].kind == "distribution" else config.max_residual_distance**2)
                if good.sum() >= 3:
                    assignments.append((points[good], targets[match]))
        used = sum(len(p) for p, _ in assignments)
        if used < 12:
            break
        # Recenter the target frame at the current translation before each LM
        # solve. This keeps rotation lever arms independent of global map origin.
        pivot = pose[:3, 3].copy()
        def evaluate(candidate, jacobian=False):
            residuals, jacobians = [], []
            for points, patch in assignments:
                p = transform(points, candidate)
                residual, gradient = distance_and_gradient(patch, p)
                residuals.append(residual)
                if jacobian:
                    jacobians.append(pose_jacobian(p-pivot, gradient))
            return np.concatenate(residuals), np.vstack(jacobians) if jacobian else None
        def objective(residual):
            if config.robust_scale <= 0:
                return residual@residual
            c = config.robust_scale
            return float(c*c*np.log1p((residual/c)**2).sum())
        for _ in range(config.lm_iterations):
            residual, jacobian = evaluate(pose, True)
            if config.robust_scale > 0:
                weight = 1/np.sqrt(1+(residual/config.robust_scale)**2)
                weighted_j = jacobian*weight[:, None]
                weighted_r = residual*weight
            else:
                weighted_j, weighted_r = jacobian, residual
            h, g = weighted_j.T@weighted_j, weighted_j.T@weighted_r
            try:
                delta = -np.linalg.solve(h+damping*np.diag(np.maximum(np.diag(h), 1e-6)), g)
            except np.linalg.LinAlgError:
                break
            if not np.isfinite(delta).all():
                break
            # A bounded trust region protects against rank-deficient single planes.
            ratio = max(np.linalg.norm(delta[:3])/2, np.linalg.norm(delta[3:])/.2, 1)
            delta /= ratio
            candidate = pose.copy()
            candidate[:3, 3] -= pivot
            candidate = se3_exp(delta)@candidate
            candidate[:3, 3] += pivot
            trial, _ = evaluate(candidate)
            if objective(trial) < objective(residual):
                pose, damping = candidate, max(damping/3, 1e-8)
                accepted += 1
                if np.linalg.norm(delta) < 1e-5:
                    break
            else:
                damping = min(damping*10, 1e8)
    return pose, {"selected_points": used, "accepted_steps": accepted}


class QuadricMap:
    def __init__(self, config):
        self.config = config
        self.active = []
        self.archive = []

    def update(self, patches, lists, pose):
        # Associate this scan against the previous map, not against other patches
        # newly added from the same scan. The moment merges remain incremental.
        bank = PatchBank(self.active)
        for patch, points in zip(patches, lists):
            if patch.count < self.config.mapping_min_patch:
                continue
            moments = patch.moments.transformed(pose)
            sample = transform(subset(points, self.config.association_samples), pose)
            index = associate(sample, bank, self.config)
            if index is not None:
                candidate = self.active[index]
                d, _ = distance_and_gradient(candidate, sample)
                # Gate actual squared surface distance as well as patch centers.
                tolerance = 1.0 if candidate.kind == "distribution" else .09
                if float(np.mean(d)) <= tolerance:
                    candidate.moments.merge(moments)
                    self.active[index] = fit_patch(candidate.moments, self.config)
                    continue
            self.active.append(fit_patch(moments, self.config))
        retained = []
        for patch in self.active:
            if np.linalg.norm(patch.center-pose[:3, 3]) <= self.config.map_radius:
                retained.append(patch)
            else:
                self.archive.append(patch)
        self.active = retained

    def save(self, path):
        # Paper Sec. V-C: one lowest-MSE patch per 1.5 m hash cell.
        grid = {}
        for patch in self.archive+self.active:
            key = tuple(np.floor(patch.center/self.config.hash_resolution).astype(int))
            if key not in grid or patch.mse < grid[key].mse:
                grid[key] = patch
        patches = list(grid.values())
        tri = np.triu_indices(10)
        np.savez_compressed(path, origins=np.array([p.moments.origin for p in patches]),
                            moments=np.array([p.moments.gram[tri] for p in patches]),
                            counts=np.array([p.count for p in patches], dtype=np.int64),
                            resolution=self.config.hash_resolution)
        return len(patches)


def load_map(path, config):
    with np.load(path, allow_pickle=False) as data:
        patches = []
        tri = np.triu_indices(10)
        for origin, upper, count in zip(data["origins"], data["moments"], data["counts"]):
            gram = np.zeros((10, 10))
            gram[tri] = upper
            gram += np.triu(gram, 1).T
            patches.append(fit_patch(Moments(origin, gram, int(count)), config))
    return patches


class SpatialHash:
    """Integer-cell hash equivalent to the paper's fixed-resolution center code."""
    def __init__(self, patches, resolution=1.5):
        self.resolution = resolution
        self.cells = {tuple(np.floor(p.center/resolution).astype(int)): p for p in patches}

    def query(self, center, radius=25):
        lo = np.floor((center-radius)/self.resolution).astype(int)
        hi = np.floor((center+radius)/self.resolution).astype(int)
        found = []
        for x in range(lo[0], hi[0]+1):
            for y in range(lo[1], hi[1]+1):
                for z in range(lo[2], hi[2]+1):
                    p = self.cells.get((x, y, z))
                    if p is not None and np.linalg.norm(p.center-center) <= radius:
                        found.append(p)
        return found
