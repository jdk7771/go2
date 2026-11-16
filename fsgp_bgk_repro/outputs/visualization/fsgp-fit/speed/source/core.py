"""Paper equations (1)--(11), with explicit numerical/coverage conventions.

This is a paper-equation reference, not a bit-for-bit port of the upstream
GPyTorch program. See README.md for the model and preprocessing differences.
Coordinates are gravity-aligned world coordinates, in metres.
"""

from dataclasses import dataclass, asdict
from time import perf_counter

import numpy as np
from scipy.linalg import cholesky, cho_solve, solve_triangular
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class Config:
    resolution: float = 0.2
    length: float = 5.0
    neighbors: int = 20
    interpolation_neighbors: int = 5
    voxel_size: float = 0.1
    curvature_threshold: float = 0.03
    gradient_threshold: float = 0.03
    inducing_points: int = 125
    pca: bool = True
    lengthscale: float = 0.6
    alpha: float = 10.0
    signal_variance: float = 1.0
    noise_variance: float = 0.0004
    jitter: float = 1e-8
    weights: tuple = (1 / 3, 1 / 3, 1 / 3)  # curvature, height variation, GP slope
    support_radius: float = 0.45
    max_variance: float = 0.8
    decay_rate: float = 1.0
    history_ttl: float = 60.0
    smoothing_sigma: float = 0.8  # grid cells
    seed: int = 7

    def __post_init__(self):
        positive = ("resolution", "length", "voxel_size", "lengthscale", "alpha",
                    "signal_variance", "jitter", "support_radius", "max_variance",
                    "history_ttl")
        for name in positive:
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("noise_variance", "decay_rate", "smoothing_sigma",
                     "curvature_threshold", "gradient_threshold"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        for name, minimum in (("neighbors", 3), ("interpolation_neighbors", 1),
                              ("inducing_points", 3)):
            value = getattr(self, name)
            if not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        if len(self.weights) != 3 or min(self.weights) < 0 or not np.isclose(sum(self.weights), 1):
            raise ValueError("three nonnegative weights must sum to one")
        if not np.isclose(self.length / self.resolution, round(self.length / self.resolution)):
            raise ValueError("length must be an integer multiple of resolution")

    def as_dict(self):
        return asdict(self)


def finite_points(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("point cloud must be N x 3 (or more columns)")
    points = points[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def extract_features(points, neighbors=20):
    """Eq. (1): 3D KNN including self, covariance curvature, mean |delta z|."""
    points = finite_points(points)
    if len(points) < 3:
        raise ValueError("at least three finite points are required")
    _, indices = cKDTree(points).query(points, k=min(neighbors, len(points)))
    local = points[indices]
    centered = local - local.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / (indices.shape[1] - 1)
    eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0)
    curvature = eigenvalues[:, 0] / (eigenvalues.sum(axis=1) + 1e-6)
    variation = np.abs(local[:, :, 2] - points[:, None, 2]).mean(axis=1)
    return np.column_stack((points, curvature, variation))


def select_inducing(features, config, rng, feature_based=True):
    """Eq. (2): feature points union voxel representatives, then random cap."""
    if feature_based:
        is_feature = ((features[:, 3] > config.curvature_threshold) |
                      (features[:, 4] > config.gradient_threshold))
        non_feature = np.flatnonzero(~is_feature)
        voxels = np.floor(features[non_feature, :3] / config.voxel_size).astype(np.int64)
        _, representatives = np.unique(voxels, axis=0, return_index=True)
        indices = np.concatenate((np.flatnonzero(is_feature), non_feature[representatives]))
    else:
        indices = np.arange(len(features))
    if len(indices) > config.inducing_points:
        indices = rng.choice(indices, config.inducing_points, replace=False)
    return features[indices]


def interpolate_features(features, query_xy, neighbors=5):
    """Eqs. (4),(5): normalized inverse-distance weights in the XY plane."""
    distances, indices = cKDTree(features[:, :2]).query(
        query_xy, k=min(neighbors, len(features)))
    if distances.ndim == 1:
        distances, indices = distances[:, None], indices[:, None]
    weights = 1 / (distances + 1e-6)
    weights /= weights.sum(axis=1, keepdims=True)
    attributes = np.einsum("nk,nkd->nd", weights, features[indices, 3:5])
    return np.column_stack((query_xy, attributes)), distances[:, 0]


class SparseTerrainGP:
    """Eqs. (6),(7): GP conditioned on M retained observations.

    Rational quadratic kernel, fixed declared hyperparameters, empirical constant
    mean, latent posterior variance. Training costs O(M^3); queries O(Q M^2).
    PCA is fitted once on training inputs and reused for prediction/derivatives.
    """

    def __init__(self, config):
        self.config = config

    def fit(self, x, heights):
        self.input_mean = x.mean(axis=0)
        self.rotation = np.eye(x.shape[1])
        if self.config.pca:
            _, self.rotation = np.linalg.eigh(np.cov(x, rowvar=False))
            self.rotation = self.rotation[:, ::-1]
        self.train_x = (x - self.input_mean) @ self.rotation
        self.height_mean = float(np.mean(heights))
        covariance = self.kernel(self.train_x, self.train_x)
        covariance.flat[::len(x) + 1] += self.config.noise_variance + self.config.jitter
        self.factor = cholesky(covariance, lower=True, check_finite=False)
        self.coefficients = cho_solve((self.factor, True), heights - self.height_mean,
                                      check_finite=False)
        return self

    def kernel(self, x, z):
        squared_distance = np.sum((x[:, None, :] - z[None, :, :]) ** 2, axis=2)
        c = self.config
        return c.signal_variance * (1 + squared_distance / (2 * c.alpha * c.lengthscale**2)) ** (-c.alpha)

    def predict(self, x, batch_size=512):
        means, variances, gradients = [], [], []
        c = self.config
        for start in range(0, len(x), batch_size):
            transformed = (x[start:start + batch_size] - self.input_mean) @ self.rotation
            delta = transformed[:, None, :] - self.train_x[None, :, :]
            base = 1 + np.sum(delta**2, axis=2) / (2 * c.alpha * c.lengthscale**2)
            kernel = c.signal_variance * base**(-c.alpha)
            means.append(self.height_mean + kernel @ self.coefficients)
            solved = solve_triangular(self.factor, kernel.T, lower=True, check_finite=False)
            variances.append(np.maximum(c.signal_variance - np.sum(solved**2, axis=0), 0))
            derivative = -kernel[:, :, None] * delta / (c.lengthscale**2 * base[:, :, None])
            gradient = np.einsum("nmd,m->nd", derivative, self.coefficients) @ self.rotation.T
            gradients.append(gradient[:, :2])  # partial wrt XY; hold kappa,g fixed
        return np.concatenate(means), np.concatenate(variances), np.vstack(gradients)


def smooth_known(values, known, sigma):
    """Eq. (11), normalized; unknown cells neither contribute nor become known."""
    if sigma == 0:
        return np.where(known, values, np.nan)
    numerator = gaussian_filter(np.where(known, values, 0.0), sigma, mode="constant")
    denominator = gaussian_filter(known.astype(float), sigma, mode="constant")
    return np.where(known, numerator / np.maximum(denominator, 1e-15), np.nan)


def bgk_update(previous_cost, previous_variance, elapsed, cost, variance, decay_rate):
    """Literal Eqs. (9),(10); no current-variance confidence in Eq. (9)."""
    temporal_weight = np.exp(-decay_rate * elapsed)
    prior_weight = temporal_weight / (previous_variance + 1e-6)
    fused = (prior_weight * previous_cost + cost) / (prior_weight + 1)
    fused_variance = (temporal_weight * previous_variance + variance) / (temporal_weight + 1)
    return float(fused), float(fused_variance)


class BGKHistory:
    """World cell IDs, timestamped evidence, bounded rolling window and TTL."""

    def __init__(self, config):
        self.config = config
        self.cells = {}
        self.latest_time = -np.inf

    def fuse(self, cell_ids, cost, variance, observed, timestamp):
        if not np.isfinite(timestamp) or timestamp < self.latest_time:
            raise ValueError("timestamps must be finite and nondecreasing")
        self.latest_time = timestamp
        active = {tuple(cell) for cell in cell_ids}
        self.cells = {key: value for key, value in self.cells.items()
                      if key in active and timestamp - value[2] <= self.config.history_ttl}
        fused, uncertainty = np.full(len(cost), np.nan), np.full(len(cost), np.nan)
        known = np.zeros(len(cost), dtype=bool)
        for i, cell in enumerate(cell_ids):
            key = tuple(cell)
            if observed[i]:
                current_cost, current_variance = float(cost[i]), float(variance[i])
                if key in self.cells:
                    old_cost, old_variance, old_time = self.cells[key]
                    current_cost, current_variance = bgk_update(
                        old_cost, old_variance, timestamp - old_time,
                        current_cost, current_variance, self.config.decay_rate)
                self.cells[key] = (current_cost, current_variance, timestamp)
            if key in self.cells:
                fused[i], uncertainty[i], _ = self.cells[key]
                known[i] = True
        return fused, uncertainty, known


class TraversabilityMapper:
    def __init__(self, config=None):
        self.config = config or Config()
        self.rng = np.random.default_rng(self.config.seed)
        self.history = BGKHistory(self.config)

    def grid(self, center):
        c = self.config
        size = round(c.length / c.resolution)
        origin = np.floor(np.asarray(center[:2]) / c.resolution).astype(np.int64) - size // 2
        x, y = np.meshgrid(np.arange(size) + origin[0], np.arange(size) + origin[1])
        cell_ids = np.column_stack((x.ravel(), y.ravel()))
        return (cell_ids + 0.5) * c.resolution, cell_ids, (size, size)

    def assess(self, points, center=(0, 0), timestamp=0.0, feature_based=True):
        """Compute a single-frame SGP/FSGP estimate; fuse() adds temporal BGK."""
        start = perf_counter()
        c = self.config
        xy, cell_ids, shape = self.grid(center)
        points = finite_points(points)
        margin = c.length / 2 + c.support_radius
        inside = np.max(np.abs(points[:, :2] - np.asarray(center[:2])), axis=1) <= margin
        points = points[inside]
        if len(points) < 3:
            raise ValueError("local window contains fewer than three finite points")
        features = extract_features(points, c.neighbors)
        inducing = select_inducing(features, c, self.rng, feature_based)
        if len(inducing) < 3:
            raise ValueError("fewer than three inducing points; reduce voxel size")
        test, nearest_distance = interpolate_features(features, xy, c.interpolation_neighbors)
        train = inducing[:, [0, 1, 3, 4]] if feature_based else inducing[:, :2]
        test_inputs = test if feature_based else xy
        feature_time = perf_counter() - start
        gp_start = perf_counter()
        gp = SparseTerrainGP(c).fit(train, inducing[:, 2])
        height, variance, gradient = gp.predict(test_inputs)
        slope = np.linalg.norm(gradient, axis=1)
        # Eq. (8), raw physical attributes, no undocumented per-frame rescaling.
        preliminary = (c.weights[0] * test[:, 2] + c.weights[1] * test[:, 3] +
                       c.weights[2] * slope)
        observed = (nearest_distance <= c.support_radius) & (variance <= c.max_variance)
        cost = smooth_known(preliminary.reshape(shape), observed.reshape(shape), c.smoothing_sigma)
        return {"xy": xy, "cell_ids": cell_ids, "shape": shape, "height": height,
                "variance": variance, "slope": slope, "curvature": test[:, 2],
                "height_variation": test[:, 3], "preliminary": preliminary,
                "observed": observed, "known": observed.copy(), "cost": cost,
                "inducing": inducing, "points": points, "timestamp": float(timestamp),
                "timing_ms": {"features": feature_time * 1000,
                              "gp_and_score": (perf_counter() - gp_start) * 1000,
                              "total": (perf_counter() - start) * 1000}}

    def fuse(self, result):
        start = perf_counter()
        cost, variance, known = self.history.fuse(
            result["cell_ids"], result["preliminary"], result["variance"],
            result["observed"], result["timestamp"])
        smoothed = smooth_known(cost.reshape(result["shape"]), known.reshape(result["shape"]),
                                self.config.smoothing_sigma)
        timing = dict(result["timing_ms"])
        timing["bgk"] = (perf_counter() - start) * 1000
        timing["total"] += timing["bgk"]
        return {**result, "cost": smoothed, "fused_preliminary": cost,
                "fused_variance": variance, "known": known, "timing_ms": timing}
