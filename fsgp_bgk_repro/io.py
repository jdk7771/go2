"""Small, strict point-cloud reader; no Open3D dependency."""

from pathlib import Path
import numpy as np

from .core import finite_points


def read_pcd(path):
    """Read PCD ASCII or uncompressed little-endian binary, arbitrary fields."""
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD DATA header is missing")
            fields = line.decode("ascii").strip().split()
            if not fields or fields[0].startswith("#"):
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == "DATA":
                break
        names = header["FIELDS"]
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PCD requires x, y, z fields")
        counts = list(map(int, header.get("COUNT", ["1"] * len(names))))
        count = int(header["POINTS"][0])
        mode = header["DATA"][0]
        if mode == "ascii":
            array = np.loadtxt(stream, ndmin=2)
            if array.shape != (count, sum(counts)):
                raise ValueError("PCD ASCII payload does not match header")
            offsets = np.cumsum([0] + counts[:-1])
            return finite_points(array[:, [offsets[names.index(n)] for n in ("x", "y", "z")]])
        if mode != "binary":
            raise ValueError(f"unsupported PCD DATA {mode}; export ASCII or binary")
        type_map = {"F": "f", "I": "i", "U": "u"}
        dtype = np.dtype([(name, "<" + type_map[kind] + size, (n,))
                          for name, kind, size, n in zip(names, header["TYPE"], header["SIZE"], counts)])
        payload = stream.read()
        if len(payload) != count * dtype.itemsize:
            raise ValueError("PCD binary payload does not match header")
        array = np.frombuffer(payload, dtype=dtype, count=count)
        return finite_points(np.column_stack([array[n][:, 0] for n in ("x", "y", "z")]))


def read_cloud(path):
    path = Path(path)
    if path.suffix.lower() == ".pcd":
        return read_pcd(path)
    if path.suffix.lower() == ".npy":
        return finite_points(np.load(path, allow_pickle=False))
    if path.suffix.lower() == ".npz":
        with np.load(path, allow_pickle=False) as data:
            return finite_points(data["points"])
    return finite_points(np.loadtxt(path, delimiter="," if path.suffix.lower() == ".csv" else None,
                                    ndmin=2))


def save_result(path, result):
    keys = ("xy", "cell_ids", "height", "variance", "slope", "curvature",
            "height_variation", "preliminary", "cost", "known", "observed", "inducing")
    data = {key: result[key] for key in keys}
    for key in ("fused_preliminary", "fused_variance"):
        if key in result:
            data[key] = result[key]
    np.savez_compressed(path, **data, shape=result["shape"], timestamp=result["timestamp"])
