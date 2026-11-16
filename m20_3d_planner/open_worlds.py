"""Import original AWS RoboMaker collision meshes as local planning tasks.

Geometry is preserved in metres. Tasks change only the rigid coordinate frame
and crop outside a box containing every reachable body pose. No planner result
is consulted when choosing tasks. Raw sources and download hashes are retained.
"""

from functools import lru_cache
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from . import core, scenes

DATA = Path(__file__).with_name("datasets") / "open_worlds"
TRUTH_SPACING = .012
CROP_LO = np.array([-1.95, -1.15, -.15])
CROP_HI = np.array([1.95, 1.15, 2.05])


def transform_pose(text=None):
    pose = np.fromstring(text or "0 0 0 0 0 0", sep=" ")
    if pose.shape != (6,):
        raise ValueError("SDF pose must have six components")
    out = np.eye(4)
    out[:3, :3] = core.rpy_matrix(pose[3:])
    out[:3, 3] = pose[:3]
    return out


def apply_transform(triangles, matrix):
    return triangles @ matrix[:3, :3].T + matrix[:3, 3]


@lru_cache(maxsize=128)
def load_collada(path):
    """Read the triangles/polylist and scene matrices in the downloaded DAEs."""
    root = ET.parse(path).getroot()
    for node in root.iter():
        node.tag = node.tag.rsplit("}", 1)[-1]
    if root.findtext("asset/up_axis", "Z_UP") != "Z_UP":
        raise ValueError("Only Z_UP assets are accepted; do not silently rotate a world")
    unit = root.find("asset/unit")
    metre = float(unit.attrib.get("meter", 1)) if unit is not None else 1.
    geometries = {}
    for geometry in root.findall("library_geometries/geometry"):
        mesh = geometry.find("mesh")
        sources = {}
        for source in mesh.findall("source"):
            access = source.find("technique_common/accessor")
            stride = int(access.attrib.get("stride", 1))
            values = np.fromstring(source.findtext("float_array"), sep=" ")
            sources[source.attrib["id"]] = values.reshape(-1, stride)
        vertices = {v.attrib["id"]: v.find("input[@semantic='POSITION']").attrib["source"][1:]
                    for v in mesh.findall("vertices")}
        triangles = []
        for primitive in list(mesh):
            if primitive.tag not in ("triangles", "polylist"):
                if primitive.tag == "polygons":
                    raise ValueError("Unsupported COLLADA polygons")
                continue
            inputs = primitive.findall("input")
            stride = max(int(i.attrib.get("offset", 0)) for i in inputs) + 1
            vertex = next(i for i in inputs if i.attrib["semantic"] == "VERTEX")
            index = np.fromstring(primitive.findtext("p"), sep=" ", dtype=int).reshape(-1, stride)
            index = index[:, int(vertex.attrib.get("offset", 0))]
            positions = sources[vertices[vertex.attrib["source"][1:]]][:, :3]
            counts = (np.full(len(index) // 3, 3) if primitive.tag == "triangles"
                      else np.fromstring(primitive.findtext("vcount"), sep=" ", dtype=int))
            cursor = 0
            for count in counts:
                face = index[cursor:cursor + count]
                triangles.extend(positions[face[[0, j, j + 1]]] for j in range(1, count - 1))
                cursor += count
            if cursor != len(index):
                raise ValueError("Malformed COLLADA primitive indices")
        geometries[geometry.attrib["id"]] = np.asarray(triangles)
    active = root.find("scene/instance_visual_scene")
    visual = root.find("library_visual_scenes/visual_scene[@id='%s']" % active.attrib["url"][1:])
    output = []

    def visit(node, parent):
        local = np.eye(4)
        for child in node:
            if child.tag == "matrix":
                local = local @ np.fromstring(child.text, sep=" ").reshape(4, 4)
            elif child.tag in ("translate", "rotate", "scale", "instance_node"):
                raise ValueError("Unexpected COLLADA transform; extend parser explicitly")
        matrix = parent @ local
        for instance in node.findall("instance_geometry"):
            output.append(apply_transform(geometries[instance.attrib["url"][1:]], matrix) * metre)
        for child in node.findall("node"):
            visit(child, matrix)

    for node in visual.findall("node"):
        visit(node, np.eye(4))
    if not output:
        raise ValueError(f"No mesh instances in {path}")
    result = np.concatenate(output)
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite mesh")
    return result


def load_world(kind):
    root = DATA / kind
    relative = ("worlds/no_roof_small_warehouse/no_roof_small_warehouse.world" if kind == "warehouse"
                else "worlds/small_house.world")
    world_path = root / relative
    world = ET.parse(world_path).find("world")
    output = []
    for instance in world.findall("model"):
        uri = instance.findtext("include/uri")
        if uri is None:
            raise ValueError("Unexpected inline world model")
        name = uri.removeprefix("model://")
        sdf_path = root / "models" / name / "model.sdf"
        model = ET.parse(sdf_path).find("model")
        model_matrix = transform_pose(instance.findtext("pose")) @ transform_pose(model.findtext("pose"))
        parts = []
        for link in model.findall("link"):
            link_matrix = model_matrix @ transform_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                matrix = link_matrix @ transform_pose(collision.findtext("pose"))
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    sphere = collision.find("geometry/sphere")
                    if sphere is None:
                        raise ValueError(f"Unsupported collision geometry in {sdf_path}")
                    radius = float(sphere.findtext("radius"))
                    n = 64
                    ring = np.arange(n) * 2 * np.pi / n
                    latitude = np.linspace(-np.pi / 2, np.pi / 2, n // 2 + 1)
                    vertices = radius * np.stack(np.broadcast_arrays(
                        np.cos(latitude[:, None]) * np.cos(ring),
                        np.cos(latitude[:, None]) * np.sin(ring), np.sin(latitude[:, None])
                    ), axis=-1)
                    faces = []
                    for i in range(len(latitude) - 1):
                        for j in range(n):
                            k = (j + 1) % n
                            faces.extend([[vertices[i, j], vertices[i + 1, j], vertices[i + 1, k]],
                                          [vertices[i, j], vertices[i + 1, k], vertices[i, k]]])
                    parts.append(apply_transform(np.asarray(faces), matrix))
                    continue
                mesh_uri = mesh.findtext("uri").removeprefix("file://").removeprefix("model://")
                if not mesh_uri.startswith("models/"):
                    mesh_uri = "models/" + mesh_uri
                scale = np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ")
                parts.append(apply_transform(load_collada(str(root / mesh_uri)) * scale, matrix))
        if parts:
            output.append(dict(name=instance.attrib["name"], model=name, triangles=np.concatenate(parts),
                               pose=instance.findtext("pose"), source_sdf=str(sdf_path.relative_to(DATA)),
                               approximation_bound_m=(4 * radius * np.pi ** 2 / n ** 2
                                                      if model.find(".//geometry/sphere") is not None else 0.)))
    return output, world_path


def clip_triangles(triangles, lower=CROP_LO, upper=CROP_HI):
    keep = np.all(triangles.max(axis=1) >= lower, axis=1) & np.all(triangles.min(axis=1) <= upper, axis=1)
    output = []
    for triangle in triangles[keep]:
        polygon = list(triangle)
        for axis in range(3):
            for sign, limit in ((1., lower[axis]), (-1., upper[axis])):
                clipped = []
                for a, b in zip(polygon, polygon[1:] + polygon[:1]):
                    a_in, b_in = sign * (a[axis] - limit) >= 0, sign * (b[axis] - limit) >= 0
                    if a_in:
                        clipped.append(a)
                    if a_in != b_in:
                        clipped.append(a + (limit - a[axis]) / (b[axis] - a[axis]) * (b - a))
                polygon = clipped
                if not polygon:
                    break
            if not polygon:
                break
        for j in range(1, len(polygon) - 1):
            output.append([polygon[0], polygon[j], polygon[j + 1]])
    return np.asarray(output).reshape(-1, 3, 3)


def sample_triangles(triangles, spacing):
    """Longest-edge refinement: each microtriangle has edge <= spacing.

    Every original surface point is within `spacing` of a returned sample.
    This is a deterministic coverage bound, not a random density assumption.
    """
    triangles = np.asarray(triangles)
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    work = triangles[area > 1e-14]
    output = []
    while len(work):
        edge_square = np.sum((np.roll(work, -1, axis=1) - work) ** 2, axis=2)
        done = edge_square.max(axis=1) <= spacing ** 2
        if done.any():
            output.append(work[done].reshape(-1, 3))
        work = work[~done]
        if not len(work):
            break
        index = edge_square[~done].argmax(axis=1)
        rows = np.arange(len(work))
        a, b, c = work[rows, index], work[rows, (index + 1) % 3], work[rows, (index + 2) % 3]
        midpoint = (a + b) / 2
        work = np.concatenate((np.stack((a, midpoint, c), axis=1), np.stack((midpoint, b, c), axis=1)))
    if not output:
        return np.empty((0, 3))
    return np.unique(np.round(np.concatenate(output), 10), axis=0)


def triangle_hits_box(triangles, position, rotation, half):
    """Separating-axis triangle/OBB intersection, including edge/edge axes."""
    t = (triangles - position) @ rotation
    keep = np.all(t.min(axis=1) <= half, axis=1) & np.all(t.max(axis=1) >= -half, axis=1)
    t = t[keep]
    if not len(t):
        return False
    edges = np.roll(t, -1, axis=1) - t
    axes = [np.cross(edges[:, 0], edges[:, 1])]
    axes.extend(np.cross(edges[:, i], unit) for i in range(3) for unit in np.eye(3))
    valid = np.ones(len(t), dtype=bool)
    for axis in axes:
        projection = np.einsum("nij,nj->ni", t, axis)
        bound = np.abs(axis) @ half
        valid &= (projection.min(axis=1) <= bound + 1e-12) & (projection.max(axis=1) >= -bound - 1e-12)
    return bool(valid.any())


def point_inside_mesh(point, triangles):
    direction = np.array([1., .37139, .22361])
    a, b, c = triangles.transpose(1, 0, 2)
    e1, e2 = b - a, c - a
    h = np.cross(direction, e2)
    det = np.einsum("ij,ij->i", e1, h)
    valid = np.abs(det) > 1e-12
    inv = np.zeros_like(det)
    inv[valid] = 1 / det[valid]
    delta = point - a
    u = np.einsum("ij,ij->i", delta, h) * inv
    q = np.cross(delta, e1)
    v = q @ direction * inv
    distance = np.einsum("ij,ij->i", e2, q) * inv
    hits = distance[valid & (u >= 0) & (v >= 0) & (u + v <= 1) & (distance > 1e-9)]
    return len(np.unique(np.round(hits, 8))) % 2 == 1


def prepare_tasks(half, nominal_height=.4):
    """Freeze eight source-world tasks using endpoint and collision criteria.

    Candidate tasks are accepted before running any solver. Geometry is neither
    resized nor rearranged. Nearby original objects remain in each task.
    """
    groups = {
        "warehouse": [("shelf", "仓库·货架支脚", ("ShelfD", "ShelfE", "ShelfF")),
                      ("clutter", "仓库·堆放箱群", ("ClutteringC", "ClutteringA", "ClutteringD")),
                      ("bucket", "仓库·桶组", ("Bucket",)),
                      ("pallet_jack", "仓库·搬运车", ("PalletJack",))],
        "house": [("chairs", "住宅·餐椅与桌边", ("ChairA", "ChairD")),
                  ("coffee_table", "住宅·低茶几", ("CoffeeTable",)),
                  ("sofa", "住宅·沙发转角", ("SofaC",)),
                  ("cabinet", "住宅·柜体与墙边", ("Wardrobe", "KitchenCabinet", "ReadingDesk"))],
    }
    tasks = []
    folder = DATA / "prepared"
    folder.mkdir(exist_ok=True)
    for kind, definitions in groups.items():
        parts, source_path = load_world(kind)
        triangles = np.concatenate([p["triangles"] for p in parts])
        bounds = [(p["triangles"].min(axis=(0, 1)), p["triangles"].max(axis=(0, 1))) for p in parts]
        floor = max(p["triangles"][:, :, 2].max() for p in parts if "GroundB" in p["model"] or "FloorB" in p["model"])

        def endpoint_free(point, rotation):
            if triangle_hits_box(triangles, point, rotation, half + .04):
                return False
            for part, (lo, hi) in zip(parts, bounds):
                if np.all(point >= lo) and np.all(point <= hi) and point_inside_mesh(point, part["triangles"]):
                    return False
            return True

        for slug, label, keywords in definitions:
            selected = None
            targets = [p for keyword in keywords for p in parts if keyword in p["model"]]
            for target in targets:
                band = clip_triangles(target["triangles"],
                                      np.array([-100., -100., floor + nominal_height - .05]),
                                      np.array([100., 100., floor + nominal_height + .05]))
                if not len(band):
                    continue
                centers = np.unique(np.round(band.mean(axis=1)[:, :2], 4), axis=0)
                for xy in centers:
                    origin = np.r_[xy, floor]
                    for heading in (0., np.pi / 2, np.pi / 4, 3 * np.pi / 4):
                        rotation = core.rz(heading)
                        endpoints = [origin + rotation @ np.array([x, 0., nominal_height]) for x in (-1.3, 1.3)]
                        if not all(endpoint_free(p, rotation) for p in endpoints):
                            continue
                        center = origin + np.array([0., 0., nominal_height])
                        if not triangle_hits_box(triangles, center, rotation, half - .005):
                            continue
                        selected = (target, origin, rotation, heading)
                        break
                    if selected is not None:
                        break
                if selected is not None:
                    break
            if selected is None:
                raise RuntimeError(f"No endpoint-free colliding task for {kind}/{slug}")
            target, origin, rotation, heading = selected
            local = clip_triangles((triangles - origin) @ rotation)
            sensor = sample_triangles(local, .025)
            truth = sample_triangles(local, TRUTH_SPACING)
            # Account for the analytic sphere-to-mesh approximation, if a ball
            # is present in this crop, in addition to lattice covering radius.
            curved_error = max((p["approximation_bound_m"] for p in parts
                                if len(clip_triangles((p["triangles"] - origin) @ rotation))), default=0.)
            name = f"open_{kind}_{slug}"
            np.savez_compressed(folder / (name + ".npz"), sensor_surface=sensor, truth_surface=truth,
                                collision_triangles=local)
            manifest = json.loads((DATA / kind / "download_manifest.json").read_text())
            task = dict(name=name, label=label, description=f"开源 AWS {kind} 世界局部任务：{label}；原始碰撞网格、原尺寸。初值为未经避障的 25 mm 轻微 S 弯。",
                        source_repository=manifest["repository"], source_archive_sha256=manifest["archive_sha256"],
                        source_world=str(source_path.relative_to(DATA)), source_model=target["name"],
                        source_license="MIT-0", source_world_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                        world_origin_m=origin.tolist(), heading_rad=heading, geometry_scale=1.,
                        crop_lower_m=CROP_LO.tolist(), crop_upper_m=CROP_HI.tolist(),
                        truth_surface_cover_radius_m=TRUTH_SPACING + curved_error + 1e-9,
                        surface_kind="imported_collision_mesh", sensor_spacing_m=.025,
                        truth_spacing_m=TRUTH_SPACING, analytic_tessellation_error_bound_m=curved_error,
                        initial_center_collision_margin_m=.005, endpoints_inflated_body_clearance_m=.04,
                        default_view="iso", default_zoom=1., triangle_count=len(local),
                        sensor_points=len(sensor), truth_points=len(truth))
            tasks.append(task)
            print(name, target["name"], origin.round(3), round(np.degrees(heading)),
                  len(local), len(sensor), len(truth), flush=True)
    (DATA / "tasks.json").write_text(json.dumps(dict(tasks=tasks, selection="first collision-containing task with free endpoints; no planner results consulted"), ensure_ascii=False, indent=2) + "\n")
    return tasks


def load_prepared_scenes():
    manifest_path = DATA / "tasks.json"
    manifest = json.loads(manifest_path.read_text())
    output = []
    for task in manifest["tasks"]:
        with np.load(DATA / "prepared" / (task["name"] + ".npz")) as data:
            output.append(scenes.Scene(task["name"], "open_world", data["sensor_surface"],
                                       data["truth_surface"], task))
    return output


if __name__ == "__main__":
    prepare_tasks(np.array([.375, .115, .07000000254808628]))
