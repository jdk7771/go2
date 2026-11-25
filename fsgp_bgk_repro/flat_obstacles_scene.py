"""An exposed flat floor, open-ended corridor, and two solid platforms.

Complete synthetic surfaces, without LiDAR occlusion or noise. Part labels are
saved for geometry auditing only; fitting and the six metrics receive XYZ only.
"""
import json
import numpy as np

from .composite_scene import grid, write_pcd
from .corridor_scenes import build as corridor_cloud
from .scene_fit_catalog import WEB
from .table_scene import _face

SCENE_ID = 'flat-corridor-platforms'


def build(spacing=.025):
    corridor, labels, _, _, _, _ = corridor_cloud(.6, spacing)
    structures = corridor[labels > 0].astype(float)
    structures[:, 1] += 1.5
    structure_labels = labels[labels > 0]
    platforms = [
        dict(name='矮台', label=3, min=[-2.6, -2.3, 0.], max=[-1., -.9, .15]),
        dict(name='高台', label=4, min=[1., -2.3, 0.], max=[2.6, -.9, .8]),
    ]
    xy = grid(-4, 4, -4, 4, spacing)
    walls = [dict(min=[-2., y0 + 1.5, 0.], max=[2., y1 + 1.5, .6])
             for y0, y1 in [(-1.12, -1.), (1., 1.12)]]
    covered = np.zeros(len(xy), bool)
    for box in walls + platforms:
        covered |= np.all((xy >= np.array(box['min'][:2]) - 1e-8)
                          & (xy <= np.array(box['max'][:2]) + 1e-8), axis=1)
    floor = np.c_[xy[~covered], np.zeros((~covered).sum())]
    clouds, groups = [floor, structures], [np.zeros(len(floor), np.uint8), structure_labels]
    for box in platforms:
        lo, hi = np.asarray(box['min']), np.asarray(box['max'])
        # Exposed top and four sides. The buried bottom and floor below each
        # solid are omitted. There is no traversable space under a platform.
        faces = [_face(lo, hi, axis, side, spacing) for axis in (0, 1) for side in (0, 1)]
        faces.append(_face(lo, hi, 2, 1, spacing))
        cloud = np.concatenate(faces)
        clouds.append(cloud); groups.append(np.full(len(cloud), box['label'], np.uint8))
    points, ids = np.unique(np.concatenate(clouds).astype('<f4'), axis=0, return_index=True)
    labels = np.concatenate(groups)[ids]
    names = ['地面', '通道顶板', '通道侧墙', '矮台', '高台']
    meta = dict(id=SCENE_ID, name='平地 · 通道与高低台', units='m',
        kind='complete_synthetic_surface', simulated_lidar=False, spacing_m=spacing,
        floor_size_m=[8., 8.], bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
        point_count=len(points), corridor=dict(length_m=4., inner_width_m=2., clear_height_m=.6,
            roof_thickness_m=.12, wall_thickness_m=.12, center_xy_m=[0., 1.5],
            x_range_m=[-2., 2.], inner_y_range_m=[.5, 2.5], open_ends=True),
        platforms=platforms, buried_contact_faces_sampled=False,
        fitting_input='XYZ only; generated part labels are never used for model segmentation',
        parts=[dict(label=i, name=name, point_count=int((labels == i).sum())) for i, name in enumerate(names)])
    return points, labels, meta


def export():
    points, labels, meta = build()
    folder = WEB / SCENE_ID; folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder/'scene.npz', points=points, labels=labels)
    points.tofile(folder/'points.bin')
    write_pcd(folder/'scene.pcd', points, labels, labels, np.full(len(points), 3),
              description='Flat floor with corridor and two solid platforms')
    (folder/'scene.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    return meta


if __name__ == '__main__':
    export()
