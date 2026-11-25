"""Hollow, extended variants of the second platform; original scene unchanged."""
import json
import numpy as np

from .composite_scene import grid, write_pcd
from .flat_obstacles_scene import build as original_scene
from .scene_fit_catalog import WEB
from .table_scene import _face

HEIGHTS = {'platform-underpass-070': .70, 'platform-underpass-042': .42}


def build(scene, spacing=.025):
    clearance = HEIGHTS[scene]
    points, labels, original = original_scene(spacing)
    keep = labels != 4
    # Initial yaw is zero: forward is +X and robot-right is -Y.
    # Retain the original front/back span; extend only the right lateral edge.
    slab = dict(name='第二个平台顶板', label=4,
                min=[1., -4., clearance], max=[2.6, -.9, clearance + .10])
    legs = [dict(name=f'支腿 {i+1}', label=5, min=[x, y, 0.],
                 max=[x+.12, y+.12, clearance])
            for i, (x, y) in enumerate((x, y) for x in (1., 2.48) for y in (-4., -1.02))]
    xy = grid(1., 2.6, -4., -.9, spacing)
    floor = np.c_[xy, np.zeros(len(xy))]
    clouds, groups = [points[keep], floor], [labels[keep], np.zeros(len(floor), np.uint8)]
    for box in [slab, *legs]:
        lo, hi = np.asarray(box['min']), np.asarray(box['max'])
        faces = [_face(lo, hi, axis, side, spacing) for axis in (0, 1) for side in (0, 1)]
        if box is slab:
            faces += [_face(lo, hi, 2, side, spacing) for side in (0, 1)]
        cloud = np.concatenate(faces)
        clouds.append(cloud); groups.append(np.full(len(cloud), box['label'], np.uint8))
    points, first = np.unique(np.concatenate(clouds).astype('<f4'), axis=0, return_index=True)
    labels = np.concatenate(groups)[first]
    # Omit buried contact points, including the restored floor inside each leg.
    buried = np.zeros(len(points), bool)
    for leg in legs:
        xy_inside = np.all((points[:, :2] >= np.array(leg['min'][:2])-1e-7)
                           & (points[:, :2] <= np.array(leg['max'][:2])+1e-7), axis=1)
        buried |= xy_inside & ((np.abs(points[:, 2]) < 1e-7)
                              | (np.abs(points[:, 2]-clearance) < 1e-7))
    points, labels = points[~buried], labels[~buried]
    meta = dict(original, id=scene, name=f'高低台 · 第二台下方净空 {clearance*100:.0f} cm · 向机器人右侧延伸',
        point_count=len(points), bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
        source_scene='flat-corridor-platforms', clearance_m=clearance,
        platforms=[original['platforms'][0], dict(slab, hollow=True)],
        tabletop=slab, legs=legs, boxes=[slab, *legs], upper_display_threshold_m=.30,
        second_platform=dict(x_range_m=[1., 2.6], y_range_m=[-4., -.9],
            clearance_m=clearance, top_height_m=clearance+.10,
            slab_thickness_m=.10, extends_to_right_boundary=True, floor_restored=True,
            extension_direction_world=[0.,-1.,0.],initial_yaw_rad=0.,
            forward_direction_world=[1.,0.,0.],right_direction_world=[0.,-1.,0.]),
        planning_test=dict(start_xy_m=[-3.2, -1.6], goal_xy_m=[3.2, -1.6],
            goal_under_second_platform=False, exits_platform=True),
        parts=[dict(label=i, name=name, point_count=int((labels==i).sum()))
               for i, name in enumerate(['地面','通道顶板','通道侧墙','矮台','第二台顶板','支腿'])])
    return points, labels, meta


def export(scene):
    points, labels, meta = build(scene)
    folder = WEB/scene; folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder/'scene.npz', points=points, labels=labels)
    points.tofile(folder/'points.bin')
    write_pcd(folder/'scene.pcd', points, labels, labels, np.full(len(points), 3),
              description='Extended hollow second platform on flat terrain')
    (folder/'scene.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2)+'\n')
    print(scene, 'points', len(points), 'clearance', meta['clearance_m'], flush=True)
    return meta


if __name__ == '__main__':
    for name in HEIGHTS: export(name)
