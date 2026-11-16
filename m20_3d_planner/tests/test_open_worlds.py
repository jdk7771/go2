import tempfile
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial import cKDTree

from m20_3d_planner import core, experiment, open_worlds, scenes
from m20_3d_planner.initialization import small_bend_initialization


class OpenWorldChecks(unittest.TestCase):
    def test_collada_units_matrix_and_quad_triangulation(self):
        xml = '''<COLLADA><asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>
        <library_geometries><geometry id="g"><mesh>
        <source id="p"><float_array>0 0 0 100 0 0 100 100 0 0 100 0</float_array>
        <technique_common><accessor stride="3"/></technique_common></source>
        <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
        <polylist><input semantic="VERTEX" offset="0" source="#v"/><vcount>4</vcount><p>0 1 2 3</p></polylist>
        </mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="s"><node>
        <matrix>1 0 0 100 0 1 0 200 0 0 1 300 0 0 0 1</matrix><instance_geometry url="#g"/>
        </node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'quad.dae'
            path.write_text(xml)
            triangles = open_worlds.load_collada(str(path))
        self.assertEqual(triangles.shape, (2, 3, 3))
        np.testing.assert_allclose(triangles.min(axis=(0, 1)), [1, 2, 3])
        np.testing.assert_allclose(triangles.max(axis=(0, 1)), [2, 3, 3])

    def test_clipping_keeps_crossing_triangle_and_samples_cover_surface(self):
        triangles = np.array([[[-3., -2, .4], [3, -2, .4], [0, 3, .4]]])
        clipped = open_worlds.clip_triangles(triangles, np.array([-1, -1, 0]), np.array([1, 1, 1]))
        self.assertGreater(len(clipped), 0)
        self.assertTrue((np.abs(clipped[:, :, :2]) <= 1 + 1e-12).all())
        samples = open_worlds.sample_triangles(clipped, .08)
        rng = np.random.default_rng(7)
        weights = rng.dirichlet([1, 1, 1], 3000)
        probes = np.concatenate([weights @ triangle for triangle in clipped])
        self.assertLessEqual(cKDTree(samples).query(probes)[0].max(), .08 + 1e-9)

    def test_triangle_box_sat_and_mesh_inside(self):
        vertices = np.array([[x, y, z] for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])
        from scipy.spatial import ConvexHull
        triangles = vertices[ConvexHull(vertices).simplices]
        self.assertTrue(open_worlds.point_inside_mesh(np.zeros(3), triangles))
        self.assertFalse(open_worlds.point_inside_mesh(np.array([2., 0, 0]), triangles))
        self.assertTrue(open_worlds.triangle_hits_box(triangles, np.array([1., 0, 0]), np.eye(3), np.full(3, .1)))
        self.assertFalse(open_worlds.triangle_hits_box(triangles, np.array([2., 0, 0]), np.eye(3), np.full(3, .1)))

    def test_small_bend_has_no_avoidance_and_still_crosses_obstacle(self):
        reference = scenes.initial_trajectory(.4)
        poses = small_bend_initialization(reference)
        np.testing.assert_array_equal(poses[[0, -1]], reference[[0, -1]])
        np.testing.assert_array_equal(poses[:, 3:], 0.)
        self.assertGreater(abs(poses[:, 1]).max(), .01)
        self.assertLessEqual(abs(poses[:, 1]).max(), .025)
        obstacle = scenes.tall_obstacle_scene().truth_surface
        middle = poses[len(poses) // 2]
        self.assertLess(core.obb_signed_distance(obstacle, middle[:3], np.eye(3), np.array([.375, .115, .07]))[0].min(), -.005)

    def test_indexed_validation_equals_full_cloud_including_rotations_and_interior(self):
        rng = np.random.default_rng(19)
        truth = rng.uniform(-1., 1., (7000, 3))
        poses = np.array([[-.3, .1, .4, .2, -.1, .3], [.4, -.1, .3, -.2, .2, -.3]])
        half = np.array([.375, .115, .07])
        result = experiment.dense_validation(poses, truth, half, 20, 'so3', .004, 'imported_collision_mesh')
        expected = [core.obb_signed_distance(truth, p[:3], core.rpy_matrix(p[3:]), half)[0].min()
                    for p in result['dense_poses']]
        np.testing.assert_allclose(result['dense_clearance'], expected, rtol=0, atol=1e-12)
        self.assertEqual(result['surface_cover_radius_m'], .004)
        self.assertEqual(result['surface_kind'], 'imported_collision_mesh')


if __name__ == '__main__':
    unittest.main()
