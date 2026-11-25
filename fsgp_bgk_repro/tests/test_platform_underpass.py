"""Geometry regression for restoring observable floor under a hollow platform."""
import unittest
import numpy as np
from fsgp_bgk_repro.platform_underpass_scene import build, HEIGHTS
from fsgp_bgk_repro.convex_surface_assessment import fresh_fixed_fit


class PlatformUnderpassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenes = {scene: build(scene) for scene in HEIGHTS}

    def test_platform_extends_to_right_edge_and_preserves_original_low_step(self):
        for points, labels, meta in self.scenes.values():
            self.assertEqual(meta['tabletop']['min'][1], meta['bounds_m'][0][1])
            self.assertEqual(meta['second_platform']['x_range_m'], [1., 2.6])
            self.assertEqual(meta['second_platform']['y_range_m'], [-4., -.9])
            direction=np.array(meta['second_platform']['extension_direction_world'])
            forward=np.array(meta['second_platform']['forward_direction_world'])
            np.testing.assert_allclose(direction,np.cross(forward,[0.,0.,1.]))
            self.assertAlmostEqual(direction@forward,0.)
            low = (points[:, 0]>-2.5)&(points[:, 0]<-1.1)&(points[:, 1]>-2.2)&(points[:, 1]<-1.)
            np.testing.assert_allclose(points[low, 2], .15, atol=1e-6)
            self.assertEqual(len(points), len(np.unique(points, axis=0)))

    def test_lower_space_has_floor_and_ceiling_but_no_fabricated_fill(self):
        for scene, (points, labels, meta) in self.scenes.items():
            middle = (points[:, 0]>1.4)&(points[:, 0]<2.2)&(points[:, 1]>-3.6)&(points[:, 1]<-1.1)
            np.testing.assert_allclose(np.unique(points[middle, 2]),
                                       [0., HEIGHTS[scene], HEIGHTS[scene]+.10], atol=1e-6)
            self.assertGreater(np.sum(middle & (labels==0)), 100)
            self.assertEqual(meta['planning_test']['goal_under_second_platform'], False)
            self.assertGreater(meta['planning_test']['goal_xy_m'][0],meta['tabletop']['max'][0]+.4)
            for leg in meta['legs']:
                inside = np.all((points[:, :2]>np.array(leg['min'][:2])+1e-6)
                                & (points[:, :2]<np.array(leg['max'][:2])-1e-6), axis=1)
                self.assertFalse(np.any(inside & (np.abs(points[:, 2])<1e-6)))

    def test_xyz_fitting_retains_all_three_observed_heights_without_part_labels(self):
        for scene, (points, _, _) in self.scenes.items():
            crop = points[(points[:, 0]>1.5)&(points[:, 0]<2.5)
                          &(points[:, 1]>-2.)&(points[:, 1]<-1.2)].astype(float)
            seeds, pools, fits, _, _, _ = fresh_fixed_fit(crop, 'q15-n15')
            active = np.flatnonzero(np.isfinite(fits['height']))
            np.testing.assert_allclose(np.unique(np.round(fits['height'][active], 5)),
                                       [0., HEIGHTS[scene], HEIGHTS[scene]+.10], atol=1e-6)
            for i in active:
                self.assertLess(np.ptp(crop[pools[i], 2]), 1e-6)


if __name__ == '__main__': unittest.main()
