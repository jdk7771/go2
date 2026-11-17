import unittest
import numpy as np
from fsgp_bgk_repro.analyze_table_local_errors import true_surface_distance, subtract_hole
from fsgp_bgk_repro.table_scene import make_cloud


class AnalyticSurfaceTests(unittest.TestCase):
    def test_all_generated_exposed_surfaces_have_zero_distance(self):
        points, _, _ = make_cloud(.08)
        self.assertLess(true_surface_distance(points).max(), 2e-6)

    def test_internal_contacts_are_not_counted_as_exposed_surfaces(self):
        points = np.array([[1.32, .82, 0.], [1.32, .82, 1.1], [1.32, .82, .5], [0., 0., 1.15]])
        np.testing.assert_allclose(true_surface_distance(points), [.06, .06, .06, .05], atol=1e-12)

    def test_finite_faces_and_known_offsets(self):
        points = np.array([[0., 0., .01], [0., 0., -.02], [4., 0., 0.], [0., 0., .5]])
        np.testing.assert_allclose(true_surface_distance(points), [.01, .02, 1., .5], atol=1e-12)

    def test_contact_cut_preserves_remaining_area_without_overlap(self):
        pieces = subtract_hole((0., 0., 2., 2.), (.5, .5, 1.5, 1.5))
        self.assertAlmostEqual(sum((c-a)*(d-b) for a,b,c,d in pieces), 3.)
        for i, r in enumerate(pieces):
            for q in pieces[i+1:]:
                self.assertFalse(min(r[2],q[2])>max(r[0],q[0]) and min(r[3],q[3])>max(r[1],q[1]))


if __name__ == "__main__":
    unittest.main()
