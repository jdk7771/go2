from dataclasses import replace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from quadric_lidar_repro.core import Config, Moments, Patch, fit_patch
from quadric_lidar_repro.curvature import differential_geometry, local_geometry, tangent_basis


def exact_patch(coefficients, kind="quadric"):
    return Patch(Moments(np.zeros(3), np.eye(10), 1), np.asarray(coefficients, dtype=float),
                 0., kind, np.eye(3), 2.)


class CurvatureTests(unittest.TestCase):
    def test_sphere_closed_form_and_projection(self):
        patch = exact_patch([1, 1, 1, 0, 0, 0, 0, 0, 0, -4])
        p = np.array([[0, 0, 2.1], [2.05, 0, 0], [0, -2.03, 0]])
        result = differential_geometry(patch, p)
        self.assertTrue(result["valid"].all())
        np.testing.assert_allclose(np.linalg.norm(result["points"], axis=1), 2, atol=1e-10)
        np.testing.assert_allclose(result["principal"], -.5, atol=1e-10)
        np.testing.assert_allclose(result["gaussian"], .25, atol=1e-10)

    def test_plane_cylinder_saddle_closed_forms(self):
        cases = [(exact_patch([0,0,0,0,0,0,-.3,.2,1,0], "plane"), [.2,.4,-.02], [0,0]),
                 (exact_patch([1,0,1,0,0,0,0,0,0,-4]), [0,1,2], [-.5,0]),
                 (exact_patch([-.35,.25,0,0,0,0,0,0,1,0]), [0,0,0], [-.5,.7])]
        for patch, p, expected in cases:
            result = differential_geometry(patch, np.array([p]))
            self.assertTrue(result["valid"][0])
            np.testing.assert_allclose(result["principal"][0], expected, atol=1e-10)
        self.assertAlmostEqual(differential_geometry(cases[0][0], [[0,0,0]])["slope_deg"][0],
                               np.degrees(np.arctan(np.hypot(.3,.2))))

    def test_polynomial_scale_and_normal_orientation(self):
        patch = exact_patch([1,1,1,0,0,0,0,0,0,-4])
        p = np.array([[0,0,2.01]])
        expected = differential_geometry(patch, p)
        for factor in (1e-12, 1e12, -5):
            actual = differential_geometry(replace(patch, coefficients=patch.coefficients*factor), p)
            np.testing.assert_allclose(actual["principal"], expected["principal"]*np.sign(factor), atol=1e-12)
            np.testing.assert_allclose(actual["gaussian"], expected["gaussian"], atol=1e-12)

    def test_singular_and_distribution_have_no_geometry(self):
        for patch in (exact_patch([1,1,1,0,0,0,0,0,0,0]),
                      exact_patch(np.zeros(10), "distribution")):
            result = differential_geometry(patch, [[0,0,0]])
            self.assertFalse(result["valid"][0])
            self.assertTrue(np.isnan(result["principal"]).all())

    def test_shape_operator_against_finite_difference_of_normal(self):
        patch = exact_patch([-.35,.25,0,0,0,0,0,0,1,0])
        p = np.array([[.3,-.2,.35*.3**2-.25*.2**2]])
        result = differential_geometry(patch, p)
        first, second = tangent_basis(result["normal"])
        tangent = np.column_stack((first[0], second[0]))
        eps = 1e-5
        derivatives = []
        for direction in tangent.T:
            plus = differential_geometry(patch, p+eps*direction)["normal"][0]
            minus = differential_geometry(patch, p-eps*direction)["normal"][0]
            derivatives.append((plus-minus)/(2*eps))
        shape = -tangent.T@np.column_stack(derivatives)
        np.testing.assert_allclose(np.linalg.eigvalsh((shape+shape.T)/2), result["principal"][0], atol=1e-9)

    def test_fit_and_curvature_are_rigid_transform_invariant(self):
        rng = np.random.default_rng(7)
        points = rng.normal(size=(900,3)); points *= 2/np.linalg.norm(points, axis=1, keepdims=True)
        rotation = Rotation.from_rotvec([.5,-.2,.7]).as_matrix()
        transformed = points@rotation.T+[14,-8,1]
        patch = fit_patch(Moments.from_points(transformed), Config())
        result = differential_geometry(patch, transformed[::20])
        self.assertTrue(result["valid"].all())
        np.testing.assert_allclose(np.abs(result["principal"]), .5, atol=1e-9)

    def test_local_jet_recovers_saddle_curvature_at_origin(self):
        u,v = np.meshgrid(np.linspace(-.3,.3,25),np.linspace(-.3,.3,25))
        points = np.column_stack((u.ravel(),v.ravel(),(.35*u*u-.25*v*v).ravel()))
        result = local_geometry(points, [[0,0,0]], neighbors=len(points))
        principal = result["principal"][0]
        if result["normal"][0,2]<0:
            principal = -principal[::-1]
        np.testing.assert_allclose(principal, [-.5,.7], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
