import tempfile
from pathlib import Path
import unittest
import numpy as np

from quadric_lidar_repro.core import (Config, Moments, QuadricMap, basis, distance_and_gradient,
    fit_patch, load_map, polynomial_transform, pose_jacobian, register, se3_exp, transform,
    associate, select_source)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(4)
        self.config = Config()

    def test_polynomial_transform(self):
        points = self.rng.normal(size=(200, 3))
        pose = se3_exp([.3,-.8,1.2,.2,-.1,.3])
        l = polynomial_transform(pose[:3,:3],pose[:3,3])
        np.testing.assert_allclose(basis(transform(points,pose)),basis(points)@l.T,atol=1e-13)

    def test_incremental_moments_are_batch_equivalent(self):
        a, b = self.rng.normal(size=(350,3)), self.rng.normal(size=(280,3))
        pose=se3_exp([1.1,.4,-.2,.2,.1,-.1])
        actual=Moments.from_points(a)
        actual.merge(Moments.from_points(b).transformed(pose))
        all_points=np.vstack([a,transform(b,pose)])
        q=basis(all_points-actual.origin)
        np.testing.assert_allclose(actual.gram,q.T@q,atol=2e-10)
        np.testing.assert_allclose(actual.mean,all_points.mean(axis=0),atol=1e-12)
        np.testing.assert_allclose(actual.covariance,np.cov(all_points,rowvar=False,bias=True),atol=1e-12)
        self.assertEqual(actual.count,len(all_points))

    def test_plane_distance(self):
        p=self.rng.uniform(-2,2,(400,3));p[:,2]=.7
        patch=fit_patch(Moments.from_points(p),self.config)
        self.assertEqual(patch.kind,'plane')
        d,_=distance_and_gradient(patch,np.array([[0,0,1.0],[0,0,.7]]))
        np.testing.assert_allclose(d,[.09,0],atol=1e-12)

    def test_sphere_and_analytic_jacobians(self):
        p=self.rng.normal(size=(1500,3));p=1.7*p/np.linalg.norm(p,axis=1,keepdims=True)+[1,.5,-.3]
        patch=fit_patch(Moments.from_points(p),self.config)
        self.assertEqual(patch.kind,'quadric')
        d,_=distance_and_gradient(patch,p)
        self.assertLess(d.max(),1e-18)
        q=np.array([[2.9,.7,-.2],[1.1,2.6,.3]])
        residual,gradient=distance_and_gradient(patch,q)
        eps=1e-6
        numeric=np.column_stack([(distance_and_gradient(patch,q+np.eye(3)[k]*eps)[0]
                                 -distance_and_gradient(patch,q-np.eye(3)[k]*eps)[0])/(2*eps) for k in range(3)])
        np.testing.assert_allclose(gradient,numeric,atol=1e-8,rtol=1e-6)
        jac=pose_jacobian(q,gradient)
        numeric=np.column_stack([(distance_and_gradient(patch,transform(q,se3_exp(np.eye(6)[k]*eps)))[0]
                                 -distance_and_gradient(patch,transform(q,se3_exp(-np.eye(6)[k]*eps)))[0])/(2*eps) for k in range(6)])
        np.testing.assert_allclose(jac,numeric,atol=1e-8,rtol=1e-6)

    def test_distribution_jacobian(self):
        p=self.rng.normal(size=(300,3))
        patch=fit_patch(Moments.from_points(p),Config(mse_threshold=1e-12))
        self.assertEqual(patch.kind,'distribution')
        q=np.array([[.1,.3,.8]])
        _,gradient=distance_and_gradient(patch,q)
        eps=1e-6
        numeric=np.column_stack([(distance_and_gradient(patch,q+np.eye(3)[k]*eps)[0]
                                 -distance_and_gradient(patch,q-np.eye(3)[k]*eps)[0])/(2*eps) for k in range(3)])
        np.testing.assert_allclose(gradient,numeric,atol=1e-8)

    def test_registration_and_map_roundtrip(self):
        lists=[]
        for axis,value in [(2,-1),(0,3),(1,2)]:
            p=self.rng.uniform(-2,2,(350,3));p[:,axis]=value;lists.append(p)
        sphere=self.rng.normal(size=(450,3));sphere=.7*sphere/np.linalg.norm(sphere,axis=1,keepdims=True)+[-2,-2,1]
        lists.append(sphere)
        target=[fit_patch(Moments.from_points(p),self.config) for p in lists]
        truth=se3_exp([.13,-.08,.05,.03,-.02,.04])
        source=[transform(p,np.linalg.inv(truth)) for p in lists]
        patches=[fit_patch(Moments.from_points(p),self.config) for p in source]
        estimate,stats=register(patches,source,target,np.eye(4),self.config)
        np.testing.assert_allclose(estimate,truth,atol=3e-4)
        self.assertGreater(stats['accepted_steps'],0)
        mapping=QuadricMap(self.config);mapping.active=target
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'map.npz';mapping.save(path)
            restored=load_map(path,self.config)
            self.assertEqual(len(restored),len(target))
            for a,b in zip(target,restored):
                np.testing.assert_allclose(a.coefficients,b.coefficients,atol=1e-8)
                np.testing.assert_allclose(a.moments.gram,b.moments.gram)

    def test_small_budget_remainder(self):
        p=self.rng.normal(size=(40,3));p[:,2]=0
        patch=fit_patch(Moments.from_points(p),self.config)
        config=Config(point_budget=11,points_per_patch=10)
        selected=select_source([patch,patch],[p,p],config)
        self.assertEqual(len(selected),1)
        self.assertEqual(len(selected[0][1]),10)
        self.assertEqual(associate(p[:1],[patch],config),0)


if __name__=='__main__':
    unittest.main()
