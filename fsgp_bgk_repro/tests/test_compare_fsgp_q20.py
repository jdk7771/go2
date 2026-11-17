import unittest
import numpy as np
from fsgp_bgk_repro.compare_fsgp_q20 import reference,auc_ap,gp_predict,sensor,metric
from fsgp_bgk_repro.core import Config,TraversabilityMapper


class ComparisonTests(unittest.TestCase):
    def test_gp_adapter_exactly_matches_existing_cpu_reference(self):
        rng=np.random.default_rng(81)
        xy=rng.uniform(-2.8,2.8,(350,2));points=np.column_stack((xy,.2*xy[:,0]+.03*xy[:,1]))
        cfg=Config(inducing_points=125,seed=11)
        current=TraversabilityMapper(cfg).assess(points)
        result=gp_predict(points,current['xy'],cfg)
        for key in ('height','variance','cost'):
            np.testing.assert_allclose(result[key],current[key].ravel(),atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(result['gradient'],axis=1),current['slope'],atol=1e-12)
        np.testing.assert_array_equal(result['known'],current['observed'])

    def test_ties_and_single_class_are_not_artificially_perfect(self):
        self.assertEqual(auc_ap([True,False],[1,1])['auc'],.5)
        self.assertEqual(auc_ap([True,False],[1,1])['ap'],.5)
        self.assertEqual(auc_ap([True,False],[2,1])['auc'],1)
        self.assertEqual(auc_ap([True,False],[0,1])['auc'],0)
        self.assertIsNone(auc_ap([False,False],[1,2])['auc'])

    def test_analytic_surfaces_and_independent_body_constraints(self):
        xy=np.array([[-2.25,0],[-1.75,0],[0,0],[0,2.]])
        np.testing.assert_allclose(reference('steps',xy)['height'],[.26,.38,1,0])
        for name,inside in [('corridor-040',True),('corridor-060',True),('corridor-080',False)]:
            r=reference(name,np.array([[0.,0],[0,1.2],[0,1.6]]))
            np.testing.assert_array_equal(r['hazard'],[inside,True,False])
        self.assertFalse(reference('steps-single',np.array([[.15,0]]))['height_mask'][0])

    def test_hill_train_and_targets_are_disjoint_raw_indices(self):
        x,y=np.meshgrid(np.linspace(-2.9,2.9,151),np.linspace(-2.9,2.9,151))
        clean=np.column_stack((x.ravel(),y.ravel(),.1*x.ravel()**2)).astype('<f4')
        xy,_,_=TraversabilityMapper().grid((0,0))
        points,query,truth,train_ids,test_ids,_=sensor('hill',clean,'dense_low',1101,xy)
        self.assertEqual(np.intersect1d(train_ids,test_ids).size,0)
        np.testing.assert_array_equal(query,clean[test_ids,:2])
        np.testing.assert_array_equal(truth['height'],clean[test_ids,2])
        self.assertFalse(truth['risk_mask'].any())
        self.assertTrue(np.isnan(truth['gradient']).all())

    def test_unknown_predictions_do_not_score_as_zero_error(self):
        result=dict(height=np.zeros(2),gradient=np.zeros((2,2)),variance=np.ones(2),known=np.zeros(2,bool))
        truth=dict(height=np.ones(2),gradient=np.zeros((2,2)))
        values=metric(result,truth,np.ones(2,bool))
        self.assertEqual(values['n'],0);self.assertIsNone(values['rmse_m'])


if __name__=='__main__':unittest.main()
