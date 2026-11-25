"""Disabling the center check leaves selection, rank and RMSE checks intact."""
from dataclasses import replace
import unittest
import numpy as np
from fsgp_bgk_repro.terrain_assessment import AssessmentConfig,fit_pools,window_pools


class CenterOffsetAblationTests(unittest.TestCase):
    def setUp(self):
        x,y=np.meshgrid(np.linspace(-.06,.06,9),np.linspace(-.06,.06,9))
        self.xy=np.c_[x.ravel(),y.ravel()];self.seeds=np.zeros((1,3))
        self.cfg=replace(AssessmentConfig(),minimum_points=15,max_rmse_m=.008)
        self.off=replace(self.cfg,center_offset_check=False)

    def test_only_center_validity_changes_on_a_displaced_plane(self):
        p=np.c_[self.xy,np.full(len(self.xy),.08)];pools=[np.arange(len(p))]
        before=fit_pools(p,self.seeds,pools,self.cfg);after=fit_pools(p,self.seeds,pools,self.off)
        self.assertTrue(np.isnan(before['height'][0]));self.assertAlmostEqual(after['height'][0],.08,places=8)
        for key in ('coefficient','covariance','rmse','max_error','counts','full_rank'):
            np.testing.assert_array_equal(before[key],after[key])
        self.assertEqual(len(window_pools(p,self.seeds,self.cfg)[0]),0)
        self.assertEqual(len(window_pools(p,self.seeds,self.off)[0]),0)

    def test_rmse_limit_still_rejects_a_bad_quadratic(self):
        z=.08+.025*(-1.)**np.arange(len(self.xy))
        p=np.c_[self.xy,z];f=fit_pools(p,self.seeds,[np.arange(len(p))],self.off)
        self.assertGreater(f['rmse'][0],.008);self.assertTrue(np.isnan(f['height'][0]))

    def test_rank_and_minimum_count_checks_remain(self):
        p=np.c_[np.linspace(-.07,.07,20),np.zeros(20),np.full(20,.08)]
        f=fit_pools(p,self.seeds,[np.arange(len(p))],self.off)
        self.assertFalse(f['full_rank'][0]);self.assertTrue(np.isnan(f['height'][0]))
        p=np.c_[self.xy[:14],np.full(14,.08)]
        f=fit_pools(p,self.seeds,[np.arange(len(p))],self.off)
        self.assertTrue(np.isnan(f['rmse'][0]));self.assertTrue(np.isnan(f['height'][0]))


if __name__=='__main__':unittest.main()
