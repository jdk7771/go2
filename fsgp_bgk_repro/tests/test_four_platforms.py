import unittest
import numpy as np
from fsgp_bgk_repro.four_platforms_scene import build
from fsgp_bgk_repro.convex_surface_assessment import fresh_fixed_fit
from fsgp_bgk_repro.four_platforms_audit import body_intersects_solid


class FourPlatformsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.points,cls.labels,cls.meta=build()

    def test_five_conditions_and_full_width_barriers(self):
        self.assertEqual(len(self.meta['platforms']),5)
        for s in self.meta['platforms']:
            self.assertEqual(s['tabletop']['min'][1],-4.)
            self.assertAlmostEqual(s['tabletop']['max'][0]-s['tabletop']['min'][0],1.6)
        a,b,c,d,e=self.meta['platforms']
        self.assertTrue(a['solid']);self.assertAlmostEqual(b['clearance_m'],.70)
        self.assertAlmostEqual(c['clearance_m'],.42);self.assertLess(d['clearance_m'],.415)
        self.assertLess(d['top_height_m'],.40);self.assertGreater(e['top_height_m'],.40)
        self.assertLess(e['clearance_m'],.415)
        self.assertLess(e['top_height_m']-e['clearance_m'],.415)
        for s in (b,c,d):self.assertEqual(s['y_range_m'],[-4.,4.])
        self.assertEqual(e['y_range_m'],[-4.,2.8]);self.assertAlmostEqual(self.meta['fifth_bypass']['width_m'],1.2)

    def test_solid_has_no_ground_but_all_hollow_platforms_have_observed_ground(self):
        p=self.points
        for s in self.meta['platforms']:
            xy=np.array(s['center_xy_m']);middle=np.all(np.abs(p[:,:2]-xy)<.15,axis=1)
            expected=[s['top_height_m']] if s['solid'] else [0.,s['clearance_m'],s['top_height_m']]
            np.testing.assert_allclose(np.unique(p[middle,2]),expected,atol=1e-6)
        self.assertEqual(len(p),len(np.unique(p,axis=0)))

    def test_fits_separate_38cm_ceiling_from_ground_using_xyz_only(self):
        p=self.points;take=(np.abs(p[:,0]-7.2)<.45)&(np.abs(p[:,1]+2.4)<.4)
        seeds,pools,fits,*_=fresh_fixed_fit(p[take].astype(float),'q15-n15')
        active=np.isfinite(fits['height'])
        np.testing.assert_allclose(np.unique(np.round(fits['height'][active],5)),[0.,.38,.78],atol=1e-6)

    def test_body_inside_thick_slab_is_not_free_space(self):
        slab=dict(min=[-1.,-4.,.38],max=[1.,4.,.80])
        frame=dict(body=[0.,0.,.70],rotation=np.eye(3))
        self.assertTrue(body_intersects_solid(frame,slab,[.375,.115,.07],.025))
        frame['body']=[0.,0.,1.20]
        self.assertFalse(body_intersects_solid(frame,slab,[.375,.115,.07],.025))


if __name__=='__main__':unittest.main()
