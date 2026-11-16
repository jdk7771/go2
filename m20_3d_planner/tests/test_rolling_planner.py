import unittest

import numpy as np

from m20_3d_planner.rolling_planner import (
    LocalCollision, RollingConfig, checked_prefix, interpolate, plan,
    rotation_distance,
)


class RollingPlannerTest(unittest.TestCase):
    def test_swept_gate_rejects_between_free_endpoints(self):
        gate = LocalCollision(np.array([[0., 0., .4]]), np.array([.1, .1, .1]), .02)
        a, b = np.array([-.5, 0, .4, 0, 0, 0]), np.array([.5, 0, .4, 0, 0, 0])
        self.assertTrue(gate.free(a))
        self.assertTrue(gate.free(b))
        self.assertFalse(gate.edge(a, b))
        self.assertIsNone(checked_prefix(np.array([a, b]), gate, RollingConfig(advance=.6)))

    def test_prefix_caps_translation_and_so3_rotation(self):
        gate = LocalCollision(np.empty((0, 3)), np.array([.375, .115, .07]), .035)
        a, b = np.array([0., 0., .4, 0, 0, 0]), np.array([.5, .2, .4, .6, -.2, .1])
        config = RollingConfig()
        prefix = checked_prefix(interpolate(a, b, np.linspace(0, 1, 9)), gate, config)
        self.assertLessEqual(np.linalg.norm(np.diff(prefix[:, :3], axis=0), axis=1).max(), .010000001)
        self.assertLessEqual(max(rotation_distance(a, b) for a, b in zip(prefix[:-1], prefix[1:])), config.command_rotation+1e-10)
        self.assertLessEqual(np.linalg.norm(np.diff(prefix[:, :3], axis=0), axis=1).sum(), config.advance+1e-10)

    def test_rotation_sweep_checks_between_free_attitudes(self):
        gate = LocalCollision(np.array([[.28, .28, .4]]), np.array([.4, .05, .05]), .02)
        a = np.array([0., 0., .4, 0., 0., 0.])
        b = a.copy()
        b[5] = np.pi/2
        self.assertTrue(gate.free(a))
        self.assertTrue(gate.free(b))
        self.assertFalse(gate.edge(a, b))

    def test_start_local_then_commit_and_replan(self):
        start, goal = np.array([-1.3, 0, .4, 0, 0, 0]), np.array([1.3, 0, .4, 0, 0, 0])
        config = RollingConfig(relaxation_steps=3, max_replans=3)
        executed, records, metrics = plan(np.empty((0, 3)), start, goal, np.array([.375, .115, .07]), config)
        np.testing.assert_equal(executed[0], start)
        self.assertFalse(metrics['goal_reached'])
        self.assertLess(np.linalg.norm(executed[-1, :3]-start[:3]), .301)
        self.assertEqual(len(records), 3)
        previous_end = 0
        for record in records:
            self.assertEqual(record['start_index'], previous_end)
            current = executed[record['start_index']]
            route = np.array(record['local_reachability_route'])
            self.assertLessEqual(np.linalg.norm(route[:, :3]-current[:3], axis=1).max(), config.horizon+1e-8)
            for attempt in record['attempts']:
                np.testing.assert_allclose(attempt['initial'][0], current, atol=1e-10)
                self.assertGreater(np.linalg.norm(np.array(attempt['initial'][-1])[:3]-goal[:3]), 1.)
            previous_end = record['end_index']
        self.assertEqual(previous_end, len(executed)-1)

    def test_invalid_sensing_coverage_rejected(self):
        pose = np.array([0., 0, .4, 0, 0, 0])
        with self.assertRaises(ValueError):
            plan(np.empty((0, 3)), pose, pose+np.array([1, 0, 0, 0, 0, 0]),
                 np.array([.375, .115, .07]), RollingConfig(sensing_radius=.7))


if __name__ == '__main__':
    unittest.main()
