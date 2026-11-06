import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis import ground_truth, pose_errors, rotation_rpy, summarize, orientation_axis_errors


class PoseTests(unittest.TestCase):
    def test_horizontal_and_vertical_tilts_are_separate(self):
        base = rotation_rpy([180, 0, 0])
        for rpy, h, v in [([180, 15, 0], 15, 0), ([190, 0, 0], 0, 10),
                           ([190, 20, 0], 20, 10), ([170, -25, 0], -25, -10)]:
            errors = orientation_axis_errors(rotation_rpy(rpy), base)
            self.assertAlmostEqual(errors['horizontal_angle_error_deg'], h)
            self.assertAlmostEqual(errors['vertical_angle_error_deg'], v)

    def test_angles_compare_against_nonzero_truth_and_exclude_roll(self):
        rotated = rotation_rpy([190, 20, 0])
        errors = orientation_axis_errors(rotated, rotated)
        self.assertAlmostEqual(errors['horizontal_angle_error_deg'], 0)
        self.assertAlmostEqual(errors['vertical_angle_error_deg'], 0)
        errors = orientation_axis_errors(rotation_rpy([180, 0, 30]), rotation_rpy([180, 0, 0]))
        self.assertAlmostEqual(errors['horizontal_angle_error_deg'], 0)
        self.assertAlmostEqual(errors['vertical_angle_error_deg'], 0)

    def test_detection_only_does_not_claim_pose_accuracy(self):
        rows = [{'tag_id': 1, 'status': 'detected_no_calibration', 'decision_margin': 80.},
                {'tag_id': 1, 'status': 'not_detected'}]
        report = summarize(rows, [{'id': 1}], {'min_detection_rate': .4}, detection_only=True)
        self.assertTrue(report['passed'])
        self.assertEqual(report['tags']['1']['detection_rate'], .5)
        self.assertIsNone(report['tags']['1']['pose_success_rate'])
        self.assertIsNone(report['tags']['1']['position_error_m'])
    def test_perfect_oblique_pose(self):
        t, rotation = ground_truth({'translation_m': [0.2, -0.1, 2], 'rpy_deg': [10, 30, -20]})
        errors = pose_errors(t, rotation, t, rotation)
        self.assertAlmostEqual(errors['position_error_m'], 0)
        self.assertAlmostEqual(errors['rotation_error_deg'], 0, places=5)

    def test_inverse_pose_and_rotation_error(self):
        errors = pose_errors(np.array([0, 0, 2.0]), rotation_rpy([0, 90, 0]),
                             np.array([0, 0, 2.0]), np.eye(3))
        self.assertAlmostEqual(errors['tag_position_error_m'], 0)
        self.assertAlmostEqual(errors['rotation_error_deg'], 90)
        self.assertAlmostEqual(errors['position_error_m'], np.sqrt(8))
        self.assertAlmostEqual(errors['camera_x_m'], 2)

    def test_misses_stay_in_denominator(self):
        rows = [{'tag_id': 1, 'status': 'not_detected'} for _ in range(5)]
        report = summarize(rows, [{'id': 1}], {})
        self.assertFalse(report['passed'])
        self.assertEqual(report['tags']['1']['pose_success_rate'], 0)
        self.assertIsNone(report['tags']['1']['position_error_m'])

    def test_invalid_and_ambiguous_truth(self):
        for spec in ({'distance_m': 1, 'angle_deg': 0},
                     {'translation_m': [0, 0, 1], 'rpy_deg': [0, 0, 0], 'angle_deg': 0},
                     {'translation_m': [0, 0, float('nan')], 'rpy_deg': [0, 0, 0]}):
            with self.assertRaises(ValueError):
                ground_truth(spec)

    def test_upright_shorthand_matches_native_pnp_frame(self):
        t, rotation = ground_truth({'offset_x_m': .1, 'offset_y_m': -.2,
                                    'distance_m': 1.5, 'angle_deg': 25})
        np.testing.assert_allclose(t, [.1, -.2, 1.5])
        np.testing.assert_allclose(rotation, rotation_rpy([0, 25, 0]) @ np.diag([1, -1, -1]), atol=1e-15)


if __name__ == '__main__':
    unittest.main()
