import tempfile
import unittest
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis import pose_errors, rotation_rpy
from comparison import statistics, write_showcase
import csv
from matrix import generate
from estimate_offsets import estimate_intrinsics, object_points
from video import Preprocessor, annotate

ROOT = Path(__file__).resolve().parents[1]


class ExperimentTests(unittest.TestCase):
    def test_showcase_retains_bias_and_uses_absolute_angle_error(self):
        errors = pose_errors(np.array([0., 0., 1.1]), rotation_rpy([180, 5, 0]),
                             np.array([0., 0., 1.]), rotation_rpy([180, 0, 0]))
        base = {'status': 'ok', 'case': 'a', 'family': '36h11', 'distance_m': 1.,
                'angle_deg': 0., 'calibration_basis': 'manufacturer_specification_estimate',
                'decision_margin': 80., **errors}
        rows = [base, {**base, 'angle_error_deg': -5},
                {**base, 'status': 'not_detected'}]
        del rows[2]['decision_margin']
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            write_showcase(Path(temporary), rows)
            with (Path(temporary) / 'showcase.csv').open() as stream:
                record = next(csv.DictReader(stream))
        self.assertEqual(record['Samples'], '3')
        self.assertEqual(record['Detection (%)'], '66.67')
        self.assertEqual(record['Mean depth offset (cm)'], '10.0')
        self.assertEqual(record['Mean absolute angle error (deg)'], '5.0')
        self.assertEqual(record['Calibration'], 'manufacturer_specification_estimate')

    def test_intrinsic_estimate_from_known_reference_pose(self):
        points = object_points(.205)
        translation = np.array([.05, -.1075, 1.])
        rotation = rotation_rpy([180, 0, 0])
        expected = np.array([[900., 0, 601.], [0, 910., 405.], [0, 0, 1.]])
        camera_points = points @ rotation.T + translation
        pixels = camera_points @ expected.T
        pixels = pixels[:, :2] / pixels[:, 2:]
        np.testing.assert_allclose(estimate_intrinsics(points, translation, rotation, pixels), expected, atol=1e-9)

    def test_rotation_without_calibration(self):
        processor = Preprocessor({'width': 6, 'height': 4, 'rotation_degrees': 180}, detection_only=True)
        source = np.arange(72, dtype=np.uint8).reshape(4, 6, 3)
        np.testing.assert_array_equal(processor.process(source), source[::-1, ::-1])
        self.assertIsNone(processor.k)

    def test_rotated_principal_point(self):
        processor = Preprocessor({'width': 640, 'height': 480, 'fx': 600., 'fy': 600.,
                                  'cx': 300., 'cy': 200., 'rotation_degrees': 180,
                                  'already_rectified': True})
        self.assertEqual(processor.k[0, 2], 339.)
        self.assertEqual(processor.k[1, 2], 279.)

    def test_calibration_scales_with_video(self):
        camera = {'width': 1280, 'height': 960, 'fx': 1200, 'fy': 1200, 'cx': 640,
                  'cy': 480, 'output_width': 640, 'output_height': 480}
        processor = Preprocessor(camera)
        np.testing.assert_allclose(processor.k, [[600, 0, 320], [0, 600, 240], [0, 0, 1]])
        self.assertEqual(processor.process(np.zeros((960, 1280, 3), np.uint8)).shape, (480, 640, 3))
        with self.assertRaises(ValueError):
            Preprocessor({**camera, 'output_height': 360})

    def test_matrix_counts_alias_and_physical_distances(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            folder = Path(temporary)
            plan = yaml.safe_load((ROOT / 'dorm.example.yaml').read_text())
            plan['measured_distances_m'] = [.25, .5, 1, 1.5]
            plan['tag_sizes_m'] = {'41h12': .15}
            (folder / 'camera.yaml').write_text('fx: null\n')
            path = folder / 'plan.yaml'
            path.write_text(yaml.safe_dump(plan))

            def inside(path):
                path = path.resolve()
                self.assertTrue(path.is_relative_to(ROOT))
                return path

            generate(path, inside)
            cases = [yaml.safe_load(p.read_text()) for p in folder.rglob('setup.yaml')]
            self.assertEqual(len(cases), 36)
            self.assertEqual(sum(c['sampling']['required_frames'] for c in cases), 10800)
            self.assertEqual(sum(c['detector']['family'] == 'Standard41h12' for c in cases), 12)
            for case in cases:
                self.assertEqual(case['tags'][0]['size_m'],
                                 .15 if case['detector']['family'] == 'Standard41h12' else .205)
                self.assertAlmostEqual(case['tags'][0]['ground_truth']['offset_y_m'], -.1075)
                self.assertEqual(case['tags'][0]['ground_truth']['distance_m'],
                                 case['experiment']['nominal_distance_m'] / 2)
                self.assertEqual(case['detector']['max_hamming'], 0)
            with self.assertRaises(ValueError):
                generate(path, inside)

    def test_grouped_metrics_use_failures_in_denominator(self):
        errors = pose_errors(np.array([0., 0., 1.1]), rotation_rpy([180, 15, 0]),
                             np.array([0., 0., 1.]), rotation_rpy([180, 10, 0]))
        rows = [{'status': 'ok', 'case': 'a', 'decision_margin': 80., **errors},
                {'status': 'not_detected', 'case': 'a'}]
        stats = statistics(rows)
        self.assertEqual(stats['detection_success_pct'], 50)
        self.assertEqual(stats['mean_decision_margin'], 80)
        self.assertAlmostEqual(stats['angle_error_deg_mean'], 5)
        self.assertAlmostEqual(stats['position_error_m_mse'], errors['position_error_m']**2)

    def test_annotation_only_draws_relevant_actual_detections(self):
        points = [SimpleNamespace(x=x, y=y) for x, y in [(50, 60), (150, 60), (150, 160), (50, 160)]]
        detection = SimpleNamespace(id=1, family='tag36h11', corners=points, decision_margin=100.)
        source = np.zeros((240, 320, 3), np.uint8)
        relevant = annotate(source, [detection], [{'tag_id': 1, 'status': 'ok'}], 0, 0)
        unrelated = annotate(source, [detection], [{'tag_id': 2, 'status': 'not_detected'}], 0, 0)
        self.assertGreater(int(relevant[60, 100, 1]), 100)
        self.assertEqual(int(unrelated[60, 100, 1]), 0)
        self.assertFalse(source.any())


if __name__ == '__main__':
    unittest.main()
