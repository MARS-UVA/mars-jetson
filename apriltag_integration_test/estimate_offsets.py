"""Conditional PnP offsets using one known-pose clip as an intrinsic reference.

Run after run.sh: python3 estimate_offsets.py results/<run-directory>
This does not replace measured calibration or independently validate its reference.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from analysis import ground_truth, pose_errors

ROOT = Path(__file__).resolve().parent


def object_points(size):
    half = size / 2
    return np.array([[-half, -half, 0], [half, -half, 0],
                     [half, half, 0], [-half, half, 0]], dtype=np.float64)


def estimate_intrinsics(points, translation, rotation, mean_corners):
    projected = points @ rotation.T + translation
    if np.any(projected[:, 2] <= 0):
        raise ValueError('Reference corners must be in front of camera')
    coefficients = []
    for axis in range(2):
        design = np.column_stack((projected[:, axis] / projected[:, 2], np.ones(4)))
        fit, _, rank, _ = np.linalg.lstsq(design, mean_corners[:, axis], rcond=None)
        if rank < 2 or fit[0] <= 0 or not np.isfinite(fit).all():
            raise ValueError('Reference does not determine positive focal lengths')
        coefficients.append(fit)
    return np.array([[coefficients[0][0], 0, coefficients[0][1]],
                     [0, coefficients[1][0], coefficients[1][1]], [0, 0, 1.]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--reference-distance', type=float, default=1.0)
    args = parser.parse_args()
    source = args.results.resolve()
    if not source.is_relative_to(ROOT / 'results'):
        parser.error('Results must be under apriltag_integration_test/results')
    data = []
    for folder in sorted(source.iterdir()):
        if not (folder / 'frames.csv').is_file():
            continue
        config = yaml.safe_load((folder / 'setup.yaml').read_text())
        if len(config['tags']) != 1:
            raise ValueError('This estimator expects one configured tag per case')
        tag = config['tags'][0]
        with (folder / 'frames.csv').open() as stream:
            rows = [r for r in csv.DictReader(stream) if r.get('decision_margin')]
        if not rows:
            raise ValueError(f'No detections in {folder}')
        corners = np.array([[[float(r[f'corner_{i}_x_px']), float(r[f'corner_{i}_y_px'])]
                             for i in range(4)] for r in rows])
        data.append((folder, tag, rows, corners, config))
    reference = [d for d in data if np.isclose(ground_truth(d[1]['ground_truth'])[0][2], args.reference_distance)]
    if len(reference) != 1:
        raise ValueError('Need exactly one case at reference-distance')
    ref = reference[0]
    k = estimate_intrinsics(object_points(ref[1]['size_m']), *ground_truth(ref[1]['ground_truth']),
                            ref[3].mean(axis=0))
    output = source / 'inferred_offsets'
    output.mkdir()
    summaries = []
    for folder, tag, rows, corners, config in data:
        if config['camera'] != ref[4]['camera']:
            raise ValueError('Reference and comparison cases must use the same camera configuration')
        translation, rotation = ground_truth(tag['ground_truth'])
        records = []
        for row, points in zip(rows, corners):
            ok, rvec, tvec = cv2.solvePnP(object_points(tag['size_m']), points, k, np.zeros(5))
            if not ok:
                raise RuntimeError(f'PnP failed for {folder.name}, frame {row["frame"]}')
            estimate_rotation, _ = cv2.Rodrigues(rvec)
            records.append({'frame': int(row['frame']), 'video_time_s': float(row['video_time_s']),
                            **pose_errors(tvec.ravel(), estimate_rotation, translation, rotation)})
        with (output / (folder.name + '_apparent_offsets.csv')).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        widths = (np.linalg.norm(corners[:, 1]-corners[:, 0], axis=1)
                  + np.linalg.norm(corners[:, 2]-corners[:, 3], axis=1)) / 2
        summary = {'distance_label_m': float(translation[2]), 'frames': len(records),
                   'calibration_role': 'reference_fit' if folder == ref[0] else 'comparison_using_reference',
                   'mean_tag_width_px': float(widths.mean()),
                   'implied_fx_from_known_distance_px': float(widths.mean()*translation[2]/tag['size_m']),
                   'validity': 'conditional estimate, not independent measured-calibration accuracy'}
        for column, metric in [('mean_estimated_depth_m', 'tag_z_m'), ('mean_depth_offset_m', 'depth_error_m'),
                               ('mean_tag_angle_offset_deg', 'angle_error_deg'),
                               ('mean_rotation_error_deg', 'rotation_error_deg'),
                               ('mean_position_error_m', 'position_error_m')]:
            summary[column] = float(np.mean([r[metric] for r in records]))
        summaries.append(summary)
    with (output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    metadata = {'reference_clip': str(ref[0]), 'upright_camera_matrix': k.tolist(),
                'distortion_assumption': [0.]*5,
                'limitations': ['Reference clip is calibration data, not an independent accuracy test',
                                'Known tag size and full configured poses assumed exact',
                                'Same intrinsics across clips and zero residual distortion assumed',
                                'Inferred calibration does not replace measured video-mode calibration']}
    (output / 'calibration_assumptions.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps({'output': str(output), 'summary': summaries}, indent=2))


if __name__ == '__main__':
    main()
