"""Pose math and reports; no ROS or robot workspace dependencies."""
import csv
import json
from pathlib import Path

import numpy as np


def rotation_rpy(degrees):
    r, p, y = np.deg2rad(degrees)
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def ground_truth(spec):
    if 'translation_m' in spec:
        if set(spec) != {'translation_m', 'rpy_deg'}:
            raise ValueError('Full ground_truth requires only translation_m and rpy_deg')
        t, rpy = spec['translation_m'], spec['rpy_deg']
    else:
        if set(spec) != {'offset_x_m', 'offset_y_m', 'distance_m', 'angle_deg'}:
            raise ValueError('ground_truth requires offsets, distance_m and angle_deg')
        t = [spec['offset_x_m'], spec['offset_y_m'], spec['distance_m']]
        # Native upright AprilTag PnP frame: X right, Y up, Z toward viewer.
        rpy = [180, spec['angle_deg'], 0]
    t, rpy = np.asarray(t, dtype=float), np.asarray(rpy, dtype=float)
    if t.shape != (3,) or rpy.shape != (3,) or not np.isfinite([t, rpy]).all() or t[2] <= 0:
        raise ValueError('ground_truth must have finite 3D translation/RPY and positive depth')
    return t, rotation_rpy(rpy)


def quaternion_rotation(q):
    q = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def orientation_axis_errors(rotation, true_rotation):
    """Tag-normal azimuth (right positive) and elevation (up positive), in degrees.

    Native PnP tag +Z faces the viewer. Negate it to get the forward normal,
    which is optical +Z for an upright, front-facing tag. In-plane roll is excluded.
    """
    def angles(r):
        normal = -np.asarray(r)[:, 2]
        horizontal = np.rad2deg(np.arctan2(normal[0], normal[2]))
        vertical = np.rad2deg(np.arctan2(-normal[1], np.hypot(normal[0], normal[2])))
        return float(horizontal), float(vertical)

    horizontal, vertical = angles(rotation)
    true_horizontal, true_vertical = angles(true_rotation)
    wrap = lambda value: (value + 180) % 360 - 180
    return {'estimated_horizontal_angle_deg': horizontal, 'estimated_vertical_angle_deg': vertical,
            'true_horizontal_angle_deg': true_horizontal, 'true_vertical_angle_deg': true_vertical,
            'horizontal_angle_error_deg': wrap(horizontal-true_horizontal),
            'vertical_angle_error_deg': wrap(vertical-true_vertical)}


def pose_errors(t, rotation, true_t, true_rotation):
    # Invert camera->tag to obtain camera localization in the surveyed tag frame.
    camera = -rotation.T @ t
    true_camera = -true_rotation.T @ true_t
    delta = t - true_t
    camera_delta = camera - true_camera
    cosine = (np.trace(true_rotation.T @ rotation) - 1) / 2
    axes = orientation_axis_errors(rotation, true_rotation)
    result = {'tag_position_error_m': float(np.linalg.norm(delta)),
              'position_error_m': float(np.linalg.norm(camera_delta)),
              'relative_position_error': float(np.linalg.norm(camera_delta) / np.linalg.norm(true_camera)),
              **axes,
              'estimated_angle_deg': axes['estimated_horizontal_angle_deg'],
              'angle_error_deg': axes['horizontal_angle_error_deg'],
              'rotation_error_deg': float(np.rad2deg(np.arccos(np.clip(cosine, -1, 1)))),
              'depth_error_m': float(t[2] - true_t[2]),
              'range_error_m': float(np.linalg.norm(t) - np.linalg.norm(true_t))}
    for i, axis in enumerate('xyz'):
        result[f'tag_{axis}_m'] = float(t[i])
        result[f'tag_d{axis}_m'] = float(delta[i])
        result[f'camera_{axis}_m'] = float(camera[i])
        result[f'camera_d{axis}_m'] = float(camera_delta[i])
    return result


def summarize(rows, tags, thresholds, detection_only=False):
    report = {'tags': {}, 'passed': True}
    for tag in tags:
        samples = [r for r in rows if r['tag_id'] == tag['id']]
        good = [r for r in samples if r['status'] == 'ok']
        rate = len(good) / len(samples) if samples else 0.0
        stats = {'samples': len(samples), 'poses': len(good),
                 'detection_rate': sum(r['status'] in ('ok', 'missing_tf', 'detected_no_calibration') for r in samples) / len(samples)
                 if samples else 0.0, 'pose_success_rate': None if detection_only else rate,
                 'status_counts': {s: sum(r['status'] == s for r in samples)
                                   for s in sorted({r['status'] for r in samples})}}
        for key in ('position_error_m', 'tag_position_error_m', 'relative_position_error',
                    'rotation_error_deg', 'angle_error_deg', 'horizontal_angle_error_deg',
                    'vertical_angle_error_deg', 'depth_error_m', 'range_error_m',
                    'camera_dx_m', 'camera_dy_m', 'camera_dz_m', 'tag_dx_m', 'tag_dy_m', 'tag_dz_m'):
            values = np.array([r[key] for r in good], dtype=float)
            stats[key] = ({'mean': float(values.mean()), 'std': float(values.std()),
                           'mse': float(np.mean(values**2)), 'rmse': float(np.sqrt(np.mean(values**2))),
                           'p95_abs': float(np.percentile(np.abs(values), 95))}
                          if len(values) else None)
        margins = [r['decision_margin'] for r in samples if 'decision_margin' in r]
        stats['mean_decision_margin'] = float(np.mean(margins)) if margins else None
        failures = []
        if detection_only and (not samples or stats['detection_rate'] < thresholds.get('min_detection_rate', .9)):
            failures.append('detection_rate below minimum or no samples')
        elif not detection_only and (not good or rate < thresholds.get('min_detection_rate', 0.9)):
            failures.append('pose_success_rate below minimum or no valid poses')
        if any(r['status'] in ('timeout', 'missing_tf') for r in samples):
            failures.append('detector response or pose TF missing')
        for limit, metric in [('max_position_p95_m', 'position_error_m'),
                              ('max_rotation_p95_deg', 'rotation_error_deg')]:
            if limit in thresholds and (not good or stats[metric]['p95_abs'] > thresholds[limit]):
                failures.append(f'{metric} exceeds {limit}')
        stats['failures'] = failures
        stats['passed'] = not failures
        report['tags'][str(tag['id'])] = stats
        report['passed'] &= stats['passed']
    return report


def write_reports(directory, rows, report):
    directory = Path(directory)
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with (directory / 'frames.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (directory / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    # Standalone SVG, no plotting dependencies; gaps remain visible for missing detections.
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="500" viewBox="0 0 1000 500">',
             '<rect width="1000" height="500" fill="white"/>']
    detection_only = report.get('analysis_mode') == 'detection_only'
    panels = ([('decision_margin', 'Decision margin (decoder score)'),
               ('detection_success', 'Detection success (0 = missed, 1 = detected)')]
              if detection_only else [('position_error_m', 'Camera position error (m)'),
                                     ('rotation_error_deg', 'Rotation error (degrees)')])
    if detection_only:
        rows = [{**r, 'detection_success': int('decision_margin' in r)} for r in rows]
    for panel, (key, label) in enumerate(panels):
        top = 35 + panel * 245
        good = [r for r in rows if (key in r if detection_only else r['status'] == 'ok')]
        ymax = max([r[key] for r in good] + [0.001]) * 1.1
        xmax = max([r['frame'] for r in rows] + [1])
        parts.append(f'<text x="55" y="{top}" font-size="16">{label}; scale 0–{ymax:.4g}</text>')
        parts.append(f'<path d="M55 {top+10} V{top+180} H970" fill="none" stroke="black"/>')
        for i, tag_id in enumerate(sorted({r['tag_id'] for r in rows})):
            color = ['#1464b4', '#c43825', '#278337', '#8b389b'][i % 4]
            parts.append(f'<text x="{450+i*110}" y="{top}" fill="{color}">tag {tag_id}</text>')
            for row in good:
                if row['tag_id'] == tag_id:
                    x = 55 + row['frame'] / xmax * 915
                    y = top + 180 - row[key] / ymax * 170
                    parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="{color}"/>')
        parts.append(f'<text x="55" y="{top+205}">Video frame: 0 to {xmax}; missing poses omitted (see CSV)</text>')
    parts.append('</svg>')
    (directory / 'errors.svg').write_text('\n'.join(parts))
