"""Experiment-wide frame-weighted statistics with explicit failure denominators."""
import csv
import json
from collections import defaultdict

import numpy as np
from angle_axes import write_angle_axis_reports

METRICS = ('position_error_m', 'tag_position_error_m', 'relative_position_error',
           'rotation_error_deg', 'angle_error_deg', 'depth_error_m', 'range_error_m',
           'camera_dx_m', 'camera_dy_m', 'camera_dz_m', 'tag_dx_m', 'tag_dy_m', 'tag_dz_m')

SHOWCASE_FIELDS = ['Family', 'Distance (m)', 'Angle (deg)', 'Samples', 'Detection (%)',
                   'Mean depth offset (cm)', 'Mean position error (cm)', 'P95 position error (cm)',
                   'Mean absolute angle error (deg)', 'P95 angle error (deg)', 'Calibration']


def write_showcase(output, rows):
    """Compact presentation table, preserving calibration provenance and missing metrics."""
    groups = defaultdict(list)
    for row in rows:
        key = (row['family'], row['distance_m'], row['angle_deg'],
               row.get('calibration_basis', 'unspecified'))
        groups[key].append(row)
    records = []
    for (family, distance, angle, basis), samples in sorted(groups.items()):
        stats = statistics(samples)
        good = [r for r in samples if r['status'] == 'ok']

        def value(metric, scale=1):
            number = stats[metric]
            return round(number * scale, 2) if number is not None else ''

        records.append(dict(zip(SHOWCASE_FIELDS, [
            family, distance, angle, len(samples), round(stats['detection_success_pct'], 2),
            value('depth_error_m_mean', 100), value('position_error_m_mean', 100),
            value('position_error_m_p95_abs', 100),
            round(float(np.mean([abs(r['angle_error_deg']) for r in good])), 2) if good else '',
            value('angle_error_deg_p95_abs'), basis,
        ])))
    with (output / 'showcase.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=SHOWCASE_FIELDS)
        writer.writeheader()
        writer.writerows(records)


def statistics(rows):
    good = [row for row in rows if row['status'] == 'ok']
    detected = [row for row in rows if 'decision_margin' in row]
    localization_available = any(r.get('analysis_mode', 'localization') == 'localization' for r in rows)
    result = {'cases': len({row['case'] for row in rows}), 'samples': len(rows),
              'calibration_bases': ','.join(sorted({r.get('calibration_basis', 'unspecified') for r in rows})),
              'valid_poses': len(good), 'detections': len(detected),
              'detection_success_pct': 100 * len(detected) / len(rows) if rows else 0.,
              'localization_available': localization_available,
              'pose_success_pct': (100 * len(good) / len(rows) if rows else 0.) if localization_available else None,
              'mean_decision_margin': float(np.mean([r['decision_margin'] for r in detected]))
              if detected else None}
    for metric in METRICS:
        values = np.array([r[metric] for r in good], dtype=float)
        for suffix, value in (
            ('mean', float(values.mean()) if len(values) else None),
            ('std', float(values.std()) if len(values) else None),
            ('mse', float(np.mean(values**2)) if len(values) else None),
            ('p95_abs', float(np.percentile(np.abs(values), 95)) if len(values) else None),
        ):
            result[f'{metric}_{suffix}'] = value
    return result


def write_comparison(output, rows, reports):
    groupings = {'family': ('family',), 'distance': ('distance_m',),
                 'family_distance': ('family', 'distance_m'),
                 'family_angle': ('family', 'angle_deg'), 'case': ('case', 'family')}
    comparisons = {'protocol_complete': all('error' not in r and not r.get('protocol_failures') for r in reports),
                   'case_failures': [r for r in reports if not r['passed']], 'groups': {}}
    for name, keys in groupings.items():
        groups = defaultdict(list)
        for row in rows:
            groups[tuple(row[k] for k in keys)].append(row)
        records = [{**dict(zip(keys, key)), **statistics(samples)}
                   for key, samples in sorted(groups.items(), key=lambda item: str(item[0]))]
        comparisons['groups'][name] = records
        with (output / f'{name}_comparison.csv').open('w', newline='') as stream:
            fields = list(records[0]) if records else list(keys)
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
    (output / 'comparison.json').write_text(json.dumps(comparisons, indent=2, allow_nan=False) + '\n')
    write_showcase(output, rows)
    write_angle_axis_reports(output, rows)
