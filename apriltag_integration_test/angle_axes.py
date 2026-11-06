"""Export horizontal/vertical tag-normal errors from actual pose quaternions.

Usage: python3 angle_axes.py results/<run-directory>
No detector rerun or camera refit is needed to analyze saved poses.
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from analysis import ground_truth, orientation_axis_errors, quaternion_rotation

ROOT = Path(__file__).resolve().parent
AXIS_FIELDS = ['estimated_horizontal_angle_deg', 'true_horizontal_angle_deg', 'horizontal_angle_error_deg',
               'estimated_vertical_angle_deg', 'true_vertical_angle_deg', 'vertical_angle_error_deg']


def load_axis_rows(directory):
    rows = []
    for path in sorted(Path(directory).glob('*/frames.csv')):
        config = yaml.safe_load((path.parent / 'setup.yaml').read_text())
        rotations = {t['id']: ground_truth(t['ground_truth'])[1] for t in config['tags']}
        with path.open() as stream:
            for raw in csv.DictReader(stream):
                row = {}
                for key, value in raw.items():
                    if value == '':
                        continue
                    try:
                        row[key] = float(value)
                    except ValueError:
                        row[key] = value
                if row['status'] == 'ok':
                    q = SimpleNamespace(**{axis: row['q'+axis] for axis in 'xyzw'})
                    row.update(orientation_axis_errors(quaternion_rotation(q), rotations[int(row['tag_id'])]))
                rows.append(row)
    if not rows:
        raise ValueError('No per-frame CSV data found')
    return rows


def write_angle_axis_reports(output, rows, ddof=0):
    output = Path(output)
    fields = ['Family', 'Distance (m)', 'Samples', 'Valid poses']
    for axis in ('Horizontal', 'Vertical'):
        fields.extend([f'{axis} mean offset (deg)', f'{axis} stdev (deg)',
                       f'{axis} mean absolute error (deg)', f'{axis} P95 absolute error (deg)'])
    fields.append('Calibration')
    groups = defaultdict(list)
    for row in rows:
        basis = row.get('calibration_basis', 'unspecified')
        for distance in (row['distance_m'], 'Total average'):
            groups[(row['family'], distance, basis)].append(row)
    records = []
    for (family, distance, basis), samples in sorted(groups.items(), key=lambda item: (
            item[0][0], item[0][1] == 'Total average',
            float(item[0][1]) if item[0][1] != 'Total average' else 0, item[0][2])):
        valid = [r for r in samples if r['status'] == 'ok']
        record = {'Family': family, 'Distance (m)': distance, 'Samples': len(samples),
                  'Valid poses': len(valid), 'Calibration': basis}
        for axis in ('Horizontal', 'Vertical'):
            values = np.array([r[axis.lower()+'_angle_error_deg'] for r in valid])
            record.update({f'{axis} mean offset (deg)': round(float(values.mean()), 4) if len(values) else '',
                           f'{axis} stdev (deg)': round(float(values.std(ddof=ddof)), 4) if len(values) > ddof else '',
                           f'{axis} mean absolute error (deg)': round(float(np.abs(values).mean()), 4) if len(values) else '',
                           f'{axis} P95 absolute error (deg)': round(float(np.percentile(np.abs(values),95)), 4) if len(values) else ''})
        records.append(record)
    summary_path = output / 'horizontal_vertical_angle_errors.csv'
    with summary_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    frame_fields = ['case', 'family', 'distance_m', 'tag_id', 'frame', 'video_time_s', 'status',
                    *AXIS_FIELDS, 'calibration_basis']
    with (output / 'horizontal_vertical_angle_errors_frames.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=frame_fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    directory = args.results.resolve()
    if not directory.is_relative_to(ROOT / 'results'):
        parser.error('Results must be inside apriltag_integration_test/results')
    records = write_angle_axis_reports(directory, load_axis_rows(directory))
    print(directory / 'horizontal_vertical_angle_errors.csv')
    for record in records:
        print(record)


if __name__ == '__main__':
    main()
