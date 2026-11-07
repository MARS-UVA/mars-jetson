"""Generate reviewable per-video cases from the dorm experiment plan."""
import csv
import os
from pathlib import Path

import numpy as np
import yaml

FAMILIES = {'36h11': '36h11', '41h12': 'Standard41h12', '25h9': '25h9'}


def generate(path, inside):
    path = inside(path)
    plan = yaml.safe_load(path.read_text())
    target = inside(path.parent / plan['output_directory'])
    camera_path = inside(path.parent / plan['camera_file'])
    if not camera_path.is_file():
        raise ValueError(f'Missing camera file: {camera_path}')
    angles = plan['angles_deg']
    nominal = plan['nominal_distances_m']
    actual = plan.get('measured_distances_m', nominal)
    if len(nominal) != len(actual) or not nominal or not angles:
        raise ValueError('Need angles and matching nominal/measured distance lists')
    if not np.isfinite(angles).all() or not np.isfinite(nominal).all() or not np.isfinite(actual).all():
        raise ValueError('Angles and distances must be finite')
    if min(nominal) <= 0 or min(actual) <= 0:
        raise ValueError('Distances must be positive')
    offset_y = 0.
    if 'mount' in plan:
        mount = plan['mount']
        heights = [float(mount[key]) for key in ('camera_height_m', 'tag_bottom_height_m', 'tag_height_m')]
        if not np.isfinite(heights).all() or min(heights[:2]) < 0 or heights[2] <= 0:
            raise ValueError('Mount heights must be finite, with nonnegative floor heights and positive tag height')
        # Camera optical +Y points down; assumes a level camera and upright tag.
        offset_y = heights[0] - (heights[1] + heights[2]/2)
    fps, duration = float(plan.get('fps', 60)), float(plan.get('duration_s', 5))
    frames = round(fps * duration)
    if fps <= 0 or duration <= 0 or frames <= 0:
        raise ValueError('FPS and duration must be positive')
    cases = []
    for family in plan['families']:
        canonical = FAMILIES.get(str(family), str(family))
        for angle in angles:
            for nominal_distance, distance in zip(nominal, actual):
                name = f'{family}_angle_{angle:+g}_distance_{nominal_distance:g}m'
                folder = inside(target / name)
                case = {'video': 'video.mp4',
                        'camera_file': os.path.relpath(camera_path, folder),
                        'detector': {'family': canonical, 'max_hamming': 0, 'decimate': 1.0},
                        'tags': [{'id': int(plan.get('tag_ids', {}).get(str(family), 1)),
                                  'size_m': float(plan.get('tag_sizes_m', {}).get(str(family), plan.get('tag_size_m', .2))),
                                  'ground_truth': {'offset_x_m': 0., 'offset_y_m': float(offset_y),
                                                   'distance_m': float(distance), 'angle_deg': float(angle)}}],
                        'experiment': {'nominal_distance_m': float(nominal_distance)},
                        'sampling': {'start_frame': 0, 'stride': 1, 'max_frames': frames,
                                     'required_frames': frames, 'expected_fps': fps, 'timeout_s': 5.},
                        'annotation': {'enabled': True},
                        'thresholds': plan.get('thresholds', {'min_detection_rate': .9})}
                cases.append((folder, case))
    if len({folder for folder, _ in cases}) != len(cases):
        raise ValueError('Duplicate family/angle/distance combinations')
    # Preflight all outputs before writing; rerunning cannot erase edited measurements.
    manifest = target / 'manifest.csv'
    if manifest.exists() or any((folder / 'setup.yaml').exists() for folder, _ in cases):
        raise ValueError('Matrix already exists; edit existing setups or choose a new output_directory')
    target.mkdir(parents=True, exist_ok=True)
    with manifest.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['setup', 'video', 'family', 'angle_deg', 'nominal_distance_m',
                         'measured_distance_m', 'expected_frames'])
        for folder, case in cases:
            folder.mkdir(parents=True, exist_ok=True)
            (folder / 'setup.yaml').write_text(yaml.safe_dump(case, sort_keys=False))
            truth = case['tags'][0]['ground_truth']
            writer.writerow([str((folder / 'setup.yaml').relative_to(target)),
                             str((folder / 'video.mp4').relative_to(target)),
                             case['detector']['family'], truth['angle_deg'],
                             case['experiment']['nominal_distance_m'], truth['distance_m'], frames])
    print(f'Generated {len(cases)} cases ({frames} frames each) in {target}\nChecklist: {manifest}')
