#!/usr/bin/env python3
"""Run the installed apriltag_ros against isolated, stationary video cases."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

from analysis import ground_truth, pose_errors, quaternion_rotation, summarize, write_reports
from comparison import write_comparison
from video import Preprocessor, annotate, processed_camera

ROOT = Path(__file__).resolve().parent


def inside(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f'All inputs and outputs must be inside {ROOT}: {path}')
    return path


def load_case(path):
    path = inside(path)
    spec = yaml.safe_load(path.read_text())
    mode = spec.setdefault('analysis_mode', 'localization')
    if mode not in ('localization', 'detection_only'):
        raise ValueError('analysis_mode must be localization or detection_only')
    detection_only = mode == 'detection_only'
    spec['video_path'] = str(inside(path.parent / spec['video']))
    if not Path(spec['video_path']).is_file():
        raise ValueError(f'Video missing: {spec["video_path"]}')
    if 'camera_file' in spec:
        if 'camera' in spec:
            raise ValueError('Use either camera or camera_file')
        camera_path = inside(path.parent / spec['camera_file'])
        spec['camera'] = yaml.safe_load(camera_path.read_text())
        spec['camera_source_path'] = str(camera_path)
        del spec['camera_file']
    camera = spec['camera']
    if camera.get('width') == 'auto' or camera.get('height') == 'auto':
        capture = cv2.VideoCapture(spec['video_path'])
        try:
            ok, first_frame = capture.read()
            if not ok:
                raise ValueError('Cannot decode video to determine source resolution')
            source_height, source_width = first_frame.shape[:2]
        finally:
            capture.release()
    else:
        source_width, source_height = camera.get('width'), camera.get('height')
    camera['width'], camera['height'] = source_width, source_height
    allowed = {'width', 'height', 'fx', 'fy', 'cx', 'cy', 'distortion',
               'output_width', 'output_height', 'already_rectified', 'rotation_degrees', 'metadata'}
    unknown = set(camera) - allowed
    if unknown:
        raise ValueError(f'Unknown camera fields: {sorted(unknown)}; use the current camera.example.yaml')
    for key in (('width', 'height') if detection_only else ('width', 'height', 'fx', 'fy', 'cx', 'cy')):
        if camera.get(key) is None:
            raise ValueError(f'camera.{key} needs a measured calibration value')
        value = float(camera[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'camera.{key} must be finite and positive')
    if any(int(camera[k]) != camera[k] for k in ('width', 'height')):
        raise ValueError('Camera width/height must be integers')
    distortion = camera.get('distortion', [0.0] * 5)
    if len(distortion) not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError('Use finite OpenCV plumb_bob distortion coefficients')
    camera['distortion'] = distortion
    for key in ('output_width', 'output_height'):
        if key in camera and (type(camera[key]) is not int or camera[key] <= 0):
            raise ValueError(f'camera.{key} must be a positive integer')
    processed_camera(camera, detection_only)
    if camera.get('rotation_degrees', 0) not in (0, 180):
        raise ValueError('rotation_degrees must be 0 or 180')
    tags = spec['tags']
    ids = [t['id'] for t in tags]
    if not ids or len(set(ids)) != len(ids) or any(type(i) is not int or i < 0 for i in ids):
        raise ValueError('tags must have unique nonnegative integer IDs')
    for tag in tags:
        if not np.isfinite(tag['size_m']) or tag['size_m'] <= 0:
            raise ValueError('Tag size_m must be positive and finite')
        ground_truth(tag['ground_truth'])
    sampling = spec.setdefault('sampling', {})
    for key, default, minimum in [('start_frame', 0, 0), ('stride', 1, 1), ('max_frames', 300, 1)]:
        value = sampling.setdefault(key, default)
        if type(value) is not int or value < minimum:
            raise ValueError(f'sampling.{key} must be an integer >= {minimum}')
    timeout = sampling.setdefault('timeout_s', 5.0)
    if not np.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout_s must be positive')
    required = sampling.get('required_frames', 0)
    if type(required) is not int or required < 0 or required > sampling['max_frames']:
        raise ValueError('required_frames must be an integer between 0 and max_frames')
    if 'expected_fps' in sampling and (not np.isfinite(sampling['expected_fps']) or sampling['expected_fps'] <= 0):
        raise ValueError('expected_fps must be positive and finite')
    thresholds = spec.setdefault('thresholds', {})
    for key, value in thresholds.items():
        if key not in ('min_detection_rate', 'max_position_p95_m', 'max_rotation_p95_deg'):
            raise ValueError(f'Unknown threshold: {key}')
        if not np.isfinite(value) or value < 0 or (key == 'min_detection_rate' and value > 1):
            raise ValueError(f'Invalid threshold: {key}')
    detector = spec.setdefault('detector', {})
    if detector.get('family') == '41h12':
        detector['family'] = 'Standard41h12'
    if detector.get('family', '36h11') not in (
        '16h5', '25h9', '36h10', '36h11', 'Circle21h7', 'Circle49h12',
        'Custom48h12', 'Standard41h12', 'Standard52h13',
    ):
        raise ValueError('Unsupported AprilTag family')
    return spec


def run_case(spec, output, namespace):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from apriltag_msgs.msg import AprilTagDetectionArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    camera, tags, sampling = spec['camera'], spec['tags'], spec['sampling']
    detector = spec['detector']
    detection_only = spec['analysis_mode'] == 'detection_only'
    expected = {t['id']: ground_truth(t['ground_truth']) for t in tags}
    params = {'/**': {'ros__parameters': {
        'use_sim_time': False, 'family': detector.get('family', '36h11'),
        'size': float(tags[0]['size_m']), 'max_hamming': int(detector.get('max_hamming', 0)),
        'image_transport': 'raw', 'qos_profile': 'default', 'pose_estimation_method': '' if detection_only else 'pnp',
        'detector.decimate': float(detector.get('decimate', 1.0)),
        'detector.threads': 1, 'detector.debug': False,
        'tag.ids': [t['id'] for t in tags],
        'tag.frames': [f'measured_tag_{t["id"]}' for t in tags],
        'tag.sizes': [float(t['size_m']) for t in tags],
    }}}
    param_path = output / 'detector.yaml'
    param_path.write_text(yaml.safe_dump(params))
    executable = Path(get_package_prefix('apriltag_ros')) / 'lib/apriltag_ros/apriltag_node'
    if not executable.is_file():
        raise RuntimeError(f'apriltag_ros executable missing: {executable}')
    command = [str(executable), '--ros-args', '--params-file', str(param_path),
               '-r', f'__ns:={namespace}', '-r', '__node:=detector',
               '-r', f'/tf:={namespace}/tf', '-r', f'/tf_static:={namespace}/tf_static',
               '-r', f'/rosout:={namespace}/rosout',
               '-r', f'/parameter_events:={namespace}/parameter_events']
    node = Node('video_analysis', namespace=namespace, enable_rosout=False,
                start_parameter_services=False, use_global_arguments=False,
                cli_args=['--ros-args', '-r', f'/parameter_events:={namespace}/parameter_events'])
    pub = node.create_publisher(Image, 'image_rect', 10)
    info_pub = node.create_publisher(CameraInfo, 'camera_info', 10)
    active = {'stamp': None, 'detection': None, 'transforms': {}}

    def stamp_key(header):
        return header.stamp.sec, header.stamp.nanosec

    def detections(msg):
        if stamp_key(msg.header) == active['stamp']:
            active['detection'] = msg

    def transforms(msg):
        for transform in msg.transforms:
            if stamp_key(transform.header) == active['stamp'] and transform.header.frame_id == 'test_camera_optical':
                active['transforms'][transform.child_frame_id] = transform.transform

    node.create_subscription(AprilTagDetectionArray, 'detections', detections, 10)
    node.create_subscription(TFMessage, 'tf', transforms, qos_profile_sensor_data)
    cap = cv2.VideoCapture(spec['video_path'])
    rows = []
    writer = None
    process = None
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot decode video')
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0:
            raise RuntimeError('Video has no valid FPS')
        preprocessor = Preprocessor(camera, detection_only)
        width, height, intrinsics = preprocessor.width, preprocessor.height, preprocessor.k
        if spec.get('annotation', {}).get('enabled', True):
            writer = cv2.VideoWriter(str(output / 'annotated.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                     fps / sampling['stride'], (width, height))
            if not writer.isOpened():
                raise RuntimeError('Cannot create annotated.avi (MJPG encoder unavailable)')
        info = CameraInfo(width=width, height=height, distortion_model='plumb_bob', d=[0.0]*5)
        if intrinsics is not None:
            info.k = intrinsics.ravel().tolist()
            info.r = np.eye(3).ravel().tolist()
            info.p = np.column_stack((intrinsics, np.zeros(3))).ravel().tolist()
        with (output / 'detector.log').open('w') as log:
            process = subprocess.Popen(command, cwd=output, stdout=log, stderr=subprocess.STDOUT)

            def spin_until(predicate, timeout):
                deadline = time.monotonic() + timeout
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError('apriltag_ros exited; see detector.log')
                    if time.monotonic() >= deadline:
                        return False
                    rclpy.spin_once(node, timeout_sec=0.02)
                return True

            ready = lambda: (pub.get_subscription_count() > 0 and info_pub.get_subscription_count() > 0
                             and node.count_publishers(f'{namespace}/detections') > 0
                             and (detection_only or node.count_publishers(f'{namespace}/tf') > 0))
            if not spin_until(ready, 20):
                raise RuntimeError('Detector discovery timed out; see detector.log')
            # Allow endpoints to finish matching before the first image.
            until = time.monotonic() + 0.3
            spin_until(lambda: time.monotonic() >= until, 1)
            frame_index, selected = 0, 0
            while selected < sampling['max_frames']:
                ok, frame = cap.read()
                if not ok:
                    break
                index = frame_index
                frame_index += 1
                if index < sampling['start_frame'] or (index - sampling['start_frame']) % sampling['stride']:
                    continue
                frame = preprocessor.process(frame)
                mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                image = Image(height=height, width=width, encoding='mono8',
                              is_bigendian=0, step=width, data=mono.tobytes())
                image.header.frame_id = 'test_camera_optical'
                image.header.stamp = node.get_clock().now().to_msg()
                info.header = image.header
                active.update(stamp=stamp_key(image.header), detection=None, transforms={})
                info_pub.publish(info)
                pub.publish(image)

                def complete():
                    message = active['detection']
                    return message is not None and (detection_only or all(
                        f'measured_tag_{d.id}' in active['transforms'] for d in message.detections))

                spin_until(complete, sampling['timeout_s'])
                message = active['detection']
                detected = {d.id: d for d in message.detections} if message is not None else {}
                frame_rows = []
                for tag in tags:
                    tag_id = tag['id']
                    detection = detected.get(tag_id)
                    transform = active['transforms'].get(f'measured_tag_{tag_id}')
                    status = ('timeout' if message is None else 'not_detected' if detection is None
                              else 'detected_no_calibration' if detection_only
                              else 'missing_tf' if transform is None else 'ok')
                    true_t, _ = expected[tag_id]
                    truth = tag['ground_truth']
                    row = {'frame': index, 'video_time_s': index / fps, 'tag_id': tag_id, 'status': status,
                           'analysis_mode': spec['analysis_mode'],
                           'calibration_basis': camera.get('metadata', {}).get('calibration_basis', 'user_supplied_intrinsics'),
                           'family': detector.get('family', '36h11'),
                           'distance_m': float(true_t[2]),
                           'angle_deg': truth.get('angle_deg', truth.get('rpy_deg', [0, 0, 0])[1]),
                           'nominal_distance_m': spec.get('experiment', {}).get('nominal_distance_m', float(true_t[2])),
                           'case': spec['case_name']}
                    if detection is not None:
                        row.update(hamming=detection.hamming, decision_margin=detection.decision_margin)
                        for corner_index, corner in enumerate(detection.corners):
                            row[f'corner_{corner_index}_x_px'] = corner.x
                            row[f'corner_{corner_index}_y_px'] = corner.y
                        for matrix_index, value in enumerate(detection.homography):
                            row[f'homography_{matrix_index}'] = value
                    if status == 'ok':
                        t = transform.translation
                        row.update(pose_errors(np.array([t.x, t.y, t.z]), quaternion_rotation(transform.rotation),
                                               *expected[tag_id]))
                        q = transform.rotation
                        row.update(qx=q.x, qy=q.y, qz=q.z, qw=q.w)
                    rows.append(row)
                    frame_rows.append(row)
                if writer is not None:
                    writer.write(annotate(frame, list(detected.values()), frame_rows, index, index / fps))
                selected += 1
                if selected % 50 == 0:
                    print(f'  processed {selected} frames', flush=True)
            if not selected:
                raise RuntimeError('No frames selected; check video and start_frame')
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
    report = summarize(rows, tags, spec['thresholds'], detection_only=detection_only)
    protocol_failures = []
    if selected < sampling.get('required_frames', 0):
        protocol_failures.append(f'Expected {sampling["required_frames"]} frames; decoded/selected {selected}')
    if 'expected_fps' in sampling and abs(fps - sampling['expected_fps']) > .1:
        protocol_failures.append(f'Expected {sampling["expected_fps"]} FPS; decoded {fps:g}')
    if protocol_failures:
        report['passed'] = False
    report['protocol_failures'] = protocol_failures
    report.update(video=spec['video_path'], frames=selected, fps=fps,
                  camera_metadata=camera.get('metadata', {}),
                  analysis_mode=spec['analysis_mode'], localization_available=not detection_only,
                  family=detector.get('family', '36h11'), processed_width=width, processed_height=height,
                  annotated_video='annotated.avi' if writer is not None else None,
                  coordinate_frame=('pixel coordinates; metric localization unavailable' if detection_only
                                    else 'camera position in tag frame; meters, degrees'))
    write_reports(output, rows, report)
    return report, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('setups', nargs='*', type=Path, help='setup.yaml paths (default: recursively under setups/)')
    parser.add_argument('--validate', action='store_true', help='Validate files and calibration without starting ROS')
    parser.add_argument('--generate-matrix', type=Path, help='Generate setup.yaml files from an experiment plan, then exit')
    args = parser.parse_args()
    if args.generate_matrix:
        from matrix import generate
        generate(args.generate_matrix, inside)
        return 0
    paths = args.setups or sorted((ROOT / 'setups').rglob('setup.yaml'))
    if not paths:
        parser.error('No setup.yaml files found')
    cases = [(inside(p), load_case(p)) for p in paths]
    for path, spec in cases:
        spec['case_name'] = str(path.parent.relative_to(ROOT))
    if args.validate:
        for path, spec in cases:
            cap = cv2.VideoCapture(spec['video_path'])
            try:
                ok, frame = cap.read()
                if not ok or frame.shape[:2] != (spec['camera']['height'], spec['camera']['width']):
                    raise ValueError(f'{path}: cannot decode video or calibration resolution mismatch')
                fps = cap.get(cv2.CAP_PROP_FPS)
                if not np.isfinite(fps) or fps <= 0:
                    raise ValueError(f'{path}: invalid video FPS')
                if 'expected_fps' in spec['sampling'] and abs(fps - spec['sampling']['expected_fps']) > .1:
                    raise ValueError(f'{path}: expected {spec["sampling"]["expected_fps"]} FPS, got {fps:g}')
                count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                available = max(0, (count - spec['sampling']['start_frame'] + spec['sampling']['stride'] - 1)
                                // spec['sampling']['stride'])
                if count > 0 and available < spec['sampling'].get('required_frames', 0):
                    raise ValueError(f'{path}: only {available} samples available; need {spec["sampling"]["required_frames"]}')
            finally:
                cap.release()
            print(f'Valid: {path}')
        return 0
    # Ignore inherited ROS domain and global topic names. Never use the ROS CLI daemon.
    domain = 100 + secrets.randbelow(100)
    namespace = '/apriltag_test_' + secrets.token_hex(6)
    output = inside(ROOT / 'results' / (time.strftime('%Y%m%dT%H%M%S') + '_' + secrets.token_hex(3)))
    output.mkdir(parents=True)
    profile = output / 'dds.xml'
    profile.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>test_loopback</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="test_only" is_default_profile="true"><rtps>
    <userTransports><transport_id>test_loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>
''')
    for variable in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_DEFAULT_PROFILES_FILE',
                     'FASTRTPS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(variable, None)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      ROS_LOG_DIR=str(output / 'ros_logs'), ROS_HOME=str(output / 'ros_home'),
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS2CLI_NO_DAEMON='1',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
    print(f'Results: {output}\nLocal ROS domain: {domain}; namespace: {namespace}', flush=True)
    import rclpy
    rclpy.init(args=[])
    reports = []
    all_rows = []
    try:
        for i, (path, spec) in enumerate(cases):
            case_output = output / f'{i+1:02d}_{path.parent.name}'
            case_output.mkdir()
            (case_output / 'setup.yaml').write_text(yaml.safe_dump({**spec, 'video': spec['video_path']}))
            print(f'Running {path.parent.name}', flush=True)
            try:
                report, rows = run_case(spec, case_output, namespace)
                all_rows.extend(rows)
            except Exception as exc:
                report = {'passed': False, 'error': str(exc), 'family': spec['detector'].get('family', '36h11')}
                (case_output / 'error.json').write_text(json.dumps(report, indent=2))
            reports.append({'setup': str(path), **report})
            print(f'  {"PASS" if report["passed"] else "FAIL"}' +
                  (f': {report["error"]}' if 'error' in report else ''), flush=True)
    finally:
        rclpy.shutdown()
    (output / 'summary.json').write_text(json.dumps(reports, indent=2, allow_nan=False) + '\n')
    write_comparison(output, all_rows, reports)
    return 0 if all(r['passed'] for r in reports) else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, TypeError, KeyError, OSError, ImportError, RuntimeError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        sys.exit(2)
