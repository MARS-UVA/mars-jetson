"""Generate synthetic videos, then exercise the REAL installed apriltag_ros node."""
import json
import ctypes
import ctypes.util
from pathlib import Path
import subprocess
import tempfile

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def native_marker(family):
    class Image(ctypes.Structure):
        _fields_ = [('width', ctypes.c_int32), ('height', ctypes.c_int32),
                    ('stride', ctypes.c_int32), ('buf', ctypes.POINTER(ctypes.c_uint8))]

    class Family(ctypes.Structure):
        _fields_ = [('ncodes', ctypes.c_uint32), ('codes', ctypes.POINTER(ctypes.c_uint64)),
                    ('width_at_border', ctypes.c_int), ('total_width', ctypes.c_int)]

    library = ctypes.util.find_library('apriltag')
    if not library:
        library = str(next(Path('/opt/ros').glob('*/lib/**/libapriltag.so')))
    lib = ctypes.CDLL(library)
    create = getattr(lib, f'tag{family}_create')
    create.restype = ctypes.POINTER(Family)
    native = create()
    lib.apriltag_to_image.argtypes = [ctypes.POINTER(Family), ctypes.c_uint32]
    lib.apriltag_to_image.restype = ctypes.POINTER(Image)
    image_ptr = lib.apriltag_to_image(native, 1)
    image = image_ptr.contents
    marker = np.ctypeslib.as_array(image.buf, shape=(image.height, image.stride))[:, :image.width].copy()
    extent_ratio = image.width / native.contents.width_at_border
    lib.image_u8_destroy.argtypes = [ctypes.POINTER(Image)]
    lib.image_u8_destroy(image_ptr)
    destroy = getattr(lib, f'tag{family}_destroy')
    destroy.argtypes = [ctypes.POINTER(Family)]
    destroy(native)
    return cv2.resize(marker, (300, 300), interpolation=cv2.INTER_NEAREST), extent_ratio


def main():
    (ROOT / 'results').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='smoke_', dir=ROOT / 'results') as temporary:
        directory = Path(temporary)
        setups = []
        definitions = [('front', 0, False, '36h11'), ('oblique', 25, False, '36h11'),
                       ('standard41', -45, False, 'Standard41h12'), ('family25', 45, False, '25h9'),
                       ('blank', 0, True, '36h11')]
        for name, angle, blank, family in definitions:
            marker, extent_ratio = native_marker(family)
            folder = directory / name
            folder.mkdir()
            video = folder / 'video.avi'
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 60, (1280, 960))
            if not writer.isOpened():
                raise RuntimeError('MJPG encoder unavailable')
            k = np.array([[1200., 0, 640], [0, 1200., 480], [0, 0, 1]])
            offset_x, offset_y = (.12, -.06) if angle else (0., 0.)
            corners = np.array([[-.1, -.1, 0], [.1, -.1, 0], [.1, .1, 0], [-.1, .1, 0]]) * extent_ratio
            projected, _ = cv2.projectPoints(corners, np.array([0., np.deg2rad(angle), 0.]),
                                            np.array([offset_x, offset_y, 1.]), k, np.zeros(5))
            homography = cv2.getPerspectiveTransform(np.float32([[0, 0], [299, 0], [299, 299], [0, 299]]),
                                                     projected.reshape(4, 2).astype(np.float32))
            frame = cv2.warpPerspective(marker, homography, (1280, 960), borderValue=255)
            if blank:
                frame[:] = 255
            for _ in range(4):
                writer.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
            writer.release()
            spec = {'video': 'video.avi', 'camera': {'width': 1280, 'height': 960, 'fx': 1200.,
                    'fy': 1200., 'cx': 640., 'cy': 480., 'distortion': [0.] * 5,
                    'output_width': 640, 'output_height': 480},
                    'detector': {'family': family},
                    'tags': [{'id': 1, 'size_m': .2, 'ground_truth': {
                        'offset_x_m': offset_x, 'offset_y_m': offset_y,
                        'distance_m': 1., 'angle_deg': angle}}],
                    'sampling': {'max_frames': 4, 'required_frames': 4, 'expected_fps': 60, 'timeout_s': 2},
                    'thresholds': {'min_detection_rate': 1., 'max_position_p95_m': .08,
                                   'max_rotation_p95_deg': 4.}}
            path = folder / 'setup.yaml'
            path.write_text(yaml.safe_dump(spec))
            setups.append(path)
        validation = subprocess.run([str(ROOT / 'run.sh'), '--validate', *map(str, setups)],
                                    text=True, capture_output=True)
        assert validation.returncode == 0, validation.stderr
        result = subprocess.run([str(ROOT / 'run.sh'), *map(str, setups)], text=True, capture_output=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        assert result.returncode == 1, 'Blank case should fail thresholds'
        results = Path(result.stdout.split('Results: ', 1)[1].splitlines()[0])
        reports = json.loads((results / 'summary.json').read_text())
        for report in reports[:4]:
            assert report['passed'], report
        assert reports[4]['tags']['1']['status_counts'] == {'not_detected': 4}, reports[4]
        for index, (name, _, blank, _) in enumerate(definitions, 1):
            for filename in ('frames.csv', 'summary.json', 'errors.svg', 'annotated.avi'):
                assert (results / f'{index:02d}_{name}' / filename).is_file()
            video = cv2.VideoCapture(str(results / f'{index:02d}_{name}' / 'annotated.avi'))
            assert video.get(cv2.CAP_PROP_FPS) == 60
            assert video.get(cv2.CAP_PROP_FRAME_COUNT) == 4
            ok, annotated = video.read()
            video.release()
            assert ok and annotated.shape == (480, 640, 3)
            # Verify actual green polygons survived video encoding; blank clip has none.
            green = ((annotated[:, :, 1].astype(float) - annotated[:, :, 0] > 50)
                     & (annotated[:, :, 1].astype(float) - annotated[:, :, 2] > 50))
            assert bool(green.any()) != blank
        comparison = json.loads((results / 'comparison.json').read_text())
        assert len(comparison['groups']['family']) == 3
        assert sum(g['samples'] for g in comparison['groups']['family']) == 20
        print('Smoke checks passed: all three families, offsets/angles, downscaling, annotations and blank failure.')


if __name__ == '__main__':
    main()
