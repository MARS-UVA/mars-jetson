"""Calibrated video resizing, distortion correction and detection annotation."""
import cv2
import numpy as np


def processed_camera(camera, detection_only=False):
    width = int(camera.get('output_width', camera['width']))
    height = int(camera.get('output_height', camera['height']))
    if width <= 0 or height <= 0 or width * camera['height'] != height * camera['width']:
        raise ValueError('Output dimensions must be positive and preserve source aspect ratio')
    if detection_only and not all(camera.get(key) is not None for key in ('fx', 'fy', 'cx', 'cy')):
        return width, height, None
    k = np.array([[camera['fx'], 0, camera['cx']],
                  [0, camera['fy'], camera['cy']], [0, 0, 1]], dtype=float)
    k[0] *= width / camera['width']
    k[1] *= height / camera['height']
    return width, height, k


class Preprocessor:
    def __init__(self, camera, detection_only=False):
        self.source_shape = (int(camera['height']), int(camera['width']))
        self.width, self.height, self.k = processed_camera(camera, detection_only)
        self.rotation = camera.get('rotation_degrees', 0)
        if self.rotation not in (0, 180):
            raise ValueError('rotation_degrees must be 0 or 180')
        self.maps = None
        if self.k is not None and not camera.get('already_rectified', False):
            self.maps = cv2.initUndistortRectifyMap(
                self.k, np.array(camera.get('distortion', [0.] * 5)), None, self.k,
                (self.width, self.height), cv2.CV_32FC1)
        if self.rotation == 180 and self.k is not None:
            self.k = self.k.copy()
            self.k[0, 2] = self.width - 1 - self.k[0, 2]
            self.k[1, 2] = self.height - 1 - self.k[1, 2]

    def process(self, frame):
        if frame.shape[:2] != self.source_shape:
            raise ValueError(f'Video resolution {frame.shape[1]}x{frame.shape[0]} differs from calibration')
        if frame.shape[:2] != (self.height, self.width):
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if self.maps is not None:
            frame = cv2.remap(frame, *self.maps, interpolation=cv2.INTER_LINEAR)
        if self.rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame


def annotate(frame, detections, rows, index, video_time):
    """Draw actual detector corners; never invent or carry forward a missing box."""
    annotated = frame.copy()
    by_id = {row['tag_id']: row for row in rows}
    for detection in detections:
        if detection.id not in by_id:
            continue
        corners = np.array([[p.x, p.y] for p in detection.corners], dtype=float)
        if not np.isfinite(corners).all():
            continue
        points = np.rint(corners).astype(np.int32)
        row = by_id[detection.id]
        color = (30, 220, 30) if row['status'] in ('ok', 'detected_no_calibration') else (0, 180, 255)
        cv2.polylines(annotated, [points], True, color, 2, cv2.LINE_AA)
        for point in points:
            cv2.circle(annotated, tuple(point), 3, color, -1)
        x, y = points.min(axis=0)
        label = f'{detection.family}:{detection.id} margin={detection.decision_margin:.1f}'
        cv2.putText(annotated, label, (max(0, int(x)), max(18, int(y)-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    text = f'frame {index}  t={video_time:.3f}s  ' + ' '.join(
        f'id {r["tag_id"]}: {r["status"]}' for r in rows)
    cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(annotated, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, .42,
                (255, 255, 255), 1, cv2.LINE_AA)
    return annotated
