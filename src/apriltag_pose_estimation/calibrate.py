import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from apriltag_pose_estimation.core import CameraParameters


CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def main() -> None:
    parser = argparse.ArgumentParser(prog='calibrate.py',
                                     description='Calibrate a camera in OpenCV from a video file')
    parser.add_argument('video', type=Path,
                        help='the video file from which calibration will be done')
    parser.add_argument('--long', type=int, required=True,
                        help='the number of squares on the long edge')
    parser.add_argument('--short', type=int, required=True,
                        help='the number of squares on the short edge')
    parser.add_argument('--skip', type=int, default=1, required=False,
                        help='the number of frames to skip between each frame (default 1)')
    args = parser.parse_args()

    y_corners = args.long - 1
    x_corners = args.short - 1
    capture = cv2.VideoCapture(str(args.video))
    skip_frequency = args.skip

    base_object_points = np.zeros((x_corners * y_corners, 3), dtype=np.float32)
    base_object_points[:, :2] = np.mgrid[:y_corners, :x_corners].T.reshape(-1, 2)

    object_points_captures = []
    image_points_captures = []

    image = None
    try:
        not_closed = True
        skip_index = 0
        while not_closed:
            not_closed, frame = capture.read()

            if skip_index == 0:
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                success, corners = cv2.findChessboardCorners(image, (y_corners, x_corners), None)

                if success:
                    object_points_captures.append(base_object_points)

                    refined_corners = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), CRITERIA)
                    image_points_captures.append(refined_corners)

                    cv2.drawChessboardCorners(frame, (y_corners, x_corners), corners, success)
                    cv2.imshow('image', frame)
                    key = cv2.waitKey(50)

                    print(f'Found {len(image_points_captures)} images')

                    if key == ord('q'):
                        break

            skip_index = (skip_index + 1) % skip_frequency
    finally:
        cv2.destroyAllWindows()

    if image is None:
        print('No images found, exiting.', file=sys.stderr)
        sys.exit(1)

    print(f'Calibrating with {len(image_points_captures)} images...')

    success, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(object_points_captures,
                                                                                        image_points_captures,
                                                                                        image.shape[::-1],
                                                                                        None,
                                                                                        None)
    if success:
        print('Calibration success!')
    else:
        print('Calibration failed, exiting.', file=sys.stderr)
        sys.exit(1)

    camera_params = CameraParameters.from_matrices(camera_matrix=camera_matrix.reshape(3, 3),
                                                   distortion_vector=distortion_coefficients.reshape(-1))

    print(f'\n\n{camera_params}')

    mean_error = 0
    for object_points, image_points, rvec, tvec in zip(object_points_captures, image_points_captures, rvecs, tvecs):
        reprojected_image_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coefficients)
        error = cv2.norm(image_points, reprojected_image_points, cv2.NORM_L2) / len(reprojected_image_points)
        mean_error += error
    mean_error /= len(object_points_captures)

    print(f'\n\nreprojection error: {mean_error}')


if __name__ == '__main__':
    main()
