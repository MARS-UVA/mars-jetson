from dataclasses import replace
from typing import List

import numpy as np
import numpy.typing as npt

from ..estimation import AprilTagPoseEstimationStrategy
from ...core.camera import CameraParameters
from ...core.detection import AprilTagDetection, AprilTagDetector
from ...core.euclidean import Transform
from ...core.pnp import PnPMethod, solve_pnp


__all__ = ['PerspectiveNPointStrategy']


class PerspectiveNPointStrategy(AprilTagPoseEstimationStrategy):
    """
    A pose estimation strategy which solves the Perspective-N-Point problem.

    This strategy is implemented with OpenCV's solvePnP function. See
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html for more information.
    """
    def __init__(self, method: PnPMethod = PnPMethod.ITERATIVE):
        self.__method = method

    @property
    def method(self):
        """The method which this strategy is using for solving the Perspective-N-Point problem."""
        return self.__method

    def estimate_tag_pose(self,
                          image: npt.NDArray[np.uint8],
                          detector: AprilTagDetector,
                          camera_params: CameraParameters,
                          tag_size: float) -> List[AprilTagDetection]:
        # noinspection PyTypeChecker
        return [replace(detection, tag_poses=self.__get_poses_from_corners(detection,
                                                                           camera_params=camera_params,
                                                                           tag_size=tag_size))
                for detection in detector.detect(image)]

    @property
    def name(self):
        return f'pnp-{self.__method.name.lower()}'

    def __get_poses_from_corners(self,
                                 detection: AprilTagDetection,
                                 camera_params: CameraParameters,
                                 tag_size: float) -> List[Transform]:
        object_points = np.array([
            [-1, +1, 0],
            [+1, +1, 0],
            [+1, -1, 0],
            [-1, -1, 0],
        ]) / 2 * tag_size
        image_points = detection.corners

        return solve_pnp(object_points, image_points, camera_params, method=self.__method,
                         object_points_frame='tag_optical')
