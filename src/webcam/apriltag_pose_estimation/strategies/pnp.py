from enum import IntEnum
from typing import List

import cv2
import numpy as np
import numpy.typing as npt
from pupil_apriltags import Detector, Detection

from ..estimation import AprilTagPoseEstimationStrategy, CameraParameters, AprilTagDetection, Pose, EstimationError


__all__ = ['PerspectiveNPointStrategy', 'PnPMethod']


class PnPMethod(IntEnum):
    """A method for solving the Perspective-N-Point problem."""
    ITERATIVE = cv2.SOLVEPNP_ITERATIVE
    """
    A method based on Levenberg-Marquardt optimization of the reprojection error. It refines an initial estimate made
    using the homography decomposition.
    """
    AP3P = cv2.SOLVEPNP_AP3P
    """
    A method based on the paper "An Efficient Algebraic Solution to the Perspective-Three-Point Problem" by T. Ke and S.
    Roumeliotis [1].
    
    [1]: Tong Ke and Stergios Roumeliotis. An efficient algebraic solution to the perspective-three-point problem.
        In Computer Vision and Pattern Recognition (CVPR), 2017 IEEE Conference on. IEEE, 2017.
    """
    IPPE = cv2.SOLVEPNP_IPPE_SQUARE
    """
    A method based on the paper "Infinitesimal Plane-Based Pose Estimation" by T. Collins and A. Bartoli [1].
    
    [1]: Toby Collins and Adrien Bartoli. Infinitesimal plane-based pose estimation. International Journal of Computer
        Vision, 109(3):252–286, 2014.
    """
    SQPNP = cv2.SOLVEPNP_SQPNP
    """
    A method based on the paper "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem"
    by G. Terzakis and M. Lourakis [1].
    
    [1]: George Terzakis and Manolis Lourakis. A consistently fast and globally optimal solution to the
        perspective-n-point problem. In European Conference on Computer Vision, pages 478–494. Springer International
        Publishing, 2020.
    """


class PerspectiveNPointStrategy(AprilTagPoseEstimationStrategy):
    """
    A pose estimation strategy which uses OpenCV's solvePnP function.

    See also: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
    """
    def __init__(self, method: PnPMethod = PnPMethod.ITERATIVE):
        self.__method = method

    @property
    def method(self):
        """The method which this strategy is using for Perspective-N-Point."""
        return self.__method

    def estimate_tag_pose(self,
                          image: npt.NDArray[np.uint8],
                          detector: Detector,
                          camera_params: CameraParameters,
                          tag_size: float) -> List[AprilTagDetection]:
        detection: Detection
        # noinspection PyTypeChecker
        return [AprilTagDetection(tag_id=detection.tag_id,
                                  tag_family=detection.tag_family,
                                  center=detection.center,
                                  corners=detection.corners,
                                  decision_margin=detection.decision_margin,
                                  hamming=detection.hamming,
                                  tag_pose=self.__get_pose_from_corners(detection, camera_params, tag_size))
                for detection in detector.detect(image)]

    def __get_pose_from_corners(self,
                                detection: Detection,
                                camera_params: CameraParameters,
                                tag_size: float) -> Pose:
        object_points = np.array([
            [-1, +1, 0],
            [+1, +1, 0],
            [+1, -1, 0],
            [-1, -1, 0],
        ]) / 2 * tag_size
        image_points = detection.corners

        success, rotation_vector, translation = cv2.solvePnP(object_points,
                                                             image_points,
                                                             camera_params.get_matrix(),
                                                             camera_params.get_distortion_vector(),
                                                             flags=int(self.__method))
        if not success:
            raise EstimationError('Failed to solve')
        rotation, _ = cv2.Rodrigues(rotation_vector)
        return Pose(rotation, translation)
