from enum import IntEnum
from operator import attrgetter
from typing import List, Optional

import cv2
import numpy as np
from numpy import typing as npt

from .euclidean import Transform
from .camera import CameraParameters
from .exceptions import EstimationError


__all__ = ['PnPMethod', 'solve_pnp']


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


def solve_pnp(object_points: npt.NDArray[np.float64],
              image_points: npt.NDArray[np.float64],
              camera_params: CameraParameters,
              method: PnPMethod = PnPMethod.ITERATIVE,
              object_points_frame: Optional[str] = None) -> List[Transform]:
    """
    Solves the Perspective-N-Point problem.

    Internally, this uses OpenCV's solvePnP function. See
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html for more information.

    :param object_points: A nx3 array of 3D points corresponding to positions of the tracked objects in the
                          world frame. If the PnP solver method being used is ``PnPMethod.IPPE``, this must be
                          in the tag frame instead of the world frame.
    :param image_points: A nx2 array of 2D points corresponding to positions of the tracked objects in the image.
    :param camera_params: Parameters of the camera used to take the image.
    :param method: The method by which to solve the Perspective-N-Point problem (default: ``PnPMethod.ITERATIVE``).
    :param object_points_frame: The coordinate frame in which the object points are specified (default: None).
    :return: A list of all possible poses of the world origin in the camera frame, ordered from least to most
             reprojection error. The input space is the frame given by ``object_points_frame``, and the output space
             is "camera_optical".
    :raise EstimationError: If an error occurs in solving the Perspective-N-Point problem.
    """
    try:
        success, rotation_vectors, translations, errors = cv2.solvePnPGeneric(object_points,
                                                                              image_points,
                                                                              camera_params.get_matrix(),
                                                                              camera_params.get_distortion_vector(),
                                                                              flags=int(method))
    except cv2.error as e:
        raise EstimationError('Failed to solve') from e

    if not success:
        raise EstimationError('Failed to solve')
    return sorted((Transform.from_opencv_vectors(rotation_vector=rotation,
                                                 translation_vector=translation,
                                                 input_space=object_points_frame,
                                                 output_space='camera_optical',
                                                 error=float(error[0]))
                   for rotation, translation, error in zip(rotation_vectors, translations, errors)),
                  key=attrgetter('error'))
