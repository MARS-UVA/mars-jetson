from collections.abc import Sequence, Callable
from typing import Optional, List

import numpy as np
from scipy.spatial.transform import Rotation

from ..estimation import PoseEstimationStrategy
from ...core.camera import CameraParameters
from ...core.detection import AprilTagDetection
from ...core.euclidean import Transform
from ...core.exceptions import EstimationError
from ...core.field import AprilTagField
from ...core.pnp import PnPMethod, solve_pnp


__all__ = ['MultiTagSpecialEstimationStrategy']


class MultiTagSpecialEstimationStrategy(PoseEstimationStrategy):
    """
    An estimation strategy which attempts to resolve ambiguous tag poses to compile a more accurate result.

    Unlike other strategies, this strategy requires knowledge of the camera's angle in the world frame (passed as the
    world origin's rotation in the camera frame). Each of the corners of the detected AprilTags is computed in the world
    frame, and each set of corners are used to solve the PnP problem to determine candidate poses for the world origin.
    Then only the solutions for each AprilTag which are closest to the actual angle kept, and their translations and
    rotations are averaged to produce a final estimate.

    If there is only one detected AprilTag or if the PnP solver fails on all AprilTags, a fallback strategy is used.

    This method relies on having an accurate source of the camera's angle, such as with an IMU.
    """
    def __init__(self,
                 angle_producer: Callable[[], Rotation],
                 fallback_strategy: Optional[PoseEstimationStrategy] = None,
                 pnp_method: PnPMethod = PnPMethod.IPPE):
        """
        :param angle_producer: A function which returns the most recently measured angle of the world origin in the
                               camera frame as a rotation. This should be a non-pure function.
        :param fallback_strategy: A strategy to use if only one AprilTag was detected or the PnP solver fails. Cannot
                                  be a :class:`MultiTagSpecialEstimationStrategy`.
        :param pnp_method: A method the strategy will use to solve the Perspective-N-Point problem. Defaults to
                           ``PnPMethod.IPPE``.
        """
        super().__init__()
        if isinstance(fallback_strategy, MultiTagSpecialEstimationStrategy):
            raise TypeError('multitag fallback strategy cannot be another multitag special strategy')
        self.__angle_producer = angle_producer
        self.__fallback_strategy = fallback_strategy
        self.__pnp_method = pnp_method

    @property
    def name(self) -> str:
        return (f'multitag-special-{self.__pnp_method.name}-{self.__fallback_strategy}'
                if self.__fallback_strategy is not None
                else f'multitag-special-{self.__pnp_method.name}')

    def estimate_pose(self, detections: Sequence[AprilTagDetection], field: AprilTagField,
                      camera_params: CameraParameters) -> Optional[Transform]:
        if not detections:
            return None
        if len(detections) == 1:
            return self.__use_fallback_strategy(detections, field, camera_params)

        poses: List[Transform] = []
        actual_rotation = self.__angle_producer()
        for detection in detections:
            object_points = field.get_corners(detection.tag_id)
            image_points = detection.corners
            try:
                pose_candidates = solve_pnp(object_points,
                                            image_points,
                                            camera_params,
                                            method=self.__pnp_method,
                                            object_points_frame=field[detection.tag_id].output_space)
            except EstimationError:
                continue
            poses.append(min(pose_candidates,
                             key=lambda pose: Rotation.from_matrix(pose.rotation.as_matrix() @ actual_rotation.as_matrix())
                                                      .magnitude()))
        if not poses:
            return self.__use_fallback_strategy(detections, field, camera_params)
        if __debug__:
            input_space = poses[0].input_space
            output_space = poses[0].output_space
            assert all(pose.input_space == input_space and pose.output_space == output_space for pose in poses)
        weights = np.array([1 / pose.error for pose in poses])
        translation = np.average(np.vstack([pose.translation for pose in poses]),
                                 axis=0,
                                 weights=weights)
        rotation = Rotation.from_matrix(np.array([pose.rotation.as_matrix() for pose in poses])).mean(weights=weights)
        return Transform.make(rotation=rotation,
                              translation=translation,
                              input_space=poses[0].input_space,
                              output_space=poses[0].output_space)

    def __repr__(self) -> str:
        return (f'{type(self).__name__}(angle_producer={self.__angle_producer!r}, '
                f'fallback_strategy={self.__fallback_strategy!r}, '
                f'pnp_method={self.__pnp_method!r})')

    def __use_fallback_strategy(self, detections: Sequence[AprilTagDetection], field: AprilTagField,
                                camera_params: CameraParameters) -> Optional[Transform]:
        if self.__fallback_strategy is None:
            return None
        return self.__fallback_strategy.estimate_pose(detections, field, camera_params)
