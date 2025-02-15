from collections.abc import Sequence
from typing import Optional

import numpy as np

from ..estimation import PoseEstimationStrategy
from ...core.camera import CameraParameters
from ...core.detection import AprilTagDetection
from ...core.euclidean import Transform
from ...core.exceptions import EstimationError
from ...core.field import AprilTagField
from ...core.pnp import PnPMethod, solve_pnp


__all__ = ['MultiTagPnPEstimationStrategy']


class MultiTagPnPEstimationStrategy(PoseEstimationStrategy):
    """
    An estimation strategy which solves the Perspective-N-Point problem across all detected AprilTag corner points.

    This strategy is implemented with OpenCV's solvePnP function. See
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html for more information.

    Each of the corners of the detected AprilTags is computed in the world frame, and these points are passed into the
    PnP solver. If there is only one detected AprilTag or if the PnP solver fails, a fallback strategy is used.

    This implementation derives heavily from MultiTag pose estimation in PhotonVision (see
    https://github.com/PhotonVision/photonvision/blob/d78f2b8650ac21a54eba94c83931f7b3dd1e32f2/photon-targeting/src/main/java/org/photonvision/estimation/VisionEstimation.java#L122).
    """
    def __init__(self, fallback_strategy: Optional[PoseEstimationStrategy] = None,
                 pnp_method: PnPMethod = PnPMethod.SQPNP):
        """
        :param fallback_strategy: A strategy to use if only one AprilTag was detected or the PnP solver fails. Cannot
                                  be a :class:`MultiTagPnPEstimationStrategy`.
        :param pnp_method: A method the strategy will use to solve the Perspective-N-Point problem. Cannot be
                           ``PnPMethod.AP3P`` or ``PnPMethod.IPPE``. Defaults to ``PnPMethod.SQPNP``.
        """
        super().__init__()
        if isinstance(fallback_strategy, MultiTagPnPEstimationStrategy):
            raise TypeError('multitag fallback strategy cannot be another multitag PnP strategy')
        if pnp_method is PnPMethod.AP3P:
            raise ValueError('PnP method cannot be AP3P for multitag PnP estimation')
        if pnp_method is PnPMethod.IPPE:
            raise ValueError('PnP method cannot be IPPE for multitag PnP estimation')
        self.__fallback_strategy = fallback_strategy
        self.__pnp_method = pnp_method

    @property
    def name(self) -> str:
        return (f'multitag-pnp-{self.__pnp_method.name}-{self.__fallback_strategy.name}'
                if self.__fallback_strategy is not None
                else f'multitag-pnp-{self.__pnp_method.name}')

    def estimate_pose(self, detections: Sequence[AprilTagDetection], field: AprilTagField,
                      camera_params: CameraParameters) -> Optional[Transform]:
        if not detections:
            return None
        if len(detections) == 1:
            return self.__use_fallback_strategy(detections, field, camera_params)

        object_points = field.get_corners(*(detection.tag_id for detection in detections))
        image_points = np.vstack([detection.corners for detection in detections])

        try:
            poses = solve_pnp(object_points, image_points, camera_params, method=self.__pnp_method,
                              object_points_frame=field.output_space)
        except EstimationError:
            return self.__use_fallback_strategy(detections, field, camera_params)
        if not poses:
            return self.__use_fallback_strategy(detections, field, camera_params)

        return poses[0]

    def __repr__(self) -> str:
        return f'{type(self).__name__}(fallback_strategy={self.__fallback_strategy!r}, pnp_method={self.__pnp_method!r})'

    def __use_fallback_strategy(self, detections: Sequence[AprilTagDetection], field: AprilTagField,
                                camera_params: CameraParameters) -> Optional[Transform]:
        if self.__fallback_strategy is None:
            return None
        return self.__fallback_strategy.estimate_pose(detections, field, camera_params)
