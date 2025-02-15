from collections.abc import Sequence
from typing import Optional, List

import numpy as np

from ..estimation import PoseEstimationStrategy
from ...core.camera import CameraParameters
from ...core.detection import AprilTagDetection
from ...core.euclidean import Transform
from ...core.field import AprilTagField
from ...core.pnp import PnPMethod, solve_pnp


__all__ = ['LowestAmbiguityEstimationStrategy']


class LowestAmbiguityEstimationStrategy(PoseEstimationStrategy):
    """
    An estimation strategy which solves the Perspective-N-Point problem for each AprilTag's corner points and chooses
    the estimate with the lowest ambiguity.

    This strategy is implemented with OpenCV's solvePnP function. See
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html for more information.

    Each of the corners of the detected AprilTags is computed in the world frame, and these points are passed for each
    detected AprilTag individually into the PnP solver.

    This implementation derives heavily from PhotonVision (see
    https://github.com/PhotonVision/photonvision/blob/d78f2b8650ac21a54eba94c83931f7b3dd1e32f2/photon-targeting/src/main/java/org/photonvision/estimation/VisionEstimation.java#L103).
    """
    def __init__(self, pnp_method: PnPMethod = PnPMethod.IPPE):
        """
        :param pnp_method: A method the strategy will use to solve the Perspective-N-Point problem. Defaults to
        ``PnPMethod.IPPE``.
        """
        super().__init__()
        self.__pnp_method = pnp_method

    @property
    def name(self) -> str:
        return f'lowest-ambiguity-{self.__pnp_method.name}'

    def estimate_pose(self, detections: Sequence[AprilTagDetection], field: AprilTagField,
                      camera_params: CameraParameters) -> Optional[Transform]:
        if not detections:
            return None
        pose_candidates: List[Transform] = []
        for detection in detections:
            if self.__pnp_method is PnPMethod.IPPE:
                object_points = np.array([
                    [-1, +1, 0],
                    [+1, +1, 0],
                    [+1, -1, 0],
                    [-1, -1, 0],
                ]) / 2 * field.tag_size
                image_points = detection.corners
                tag_poses_in_camera = solve_pnp(object_points, image_points, camera_params, method=self.__pnp_method,
                                                object_points_frame='tag_optical')
                poses = [(pose @ field[detection.tag_id].inv()).with_error(pose.error) for pose in tag_poses_in_camera]
            else:
                object_points = field.get_corners(detection.tag_id)
                image_points = detection.corners
                poses = solve_pnp(object_points, image_points, camera_params, method=self.__pnp_method,
                                  object_points_frame=field[detection.tag_id].output_space)
            if not poses:
                continue
            if len(poses) == 1:
                best_pose = poses[0]
            else:
                best_pose = poses[0].with_ambiguity(poses[0].error / poses[1].error)
            pose_candidates.append(best_pose)
        if not pose_candidates:
            return

        return min(pose_candidates, key=lambda pose: pose.ambiguity if pose.ambiguity is not None else float('inf'))

    def __repr__(self) -> str:
        return f'{type(self).__name__}(pnp_method={self.__pnp_method!r})'
