from dataclasses import dataclass
from typing import Optional, List, Unpack

import numpy as np
import numpy.typing as npt

from .estimation import PoseEstimationStrategy
from ..core.camera import CameraParameters
from ..core.detection import AprilTagDetection, AprilTagDetector, AprilTagDetectorParams
from ..core.euclidean import Transform
from ..core.field import AprilTagField


__all__ = ['PoseEstimator']


class PoseEstimator:
    """
    An estimator for the pose of a camera based on the positions of known AprilTags.

    Estimators take information about the field and camera as arguments, along with arguments for the AprilTag detector.
    They also need a :class:`PoseEstimationStrategy`, which tells the estimator how to determine the pose of the camera
    from a sequence of detected AprilTags.
    """

    @dataclass(frozen=True)
    class Result:
        estimated_pose: Optional[Transform]
        detections: List[AprilTagDetection]

    def __init__(self,
                 strategy: PoseEstimationStrategy,
                 field: AprilTagField,
                 camera_params: CameraParameters,
                 **detector_kwargs: Unpack[AprilTagDetectorParams]):
        """
        :param strategy: The strategy for this estimator to use.
        :param field: An :class:`AprilTagField` describing the positions of all the AprilTags on the field in the
                      world frame.
        :param camera_params: Characteristic parameters for the camera being used to detect AprilTags.
        :param detector_kwargs: Arguments to pass to the AprilTag detector (see :class:`AprilTagDetector`).
        """
        self.__strategy = strategy
        self.__field = field
        self.__camera_params = camera_params
        self.__detector = AprilTagDetector(families=self.__field.tag_family, **detector_kwargs)

    @property
    def strategy(self) -> PoseEstimationStrategy:
        return self.__strategy

    @property
    def field(self) -> AprilTagField:
        return self.__field

    @property
    def camera_params(self) -> CameraParameters:
        return self.__camera_params

    def estimate_pose(self, image: npt.NDArray[np.uint8]) -> 'PoseEstimator.Result':
        """
        Estimates the pose of the camera based on AprilTags in the given image.

        This function returns the estimated pose of the *world origin* in the *camera frame*, not the other way around.
        While this may seem unintuitive, it ends up being more useful when transforming the pose later.

        :param image: An image containing AprilTags taken by the camera described by this estimator's camera parameters
                      and camera pose.
        :return: The estimated pose of the world origin in the camera frame, or ``None`` if an estimate could not be
                 made.
        """
        detections = [detection
                      for detection in self.__detector.detect(img=image, estimate_tag_pose=False)
                      if detection.tag_id in self.__field and detection.tag_family == self.__field.tag_family]

        return self.Result(estimated_pose=self.__strategy.estimate_pose(detections,
                                                                        field=self.__field,
                                                                        camera_params=self.__camera_params),
                           detections=detections)
