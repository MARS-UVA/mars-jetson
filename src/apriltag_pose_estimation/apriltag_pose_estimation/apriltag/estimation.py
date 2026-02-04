import abc
from typing import List

import numpy as np
import numpy.typing as npt

from ..core.camera import CameraParameters
from ..core.detection import AprilTagDetection, AprilTagDetector

__all__ = ['AprilTagPoseEstimationStrategy']


class AprilTagPoseEstimationStrategy(abc.ABC):
    """A strategy for pose estimation of AprilTag tags."""
    @abc.abstractmethod
    def estimate_tag_pose(self,
                          image: npt.NDArray[np.uint8],
                          detector: AprilTagDetector,
                          camera_params: CameraParameters,
                          tag_size: float) -> List[AprilTagDetection]:
        """
        Estimates the poses of all detectable AprilTags in the provided image.
        :param image: The image in which tags will be detected.
        :param detector: An AprilTagDetector instance.
        :param camera_params: Parameters of the camera used to take the image.
        :param tag_size: The size of the tag, in meters.
        :return: A list of all AprilTags which were detected. If there were no AprilTags detected, an empty list is
                 returned.
        """
        pass

    @property
    @abc.abstractmethod
    def name(self):
        pass
