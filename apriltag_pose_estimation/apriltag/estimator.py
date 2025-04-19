from typing import List, Unpack

import numpy as np
import numpy.typing as npt

from .estimation import AprilTagPoseEstimationStrategy
from ..core.camera import CameraParameters
from ..core.detection import AprilTagDetection, AprilTagDetector, AprilTagDetectorParams

__all__ = ['AprilTagPoseEstimator']


class AprilTagPoseEstimator:
    """An AprilTagPoseEstimator estimates the positions of AprilTags in images."""
    def __init__(self,
                 strategy: AprilTagPoseEstimationStrategy,
                 tag_size: float,
                 camera_params: CameraParameters,
                 **detector_kwargs: Unpack[AprilTagDetectorParams]):
        """
        Initializes a new AprilTagPoseEstimator.
        :param strategy: The strategy to use to estimate the positions of AprilTags.
        :param tag_size: The size of the AprilTags which will be detected, in meters.
        :param camera_params: Parameters of the camera used to take the image.
        :param detector_kwargs: Keyword arguments to pass to the detector (see :class:`AprilTagDetector`).
        """
        self.__detector = AprilTagDetector(**detector_kwargs)
        self.__strategy = strategy
        self.__tag_size = tag_size
        self.__camera_params = camera_params

    def estimate_tag_pose(self, image: npt.NDArray[np.uint8]) -> List[AprilTagDetection]:
        """
        Estimates the poses of detectable AprilTags in the image.
        :param image: The image in which tags will be detected. The camera used to take the image should be the same
                      camera for which parameters were passed to this estimator.
        :return: A list of all AprilTags which were detected. If there were no AprilTags detected, an empty list is
                 returned.
        """
        return self.__strategy.estimate_tag_pose(image=image,
                                                 detector=self.__detector,
                                                 tag_size=self.__tag_size,
                                                 camera_params=self.__camera_params)
