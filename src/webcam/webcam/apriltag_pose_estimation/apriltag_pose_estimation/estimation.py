import abc
from dataclasses import dataclass
from typing import List

import numpy as np
import numpy.typing as npt
from pupil_apriltags import Detector

from .euclidean import Pose


__all__ = ['CameraParameters', 'AprilTagDetection', 'AprilTagPoseEstimationStrategy', 'EstimationError']


class EstimationError(Exception):
    """An error occurred during estimation."""
    def __init__(self, msg: str):
        super().__init__(msg)


@dataclass(frozen=True, kw_only=True)
class CameraParameters:
    """Parameters that represent characteristic information of a camera."""
    fx: float
    """Focal length in the x direction."""
    fy: float
    """Focal length in the y direction."""
    cx: float
    """X coordinate of the optical center."""
    cy: float
    """Y coordinate of the optical center."""
    k1: float
    """First parameter of radial distortion."""
    k2: float
    """Second parameter of radial distortion."""
    p1: float
    """First parameter of tangential distortion."""
    p2: float
    """Second parameter of tangential distortion."""
    k3: float
    """Third parameter of radial distortion."""

    @classmethod
    def from_matrices(cls, camera_matrix: npt.NDArray[np.float64], distortion_vector: npt.NDArray[np.float64]):
        """Create a CameraParameters object from a camera matrix and distortion vector."""
        return cls(
            fx=float(camera_matrix[0, 0]),
            fy=float(camera_matrix[1, 1]),
            cx=float(camera_matrix[0, 2]),
            cy=float(camera_matrix[1, 2]),
            k1=float(distortion_vector[0]),
            k2=float(distortion_vector[1]),
            p1=float(distortion_vector[2]),
            p2=float(distortion_vector[3]),
            k3=float(distortion_vector[4]),
        )

    def get_matrix(self) -> npt.NDArray[np.float32]:
        """Returns a camera matrix created from the camera parameters."""
        camera_matrix = np.zeros((3, 3), dtype=np.float32)
        camera_matrix[0, 0] = self.fx
        camera_matrix[1, 1] = self.fy
        camera_matrix[0, 2] = self.cx
        camera_matrix[1, 2] = self.cy
        camera_matrix[2, 2] = 1
        return camera_matrix

    def get_distortion_vector(self) -> npt.NDArray[np.float32]:
        """Returns a distortion vector created from the camera parameters."""
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float32)


@dataclass(frozen=True)
class AprilTagDetection:
    """A detection of an AprilTag."""
    tag_id: int
    """The ID of the tag which was detected."""
    tag_family: str
    """The family of the tag which was detected."""
    center: npt.NDArray[np.uint8]
    """The location of the center of the detected tag in the image."""
    corners: npt.NDArray[np.uint8]
    """The location of the corners of the detected tag in the image."""
    decision_margin: float
    """A measure of the quality of the binary decoding process. Higher numbers roughly indicate better decodes."""
    hamming: int
    """The number of error bits which were corrected."""
    tag_poses: List[Pose]
    """Possible poses of the tag in the camera's coordinate frame, in order from best to worst."""

    def __post_init__(self):
        if not self.tag_poses:
            raise ValueError('tag_poses is empty')

    @property
    def best_tag_pose(self) -> Pose:
        """The best tag pose calculated."""
        return self.tag_poses[0]


class AprilTagPoseEstimationStrategy(abc.ABC):
    """A strategy for pose estimation of AprilTag tags."""
    @abc.abstractmethod
    def estimate_tag_pose(self,
                          image: npt.NDArray[np.uint8],
                          detector: Detector,
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
