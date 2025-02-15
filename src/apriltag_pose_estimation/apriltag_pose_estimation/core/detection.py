from dataclasses import dataclass
from typing import Optional, List

import numpy as np
from numpy import typing as npt

from .euclidean import Transform


__all__ = ['AprilTagDetection']


@dataclass(frozen=True)
class AprilTagDetection:
    """A detection of an AprilTag."""
    tag_id: int
    """The ID of the tag which was detected."""
    tag_family: str
    """The family of the tag which was detected."""
    center: npt.NDArray[np.float64]
    """The location of the center of the detected tag in the image."""
    corners: npt.NDArray[np.float64]
    """The location of the corners of the detected tag in the image."""
    decision_margin: float
    """A measure of the quality of the binary decoding process. Higher numbers roughly indicate better decodes."""
    hamming: int
    """The number of error bits which were corrected."""
    tag_poses: Optional[List[Transform]] = None
    """Possible poses of the tag in the camera's coordinate frame, in order from best to worst."""

    def __post_init__(self):
        if self.tag_poses is not None and not self.tag_poses:
            raise ValueError('tag_poses is empty but not None')

    @property
    def best_tag_pose(self) -> Optional[Transform]:
        """The best tag pose calculated."""
        return self.tag_poses[0] if self.tag_poses is not None else None
