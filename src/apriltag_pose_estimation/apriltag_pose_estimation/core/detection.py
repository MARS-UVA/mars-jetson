import ctypes
import logging
import os
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Optional, List, TypedDict, NotRequired, Any

import numpy as np
from numpy import typing as npt
from scipy.spatial.transform import Rotation

from . import CameraParameters
from .bindings import (apriltag_detector, zarray, apriltag_detection, zarray_get, _matd_get_array,
                       apriltag_detection_info, apriltag_pose, image_u8, _image_u8_get_array, AprilTagFamilyId,
                       AprilTagLibrary, AprilTagFamily, default_search_paths)
from .euclidean import Transform


__all__ = ['AprilTagDetection', 'AprilTagDetector']


logger = logging.getLogger(__name__)


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
    """
    The location of the corners of the detected tag in the image.
    
    The corners are provided in the order of (-1, 1), (1, 1), (1, -1), (-1, -1). It should be noted that this is the
    order OpenCV's solvePnP function uses under the IPPE square mode, *not* the convention used by the
    C AprilTag library.
    """
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


class AprilTagDetectorParams(TypedDict):
    families: NotRequired[AprilTagFamilyId | Sequence[AprilTagFamilyId]]
    nthreads: NotRequired[int]
    quad_decimate: NotRequired[float]
    quad_sigma: NotRequired[float]
    refine_edges: NotRequired[bool]
    decode_sharpening: NotRequired[float]
    debug: NotRequired[bool]
    search_paths: NotRequired[Sequence[str | os.PathLike]]


class AprilTagDetector:
    """A detector for AprilTags in images."""

    def __init__(self,
                 families: AprilTagFamilyId | Sequence[AprilTagFamilyId] = 'tagStandard41h12',
                 nthreads: int = 1,
                 quad_decimate: float = 2.0,
                 quad_sigma: float = 0.0,
                 refine_edges: bool = True,
                 decode_sharpening: float = 0.25,
                 debug: bool = False,
                 search_paths: Sequence[str | os.PathLike] = default_search_paths):
        """
        Initializes a new AprilTagDetector.
        :param families: Sequence of names of AprilTag families the detector will detect.
        :param nthreads: Number of threads on which AprilTag detection is running.
        :param quad_decimate: Amount by which to decimate the image.
        :param quad_sigma: Amount of Gaussian blur to apply to the segmented image to detect quads, as a standard
                           deviation in pixels.
        :param refine_edges: If ``True``, the edges of each quad are adjusted to "snap to" strong gradients nearby.
        :param decode_sharpening: Amount by which decoded images will be sharpened.
        :param debug: If ``True``, write a variety of debugging images to the current working directory at various
                      stages in the detection process.
        :param search_paths: Paths to search for the AprilTag C library.
        """
        self.__ptr = None
        self.__library = AprilTagLibrary(search_paths=search_paths)
        self.__library.libc.apriltag_detector_create.restype = ctypes.POINTER(apriltag_detector)
        self.__ptr = self.__library.libc.apriltag_detector_create()

        self.__tag_families: dict[AprilTagFamilyId, AprilTagFamily] = {}
        if isinstance(families, str):
            families = [families]
        family: AprilTagFamilyId
        for family in families:
            self.add_family(family)

        self.__ptr.contents.nthreads = int(nthreads)
        self.__ptr.contents.quad_decimate = float(quad_decimate)
        self.__ptr.contents.quad_sigma = float(quad_sigma)
        self.__ptr.contents.refine_edges = bool(refine_edges)
        self.__ptr.contents.decode_sharpening = int(decode_sharpening)
        self.__ptr.contents.debug = bool(debug)

    @property
    def c_value(self) -> apriltag_detector:
        """Underlying C apriltag_detector value."""
        return self.__ptr.contents

    @property
    def c_ptr(self) -> Any:
        """Underlying C apriltag_detector* pointer."""
        return self.__ptr

    @property
    def families(self) -> list[AprilTagFamilyId]:
        """List of names of AprilTag families the detector can detect."""
        return list(self.__tag_families.keys())

    @property
    def nthreads(self) -> int:
        """The number of threads on which AprilTag detection is running."""
        return self.__ptr.contents.nthreads

    @nthreads.setter
    def nthreads(self, value: int):
        self.__ptr.contents.nthreads = int(value)

    @property
    def quad_decimate(self) -> float:
        """
        The amount by which to decimate the image.

        Decimation reduces the resolution of the image, improving speed at a cost of pose accuracy and slight decrease
        in detection rate.
        """
        return self.__ptr.contents.quad_decimate

    @quad_decimate.setter
    def quad_decimate(self, value: float):
        self.__ptr.contents.quad_decimate = float(value)

    @property
    def quad_sigma(self) -> float:
        """
        The amount of Gaussian blur to apply to the segmented image to detect quads, as a standard deviation in pixels.

        Very noisy images benefit from non-zero values.
        """
        return self.__ptr.contents.quad_sigma

    @quad_sigma.setter
    def quad_sigma(self, value: float):
        self.__ptr.contents.quad_sigma = float(value)

    @property
    def refine_edges(self) -> bool:
        """
        If ``True``, the edges of each quad are adjusted to "snap to" strong gradients nearby.

        This is useful when decimation is employed, as it can increase the quality of the initial quad estimate
        substantially. Generally recommended to be set to ``True``.
        :return:
        """
        return self.__ptr.contents.refine_edges

    @refine_edges.setter
    def refine_edges(self, value: bool):
        self.__ptr.contents.refine_edges = bool(value)

    @property
    def decode_sharpening(self) -> int:
        """
        The amount by which decoded images will be sharpened.

        This can help decode small tags but may or may not help in poor lighting conditions.
        """
        return self.__ptr.contents.decode_sharpening

    @decode_sharpening.setter
    def decode_sharpening(self, value: int):
        self.__ptr.contents.decode_sharpening = int(value)

    @property
    def debug(self) -> bool:
        """
        If ``True``, write a variety of debugging images to the current working directory at various stages in the
        detection process.

        This is somewhat slow.
        """
        return self.__ptr.contents.debug

    @debug.setter
    def debug(self, value: bool):
        self.__ptr.contents.debug = bool(value)

    def add_family(self, family: AprilTagFamilyId) -> None:
        """
        Adds the given AprilTag family to the set of families the detector can detect. If the family was already added,
        this method does nothing.
        :param family: Name of the family to add.
        """
        self.__library.libc.apriltag_detector_add_family_bits.restype = None
        if family not in self.__tag_families:
            self.__tag_families[family] = self.__library.get_family(family)
            self.__library.libc.apriltag_detector_add_family_bits(
                self.__ptr, self.__tag_families[family].c_ptr, 2
            )

    def remove_family(self, family: AprilTagFamilyId) -> None:
        """
        Removes the given AprilTag family from the set of families the detector can detect. If the family was not
        already added, this method does nothing.
        :param family: Name of the family to remove.
        """
        self.__library.libc.apriltag_detector_remove_family.restype = None
        if family in self.__tag_families:
            self.__library.libc.apriltag_detector_remove_family(self.__ptr, self.__tag_families[family].c_ptr)
            del self.__tag_families[family]

    def clear_families(self) -> None:
        """Unregisters all AprilTag families from the set which are detected."""
        self.__library.libc.apriltag_detector_clear_families.restype = None
        self.__library.libc.apriltag_detector_clear_families(self.__ptr)
        self.__tag_families.clear()

    def detect(self,
               img: npt.NDArray[np.uint8],
               estimate_tag_pose: bool = False,
               camera_params: CameraParameters | None = None,
               tag_size: float | None = None) -> List[AprilTagDetection]:
        """
        Detects AprilTags in the provided image.

        It will only detect AprilTags in the families set on this detector and at the provided size.

        If *estimate_tag_pose* is ``True``, *camera_params* and *tag_size* must be provided.
        :param img: Grayscale image in which AprilTags will be detected as a 2D NumPy array.
        :param estimate_tag_pose: If ``True``, estimate the pose of the AprilTag using an orthogonal iteration technique
                                  (the one used by the C AprilTag library).
        :param camera_params: Parameters for the camera used to take the image.
        :param tag_size: Distance between adjacent corner points of the tag, in meters.
        :return: A (possibly empty) list of :py:class:`AprilTagDetection` objects describing the detected AprilTags.
        """
        if len(img.shape) != 2:
            raise ValueError(f'img array must be two-dimensional; got {len(img.shape)} dimensions')
        if img.dtype != np.uint8:
            raise TypeError('img array dtype must be np.uint8')

        c_img = self.__convert_image(img)
        try:
            self.__library.libc.apriltag_detector_detect.restype = ctypes.POINTER(zarray)
            detections = self.__library.libc.apriltag_detector_detect(self.__ptr, c_img)
            try:
                detection_ptr = ctypes.POINTER(apriltag_detection)()
                results: list[AprilTagDetection] = []
                for index in range(detections.contents.size):
                    zarray_get(detections, index, ctypes.byref(detection_ptr))
                    detection = detection_ptr.contents

                    center = np.ctypeslib.as_array(detection.c, shape=(2,)).copy()
                    corners = np.ctypeslib.as_array(detection.p, shape=(4, 2)).copy()

                    if estimate_tag_pose:
                        if camera_params is None:
                            raise ValueError('camera_params is required if estimate_tag_pose is true')
                        if tag_size is None:
                            raise ValueError('tag_size is required if estimate_tag_pose is true')

                        info = apriltag_detection_info(
                            det=detection,
                            tagsize=tag_size,
                            fx=camera_params.fx,
                            fy=camera_params.fy,
                            cx=camera_params.cx,
                            cy=camera_params.cy,
                        )
                        pose = apriltag_pose()

                        self.__library.libc.estimate_tag_pose.restype = ctypes.c_double
                        err = self.__library.libc.estimate_tag_pose(
                            ctypes.byref(info), ctypes.byref(pose)
                        )

                        tag_pose = Transform.make(
                            rotation=Rotation.from_matrix(_matd_get_array(pose.R).copy()),
                            translation=_matd_get_array(pose.t).copy(),
                            input_space='tag_optical',
                            output_space='camera_optical',
                            error=err
                        )
                    else:
                        tag_pose = None

                    results.append(AprilTagDetection(
                        tag_family=ctypes.string_at(detection.family.contents.name).decode(),
                        tag_id=detection.id,
                        hamming=detection.hamming,
                        decision_margin=detection.decision_margin,
                        center=center,
                        corners=corners[(1, 0, 3, 2), :],
                        tag_poses=[tag_pose] if tag_pose is not None else None,
                    ))
            finally:
                self.__library.libc.apriltag_detections_destroy.restype = None
                self.__library.libc.apriltag_detections_destroy(detections)
        finally:
            self.__library.libc.image_u8_destroy.restype = None
            self.__library.libc.image_u8_destroy(c_img)

        return results

    def __del__(self):
        if self.__ptr is not None:
            self.__library.libc.apriltag_detector_destroy.restype = None
            self.__library.libc.apriltag_detector_destroy(self.__ptr)

    def __convert_image(self, img: npt.NDArray[np.uint8]) -> Any:
        """
        Converts an image in a NumPy array to an image_u8.
        :param img: A grayscale image stored in a 2D NumPy array.
        :return: C pointer to an image_u8.
        """
        height = img.shape[0]
        width = img.shape[1]

        self.__library.libc.image_u8_create.restype = ctypes.POINTER(image_u8)
        c_img = self.__library.libc.image_u8_create(width, height)

        img_array = _image_u8_get_array(c_img)
        img_array[:, :width] = img

        return c_img
