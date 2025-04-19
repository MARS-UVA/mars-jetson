import ctypes
import os
from collections.abc import Sequence
from pathlib import Path

import cv2
import numpy as np
import numpy.typing as npt

from ...core.bindings import AprilTagFamilyId, AprilTagLibrary, _image_u8_get_array, image_u8


class AprilTagImageGenerator:
    def __init__(self,
                 family: AprilTagFamilyId,
                 search_paths: Sequence[str | os.PathLike] = (
                     Path(__file__).parent / 'lib',
                     Path(__file__).parent / 'lib64'
                 )):
        self.__library = AprilTagLibrary(search_paths=search_paths)
        self.__family = self.__library.get_family(family)

    def generate_image(self, tag_id: int, tag_size: int | None = None) -> npt.NDArray[np.uint8]:
        if tag_id < 0:
            raise ValueError('tag_id must be non-negative')
        if tag_id >= self.__family.number_of_codes:
            raise IndexError(f'tag_id out of range for this family (max: {self.__family.number_of_codes - 1})')
        self.__library.libc.apriltag_to_image.restype = ctypes.POINTER(image_u8)
        c_img = self.__library.libc.apriltag_to_image(self.__family.c_ptr, tag_id)
        img = _image_u8_get_array(c_img).copy()
        img = img[:, :img.shape[0]]
        if tag_size is not None:
            img = cv2.resize(img, (tag_size, tag_size), interpolation=cv2.INTER_NEAREST_EXACT)
        return img
