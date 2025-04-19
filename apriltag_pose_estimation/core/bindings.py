"""
This module contains bindings for the C AprilTag 3 library.

It includes structure types for several important C AprilTag structs and convenience classes for accessing the AprilTag
C library and creating family objects.

Original author: Isaac Dulin, Spring 2016
Updates: Matt Zucker, Fall 2016
Apriltags 3 version: Aleksandar Petrov, Spring 2019
apriltag_pose_estimation version: Ivan Post, Spring 2025
"""

import ctypes
import logging
import os
import platform
import sys
from collections.abc import Sequence, Iterator
from contextlib import ExitStack
from itertools import chain
from pathlib import Path
from typing import TypeAlias, Literal, Any

import numpy as np
import numpy.typing as npt


logger = logging.getLogger(__name__)


# noinspection PyPep8Naming
class image_u8(ctypes.Structure):
    """Wraps the image_u8 C struct."""

    _fields_ = [
        ("width", ctypes.c_int32),
        ("height", ctypes.c_int32),
        ("stride", ctypes.c_int32),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


# noinspection PyPep8Naming
class matd(ctypes.Structure):
    """Wraps the matd C struct."""

    _fields_ = [
        ("nrows", ctypes.c_uint),
        ("ncols", ctypes.c_uint),
        ("data", ctypes.c_double * 1),
    ]


# noinspection PyPep8Naming
class zarray(ctypes.Structure):
    """Wraps the zarray C struct."""

    _fields_ = [
        ("el_sz", ctypes.c_size_t),
        ("size", ctypes.c_int),
        ("alloc", ctypes.c_int),
        ("data", ctypes.c_void_p),  # This is supposed to be void* instead of char*
    ]


# noinspection PyPep8Naming
class apriltag_family(ctypes.Structure):
    """Wraps the apriltag_family C struct."""

    _fields_ = [
        ("ncodes", ctypes.c_uint32),
        ("codes", ctypes.POINTER(ctypes.c_uint64)),
        ("width_at_border", ctypes.c_int),
        ("total_width", ctypes.c_int),
        ("reversed_border", ctypes.c_bool),
        ("nbits", ctypes.c_uint32),
        ("bit_x", ctypes.POINTER(ctypes.c_uint32)),
        ("bit_y", ctypes.POINTER(ctypes.c_uint32)),
        ("h", ctypes.c_int32),
        ("name", ctypes.c_char_p),
        ("impl", ctypes.c_void_p),
    ]


# noinspection PyPep8Naming
class apriltag_detection(ctypes.Structure):
    """Wraps apriltag_detection C struct."""

    _fields_ = [
        ("family", ctypes.POINTER(apriltag_family)),
        ("id", ctypes.c_int),
        ("hamming", ctypes.c_int),
        ("decision_margin", ctypes.c_float),
        ("H", ctypes.POINTER(matd)),
        ("c", ctypes.c_double * 2),
        ("p", (ctypes.c_double * 2) * 4),
    ]


# noinspection PyPep8Naming
class apriltag_detector(ctypes.Structure):
    """Wraps apriltag_detector C struct."""

    _fields_ = [
        ("nthreads", ctypes.c_int),
        ("quad_decimate", ctypes.c_float),
        ("quad_sigma", ctypes.c_float),
        ("refine_edges", ctypes.c_bool),
        ("decode_sharpening", ctypes.c_double),
        ("debug", ctypes.c_bool),
    ]


# noinspection PyPep8Naming
class apriltag_detection_info(ctypes.Structure):
    """Wraps apriltag_detection_info C struct."""

    _fields_ = [
        ("det", ctypes.POINTER(apriltag_detection)),
        ("tagsize", ctypes.c_double),
        ("fx", ctypes.c_double),
        ("fy", ctypes.c_double),
        ("cx", ctypes.c_double),
        ("cy", ctypes.c_double),
    ]


# noinspection PyPep8Naming
class apriltag_pose(ctypes.Structure):
    """Wraps apriltag_pose C struct."""

    _fields_ = [
        ("R", ctypes.POINTER(matd)),
        ("t", ctypes.POINTER(matd)),
    ]


def array_ptr_to_array2d(datatype: Any, ptr: Any, rows: int, cols: int) -> npt.NDArray:
    """
    Converts the representation of the data in a 2D C array to an equivalent NumPy array.

    Note that the resulting array still points to the underlying data. Users will often need to copy the resulting array
    to prevent modification of the original data.

    :param datatype: Data type of the array as a ctypes type.
    :param ptr: C pointer to the array.
    :param rows: Number of rows in the array.
    :param cols: Number of columns in the array.
    :return: A 2D NumPy array representing the underlying data.
    """
    array_type = (datatype * cols) * rows
    array_buf = array_type.from_address(ctypes.addressof(ptr))
    return np.ctypeslib.as_array(array_buf, shape=(rows, cols))


def _image_u8_get_array(img_ptr: Any) -> npt.NDArray[np.uint8]:
    """
    Converts the contents of an image_u8 to an equivalent NumPy array. The resulting image array is a 2D array.

    Note that the resulting array still points to the underlying data. Users will often need to copy the resulting array
    to prevent modification of the original data.

    :param img_ptr: C pointer to the image_u8 data.
    :return: A 2D NumPy array representing the underlying image data.
    """
    return array_ptr_to_array2d(ctypes.c_uint8, img_ptr.contents.buf.contents, img_ptr.contents.height,
                                img_ptr.contents.stride)


def _matd_get_array(mat_ptr: Any) -> npt.NDArray[np.cdouble]:
    """
    Converts the contents of a matd to an equivalent NumPy array.

    :param mat_ptr: C pointer to the matd data.
    :return: A 2D NumPy array representing the underlying matrix.
    """
    return array_ptr_to_array2d(ctypes.c_double, mat_ptr.contents.data, int(mat_ptr.contents.nrows),
                                int(mat_ptr.contents.ncols))


def zarray_get(za: Any, idx: int, ptr: Any):
    """
    Gets the item from the zarray at the given index and store it at the given pointer.

    :param za: The zarray to access.
    :param idx: The index at which the zarray will be accessed.
    :param ptr: C pointer at which the value in the zarray at the given index will be written.
    """

    ctypes.memmove(ptr, za.contents.data + idx * za.contents.el_sz, za.contents.el_sz)


AprilTagFamilyId: TypeAlias = Literal['tag16h5',
                                      'tag25h9',
                                      'tag36h11',
                                      'tagCircle21h7',
                                      'tagCircle49h12',
                                      'tagCustom48h12',
                                      'tagStandard41h12',
                                      'tagStandard52h13']


class AprilTagFamily:
    """An AprilTag family."""

    VALID_FAMILIES = {
        'tag16h5',
        'tag25h9',
        'tag36h11',
        'tagCircle21h7',
        'tagCircle49h12',
        'tagCustom48h12',
        'tagStandard41h12',
        'tagStandard52h13'
    }

    def __init__(self, library: 'AprilTagLibrary', family_name: AprilTagFamilyId):
        """
        Initializes an AprilTag family.
        :param library: The AprilTag library to use.
        :param family_name: The name of the family.
        :raises ValueError: If the family name is invalid.
        """
        self.__libc = library.libc

        if family_name not in self.VALID_FAMILIES:
            raise ValueError(f'{family_name} is not a valid AprilTag family')
        create_func = self.__libc[f'{family_name}_create']
        create_func.restype = ctypes.POINTER(apriltag_family)
        self.__ptr = create_func()
        self.__family_name = family_name

    @property
    def family_name(self) -> str:
        """Name of the AprilTag family."""
        return self.__family_name

    @property
    def c_value(self) -> apriltag_family:
        """Underlying C apriltag_family object."""
        return self.__ptr.contents

    @property
    def c_ptr(self) -> Any:
        """Underlying C apriltag_family* pointer."""
        return self.__ptr

    @property
    def number_of_codes(self) -> int:
        """The number of codes in this family."""
        return self.__ptr.contents.ncodes

    def __del__(self):
        destroy_func = self.__libc[f'{self.__family_name}_destroy']
        destroy_func(self.__ptr)


def __get_env_path() -> Path:
    return Path(sys.executable).parents[1]


default_search_paths = (__get_env_path() / 'lib',
                        __get_env_path() / 'lib64')


class AprilTagLibrary:
    """
    A class for objects which hold the AprilTag C shared library. It provides access to the underlying
    :py:class:`ctypes.CDLL` object for accessing C functions, as well as a couple of convenience methods for creating
    AprilTag families dynamically from a family name string.

    To initialize the AprilTag library, use the :py:meth:`load` class method and provide a sequence of search paths, if
    any (the default will usually if the library was installed via pip).
    """
    _found_libraries: 'dict[tuple[str | os.PathLike, ...], AprilTagLibrary]' = {}

    def __init__(self, search_paths: Sequence[str | os.PathLike] = default_search_paths):
        """
        Initializes a new AprilTagLibrary object.

        Use :py:meth:`load` instead, since it will cache libraries which were already found with the provided search
        paths.
        :param search_paths: A sequence of search paths relative to the
        """
        self.__libc = AprilTagLibrary._get_apriltag_library(search_paths=search_paths)

    @classmethod
    def load(cls, search_paths: Sequence[str | os.PathLike] = default_search_paths):
        """
        Loads an AprilTag library which is located at one of the provided search paths.

        The search paths will be searched in the order they are provided for the shared library. The name expected
        varies by platform; on Linux it is ``libapriltag*.so*``, on Windows it is ``*apriltag*.dll``, and on macOS it is
        ``libapriltag*.dylib``. The default search paths should work if the library was installed with pip.
        :param search_paths: Paths to search for the AprilTag C library.
        :return: A new :py:class:`AprilTagLibrary` object.
        """
        search_paths = tuple(search_paths)
        if search_paths in cls._found_libraries:
            return cls._found_libraries[search_paths]
        instance = cls(search_paths=search_paths)
        cls._found_libraries[search_paths] = instance
        return instance

    @property
    def libc(self) -> ctypes.CDLL:
        """The underlying :py:class:`ctypes.CDLL` object."""
        return self.__libc

    def get_family(self, family: AprilTagFamilyId) -> AprilTagFamily:
        """
        Gets the :py:class:`AprilTagFamily` object for the given family name.
        :param family: The AprilTag family name.
        :return: A :py:class:`AprilTagFamily` object for the given family name.
        """
        return AprilTagFamily(self, family)

    @staticmethod
    def _get_apriltag_library(search_paths: Sequence[str | os.PathLike] = (
            Path(__file__).parent / 'lib',
            Path(__file__).parent / 'lib64')
    ) -> ctypes.CDLL:
        """
        Gets the AprilTag shared library.

        :param search_paths: Paths to search for the library.
        :return: A :class:`ctypes.CDLL` instance.
        """
        filename_patterns_by_platform = {
            'Darwin': 'libapriltag*.dylib',
            'Linux': 'libapriltag*.so*',
            "Windows": '*apriltag*.dll',
        }
        filename_pattern = filename_patterns_by_platform[platform.system()]

        possible_hits: Iterator[Path] = chain.from_iterable(
            Path(path).glob(filename_pattern) for path in search_paths
        )
        stack = ExitStack()
        for hit in possible_hits:
            with stack:
                if platform.system() == 'Windows':
                    dll_path = str(hit.parent.resolve())
                    stack.enter_context(os.add_dll_directory(dll_path))
                libc = ctypes.CDLL(str(hit))
                if libc:
                    return libc
        raise FileNotFoundError(f'Could not find clib with pattern {filename_pattern} in any of {search_paths}')
