import numpy as np
import numpy.typing as npt
import cv2

from ..estimation import CameraParameters


class Distorter:
    def __init__(self, camera_params: CameraParameters, width: int, height: int, alpha: float = 0.0):
        camera_matrix = camera_params.get_matrix()
        distortion_coefficients = camera_params.get_distortion_vector()
        new_camera_matrix = cv2.getOptimalNewCameraMatrix(camera_matrix,
                                                          distortion_coefficients,
                                                          (width, height),
                                                          alpha,
                                                          (width, height))
        map_x, map_y = cv2.initUndistortRectifyMap(camera_matrix,
                                                   distortion_coefficients,
                                                   None,
                                                   new_camera_matrix,
                                                   (width, height),
                                                   cv2.CV_32F)

        self.__mapping = np.zeros((height, width, 2), np.float32)
        self.__mapping[:, :, 0], self.__mapping[:, :, 1] = map_x, map_y
        self.__inverse_mapping = self.__get_inverse_map(self.__mapping, width, height)

    def undistort(self, image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        return cv2.remap(image, self.__mapping, None, cv2.INTER_LINEAR)

    def distort(self, image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        return cv2.remap(image, self.__inverse_mapping, None, cv2.INTER_LINEAR)

    @staticmethod
    def __get_inverse_map(mapping: npt.NDArray[np.float32], width: int, height: int, iterations: int = 10):
        indices = np.zeros_like(mapping)
        indices[:, :, 1], indices[:, :, 0] = np.indices((height, width), dtype=np.float32)
        inverse = np.copy(indices)

        for _ in range(iterations):
            inverse += indices - cv2.remap(mapping, inverse, None, interpolation=cv2.INTER_LINEAR)
        return inverse