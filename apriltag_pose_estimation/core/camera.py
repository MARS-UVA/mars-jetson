from dataclasses import dataclass

import numpy as np
from numpy import typing as npt


__all__ = ['CameraParameters',
           'DEPSTECH_CAM_PARAMETERS',
           'LOGITECH_CAM_PARAMETERS',
           'IPHONE_13_MINI_MAIN_CAM_PARAMETERS']


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


DEPSTECH_CAM_PARAMETERS = CameraParameters(fx=1329.143348,
                                           fy=1326.537785,
                                           cx=945.392392,
                                           cy=521.144703,
                                           k1=-0.348650,
                                           k2=0.098710,
                                           p1=-0.000157,
                                           p2=-0.001851,
                                           k3=0.000000)
"""Camera parameters for a Depstech webcam."""

LOGITECH_CAM_PARAMETERS = CameraParameters(fx=1303.4858439074037,
                                           fy=1313.7268166341282,
                                           cx=953.0550046450967,
                                           cy=487.428417101308,
                                           k1=-0.04617273252027395,
                                           k2=0.2753447226702122,
                                           p1=-0.010067837101803492,
                                           p2=-0.005296327017158184,
                                           k3=-0.38168395944619604)
"""Camera parameters for a Logitech C920 webcam."""

IPHONE_13_MINI_MAIN_CAM_PARAMETERS = CameraParameters(fx=975.0944324169702,
                                                      fy=975.7210449202735,
                                                      cx=651.2516360655774,
                                                      cy=354.20133381181955,
                                                      k1=0.16202747407118365,
                                                      k2=-0.8339518338379913,
                                                      p1=-0.0021712184519895234,
                                                      p2=0.003869488985826069,
                                                      k3=1.326544376051906)
"""Camera parameters for the iPhone 13 mini's main back camera."""
