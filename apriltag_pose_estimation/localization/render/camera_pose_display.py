from importlib.resources import files
from typing import Optional

try:
    from PyQt5 import Qt
    import pyvista as pv
    from pyvistaqt import BackgroundPlotter
except ImportError as e:
    e.add_note('to use 3D rendering features, install this package with the "render" extra')
    raise

from . import resource
from ... import tag_images
from ...core import Transform
from ...core.field import AprilTagField


__all__ = ['CameraPoseDisplay']


PLANE_SCALE_FACTORS: dict[str, float] = {
    'tag16h5': 8 / 6,
    'tag25h9': 9 / 7,
    'tag36h11': 10 / 8,
    'tagCustom48h12': 10 / 6,
    'tagStandard41h12': 9 / 5,
    'tagStandard52h13': 10 / 6
}


class CameraPoseDisplay:
    """
    A class whose objects display the camera's current position in 3D space relative to the AprilTags on the field in
    a Qt window.
    """
    def __init__(self, field: AprilTagField, **kwargs):
        """
        :param field: The field of AprilTags.
        :param kwargs: Keyword arguments to pass to the underlying plotter (see :class:`BackgroundPlotter`).
        """
        self.__plotter = BackgroundPlotter(**kwargs)
        for tag_id, tag_pose in field.items():
            texture = (pv.read_texture(str(files(tag_images).joinpath(f'{tag_id}.png')))
                       .flip_x()
                       .flip_y())
            plane_scale_factor = PLANE_SCALE_FACTORS.get(field.tag_family,
                                                         texture.dimensions[0] / (texture.dimensions[0] - 2))
            mesh: pv.PolyData = pv.Plane(i_size=field.tag_size * plane_scale_factor,
                                         j_size=field.tag_size * plane_scale_factor)
            mesh.point_data.clear()
            mesh.texture_map_to_plane(inplace=True)
            mesh.transform(tag_pose.matrix.astype(float))
            self.__plotter.add_mesh(mesh, texture=texture)
        mesh_path = str(files(resource).joinpath('camera.stl'))

        self.__original_camera_mesh: pv.DataSet = pv.read_meshio(mesh_path)
        self.__displayed_camera_mesh: pv.DataSet = self.__original_camera_mesh.copy(deep=True)
        self.__camera_mesh_actor = self.__plotter.add_mesh(self.__displayed_camera_mesh)
        self.__camera_mesh_actor.SetVisibility(False)

    @property
    def plotter(self) -> BackgroundPlotter:
        """The underlying PyVista plotter."""
        return self.__plotter

    @property
    def qt_application(self) -> Qt.QApplication:
        """The underlying Qt application."""
        return self.__plotter.app

    @property
    def qt_main_window(self) -> Qt.QMainWindow:
        """The main window of the Qt application."""
        return self.__plotter.main_window

    def exec_application(self) -> int:
        """
        Executes the Qt application.

        :return: A status code.
        """
        return self.qt_application.exec()

    def update(self, origin_in_camera: Optional[Transform] = None) -> None:
        """
        Updates the view in the camera display to reflect the new pose of the camera.

        :param origin_in_camera: The pose of the world origin to the camera frame (optional). If not specified, the
                                 plotter will be updated with no changes.
        """
        if origin_in_camera is not None:
            self.__camera_mesh_actor.SetVisibility(True)
            camera_in_origin = origin_in_camera.inv()
            self.__displayed_camera_mesh.deep_copy(
                self.__original_camera_mesh.transform(camera_in_origin.matrix.astype(float),  # type: ignore
                                                      inplace=False))
        self.__plotter.update()

    def close(self) -> None:
        """Closes the window displaying the camera pose."""
        self.__plotter.close()
