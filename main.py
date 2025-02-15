from importlib.resources import files

import cv2
from PyQt5 import Qt

from apriltag_pose_estimation.apriltag.render import OverlayWriter
from apriltag_pose_estimation.core import DEPSTECH_CAM_PARAMETERS, load_field
from apriltag_pose_estimation.localization import PoseEstimator
from apriltag_pose_estimation.localization.render import CameraPoseDisplay, resource
from apriltag_pose_estimation.localization.strategies import MultiTagPnPEstimationStrategy, \
    LowestAmbiguityEstimationStrategy


def main() -> None:
    def timer_callback() -> None:
        not_closed, frame = video_capture.read()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = estimator.estimate_pose(image)
        if result.estimated_pose is not None:
            display.update(result.estimated_pose)
        overlay_writer = OverlayWriter(frame,
                                       detections=result.detections,
                                       camera_params=estimator.camera_params,
                                       tag_size=estimator.field.tag_size)
        overlay_writer.overlay_square(show_corners=True)
        overlay_writer.overlay_label()
        cv2.imshow('camera', frame)

    with files(resource).joinpath('testfield.json').open(mode='r') as f:
        field = load_field(f)

    estimator = PoseEstimator(
        strategy=MultiTagPnPEstimationStrategy(fallback_strategy=LowestAmbiguityEstimationStrategy()),
        field=field,
        camera_params=DEPSTECH_CAM_PARAMETERS,
        nthreads=2,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.25
    )

    video_capture = cv2.VideoCapture(0)

    cv2.namedWindow('camera')

    display = CameraPoseDisplay(estimator.field)

    timer = Qt.QTimer(display.plotter.app_window)
    timer.setSingleShot(False)
    timer.setInterval(1000 // 24)
    # noinspection PyUnresolvedReferences
    timer.timeout.connect(timer_callback)
    timer.start()

    try:
        display.exec_application()
    finally:
        cv2.destroyWindow('camera')


if __name__ == '__main__':
    main()
