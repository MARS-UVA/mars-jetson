import multiprocessing as mp
import os
import queue
import warnings
from collections import deque
from collections.abc import Generator, Iterable
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from apriltag_pose_estimation import AprilTagPoseEstimator, AprilTagPoseEstimationStrategy, CameraParameters, Pose
from apriltag_pose_estimation.strategies import (HomographyOrthogonalIterationStrategy, PerspectiveNPointStrategy,
                                                 PnPMethod)
from apriltag_pose_estimation.render import OverlayWriter


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


LOGITECH_CAM_PARAMETERS = CameraParameters(fx=1394.6027293299926,
                                           fy=1394.6027293299926,
                                           cx=995.588675691456,
                                           cy=599.3212928484164,
                                           k1=0.11480806073904032,
                                           k2=-0.21946985653851792,
                                           p1=0.0012002116999769957,
                                           p2=0.008564577708855225,
                                           k3=0.11274677130853494)
"""Camera parameters for a Logitech C920 webcam."""


class Plotter:
    """A class which facilitates plotting using matplotlib's animation package."""
    def __init__(self, axis_length: float, fig: plt.Figure = None, ax: plt.Axes = None):
        """
        Initializes a new Plotter.
        :param axis_length: The length of the drawn axes in meters.
        :param fig: The matplotlib figure to use.
        :param ax: The matplotlib axes on which the AprilTag axes will be drawn.
        """
        self.__axis_length = axis_length
        if fig is None and ax is None:
            self.__fig = plt.gcf()
            self.__ax = self.__fig.gca()
        elif fig is None:
            self.__fig = ax.get_figure()
            self.__ax = ax
        else:
            self.__fig = fig
            self.__ax = ax if ax is not None else fig.gca()
        self.__used_artists: dict[int, Line3DCollection] = {}
        self.__hidden_artists: deque[Line3DCollection] = deque()

    def __call__(self, poses: dict[int, Pose]) -> Iterable[plt.Artist]:
        """
        Updates the matplotlib artists to reflect the given poses.
        :param poses: A list of the poses of all visible AprilTags.
        :return: An iterable of all the visible artists after the update.
        """
        initial_homogenous_points = np.array([
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [1, 1, 1, 1]
        ]) * self.__axis_length
        for tag_id in list(self.__used_artists.keys()):
            if tag_id not in poses:
                artist = self.__used_artists.pop(tag_id)
                artist.set_visible(False)
                self.__hidden_artists.appendleft(artist)
        for tag_id, pose in poses.items():
            points = (pose.get_matrix() @ initial_homogenous_points)[:3, :]
            segments = np.array([points[:, (0, i)].T for i in range(1, 4)])
            if tag_id in self.__used_artists:
                self.__used_artists[tag_id].set_segments(segments)
            elif self.__hidden_artists:
                artist = self.__hidden_artists.popleft()
                artist.set_segments(segments)
                artist.set_visible(True)
                self.__used_artists[tag_id] = artist
            else:
                self.__used_artists[tag_id] = self.__ax.add_collection3d(Line3DCollection(segments,
                                                                                          colors=['r', 'g', 'b']))
        return list(self.__used_artists.values())


def frames(poses_queue: mp.Queue) -> Generator[dict[int, Pose], None, None]:
    """
    A generator factory for poses from the given multiprocessing queue.
    :param poses_queue: The queue which yields poses as they come in. The sentinel value indicating end-of-transmission
                        should be None.
    :return: A generator which yields poses of all visible AprilTags as they come in.
    """
    while True:
        if (poses := poses_queue.get()) is None:
            break
        yield poses


def plotter_process(poses_queue: mp.Queue, frame_delay: int, axis_length: float) -> None:
    """
    Plots poses coming in from the given queue until the end-of-transmission is reached. The animation will replay once
    transmission ends.
    :param poses_queue: The queue which yields poses as they come in. The sentinel value indicating end-of-transmission
                        should be None.
    :param frame_delay: The amount of time in milliseconds between frames.
    :param axis_length: The length of the drawn axes in meters.
    :return:
    """
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    fig.subplots_adjust(left=-1, right=2, bottom=-1, top=2, wspace=0)
    ax.view_init(elev=-90, azim=90, roll=180)

    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 5)
    ax.set_aspect('equal')

    animation = anim.FuncAnimation(fig, Plotter(axis_length=axis_length, fig=fig, ax=ax), frames=frames(poses_queue),
                                   cache_frame_data=False, interval=frame_delay)
    plt.show(block=True)


def processor_process(poses_queue: mp.Queue,
                      frame_delay: int,
                      strategy: AprilTagPoseEstimationStrategy,
                      tag_size: float,
                      camera_params: CameraParameters,
                      capture_index: int = 0,
                      file: os.PathLike | None = None,
                      **detector_kwargs) -> None:
    """
    Receives poses from a video capture, displays them, and broadcasts them on the given queue.
    :param poses_queue: The queue on which poses will be written. If the queue is ever full, then the frame will be
                        skipped. Once the video capture is closed or upon a runtime error, None will be pushed as a
                        sentinel value.
    :param frame_delay: The amount of time in milliseconds between frames.
    :param strategy: The strategy to use for pose estimation.
    :param tag_size: The size of the AprilTags which will be detected, in meters.
    :param camera_params: The parameters of the camera used for the video capture.
    :param capture_index: The index of the video capture (default: 0, which is the default camera). Ignored if file is
                          passed.
    :param file: If specified, the video file at the given path will be used for the capture instead of a stream.
    """
    estimator = AprilTagPoseEstimator(strategy=strategy,
                                      tag_size=tag_size,
                                      camera_params=camera_params,
                                      **detector_kwargs)
    try:
        cv2.namedWindow('Capture')
        if file is not None:
            capture = cv2.VideoCapture(str(Path(file).absolute()))
        else:
            capture = cv2.VideoCapture(capture_index)
        not_closed = True
        try:
            while not_closed:
                not_closed, frame = capture.read()
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if estimates := estimator.estimate_tag_pose(image):
                    print(estimates[0].best_tag_pose.get_matrix())
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    overlay_writer = OverlayWriter(image, estimates, camera_params, tag_size=tag_size)
                    overlay_writer.overlay_cubes()
                cv2.imshow('Capture', image)
                key = cv2.waitKey(frame_delay)
                try:
                    poses_queue.put_nowait({estimate.tag_id: estimate.best_tag_pose for estimate in estimates})
                except queue.Full:
                    warnings.warn('Frame dropped when sending to plotter')
                if key == 27:
                    break
        finally:
            cv2.destroyWindow('Capture')
            capture.release()
    finally:
        poses_queue.put(None)


def main(frame_delay: int = 20) -> None:
    poses_queue = mp.Queue(maxsize=10)
    processes = [mp.Process(target=processor_process,
                            args=(poses_queue,
                                  frame_delay,
                                  PerspectiveNPointStrategy(PnPMethod.IPPE),
                                  0.150,
                                  DEPSTECH_CAM_PARAMETERS),
                            kwargs=dict(families='tagStandard41h12',
                                        nthreads=2,
                                        quad_sigma=0,
                                        refine_edges=1,
                                        decode_sharpening=0.25)),
                 mp.Process(target=plotter_process, args=(poses_queue, frame_delay, 0.075))]
    for process in processes:
        process.start()

    for process in processes:
        process.join()


if __name__ == '__main__':
    main()
