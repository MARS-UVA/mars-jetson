from dataclasses import dataclass, field
import heapq
from itertools import count
import json
from pathlib import Path

import cv2
import numpy as np
import numpy.typing as npt
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.impl.rcutils_logger import RcutilsLogger
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from .simulation import Event


@dataclass(frozen=True)
class ImuInfo:
    orientation_covariance: npt.NDArray[np.float64] = field(default_factory=lambda: np.zeros((3, 3)))
    angular_velocity_covariance: npt.NDArray[np.float64] = field(default_factory=lambda: np.zeros((3, 3)))
    linear_acceleration_covariance: npt.NDArray[np.float64] = field(default_factory=lambda: np.zeros((3, 3)))


def load_data(data_path: Path,
              imu_info: ImuInfo = ImuInfo(),
              logger: RcutilsLogger | None = None) -> list[Event]:
    queue: list[Event] = []

    video_start_time, motion_start_time = load_metadata(data_path, logger=logger)
    _load_video(data_path, queue, video_start_time, logger=logger)
    _load_motion(data_path, queue, motion_start_time, imu_info, logger=logger)

    return queue


def _load_video(data_path: Path,
                queue: list[Event],
                start_time: float,
                logger: RcutilsLogger | None = None) -> None:
    bridge = CvBridge()

    video_path = data_path / 'video.mov'
    if logger is not None:
        logger.info(f'Fetching video from "{video_path}"...')
    capture = cv2.VideoCapture(str(video_path))
    not_closed = True
    rate = 1 / capture.get(cv2.CAP_PROP_FPS)
    if logger is not None:
        logger.info(f'Found {capture.get(cv2.CAP_PROP_FRAME_COUNT)} frames. Loading...')
    frame_counter = count()
    while not_closed:
        frame_count = next(frame_counter)
        current_time = start_time + (rate * frame_count)
        not_closed, frame = capture.read()
        if frame is None:
            continue
        image_msg = bridge.cv2_to_imgmsg(frame)
        heapq.heappush(queue,
                       Event(timestamp=current_time,
                             message=image_msg))


def _load_motion(data_path: Path,
                 queue: list[Event],
                 start_time: float,
                 imu_info: ImuInfo,
                 logger: RcutilsLogger | None = None) -> None:
    motion_path = data_path / 'motion.json'
    if logger is not None:
        logger.info(f'Fetching motion data from "{motion_path}"...')
    with motion_path.open(mode='r') as fp:
        motion_data_raw = json.load(fp)

    motion_start_time = motion_data_raw['motion_data'][0]['timestamp']
    for datapoint in motion_data_raw['motion_data']:
        heapq.heappush(queue,
                       Event(timestamp=start_time + (datapoint['timestamp'] - motion_start_time),
                             message=Imu(header=Header(frame_id='base_link'),
                                         orientation=Quaternion(**datapoint['orientation']),
                                         orientation_covariance=imu_info.orientation_covariance.reshape(-1),
                                         angular_velocity=Vector3(**datapoint['angular_velocity']),
                                         angular_velocity_covariance=imu_info.angular_velocity_covariance.reshape(-1),
                                         linear_acceleration=Vector3(**datapoint['linear_acceleration']),
                                         linear_acceleration_covariance=imu_info.linear_acceleration_covariance.reshape(-1))))


def load_metadata(data_path: Path, logger: RcutilsLogger | None = None) -> tuple[float, float]:
    metadata_path = data_path / 'metadata.json'
    if logger is not None:
        logger.info(f'Fetching metadata from "{metadata_path}"...')
    with metadata_path.open(mode='r') as fp:
        metadata = json.load(fp)
    video_start_time = metadata['video_start_date']
    motion_start_time = metadata['motion_start_date']
    if video_start_time < motion_start_time:
        return 0, motion_start_time - video_start_time
    elif motion_start_time < video_start_time:
        return video_start_time - motion_start_time, 0
    else:
        return 0, 0
