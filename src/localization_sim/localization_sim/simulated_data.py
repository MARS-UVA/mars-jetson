import argparse
import heapq
from pathlib import Path
import sys

import rclpy
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu

from .data import load_data


class SimulatedDataNode(rclpy.Node):
    """A ROS node which simulates data from a webcam and IMU.

    The node takes a single argument which is a path to a directory containing data recorded by Motion Recorder. This
    data is then processed into messages and will be published in chronological order.

    The node waits for a message from /simulation/start to start the simulation.

    This node controls the clock via the /clock topic to maximize accuracy to the recorded data. This will cause
    time skipping.
    """
    def __init__(self, data_path: str | Path, **kwargs):
        super().__init__('simulated_data', **kwargs)
        self.__queue = load_data(Path(data_path))

        self.__clock_publisher = self.create_publisher(msg_type=Clock,
                                                            topic='/clock',
                                                            qos_profile=10)
        self.__image_publisher = self.create_publisher(msg_type=Image,
                                                                  topic='webcam_image',
                                                                  qos_profile=10)
        self.__imu_publisher = self.create_publisher(msg_type=Imu,
                                                          topic='/localization/imu',
                                                          qos_profile=10)
        self.__start_subscriber = self.create_subscription(msg_type=Empty,
                                                                topic='/simulation/start',
                                                                callback=self.on_receive_start_signal,
                                                                qos_profile=10)
        self.__simulation_started = False

        self.__clock_publisher.publish(Clock())

        self.get_logger().info()

    def on_receive_start_signal(self, msg: Empty) -> None:
        if not self.__simulation_started:
            self.__simulation_started = True
            self.run()

    def run(self) -> None:
        self.get_logger().info('Simulation started.')
        while self.__queue:
            event = heapq.heappop(self.__queue)
            self.__clock_publisher.publish(event.get_clock())
            if isinstance(event.message, Image):
                self.__image_publisher.publish(event.message)
            elif isinstance(event.message, Imu):
                self.__imu_publisher.publish(event.message)
            else:
                self.get_logger().info(f'Message type "{type(self).__qualname__}" not recognized')
        self.get_logger().info('Simulation finished.')


def main() -> None:
    parser = argparse.ArgumentParser(description='Simulate data from a webcam and IMU')
    parser.add_argument('data_dir',
                        type=Path,
                        help='a directory containing three files video.mov, motion.json, and metadata.json from'
                             'a Motion Recorder recording')
    args = parser.parse_args()
    rclpy.init(sys.argv)
    node = SimulatedDataNode(data_path=args.data_dir)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
