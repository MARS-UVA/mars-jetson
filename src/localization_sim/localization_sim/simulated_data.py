import heapq
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu

from .data import ImuInfo, load_data


IMU_INFO = ImuInfo(
    orientation_covariance=1e4 * np.array([[ 9.78701273e-08,  3.60676444e-08, -1.53351636e-09],
                                           [ 3.60676444e-08,  1.20398837e-07,  3.15144082e-08],
                                           [-1.53351636e-09,  3.15144082e-08,  8.97890539e-08]]),
    angular_velocity_covariance=1e4 * np.array([[ 2.09799021e-06,  1.07090523e-06, -2.31795283e-07],
                                                [ 1.07090523e-06,  4.52955669e-06,  4.06406967e-08],
                                                [-2.31795283e-07,  4.06406967e-08,  1.53872932e-06]]),
    linear_acceleration_covariance=1e4 * np.array([[ 9.26719210e-05, -9.91040347e-07,  2.48941930e-06],
                                                   [-9.91040347e-07,  6.14368090e-05,  1.21056790e-06],
                                                   [ 2.48941930e-06,  1.21056790e-06,  9.05855861e-05]])
)


class SimulatedDataNode(Node):
    """A ROS node which simulates data from a webcam and IMU.

    The node takes a single argument which is a path to a directory containing data recorded by Motion Recorder. This
    data is then processed into messages and will be published in chronological order.

    This node controls the clock via the /clock topic to maximize accuracy to the recorded data. This will cause
    time skipping.
    """

    test_dir_param_descriptor = ParameterDescriptor(
        name='test_dir',
        type=ParameterType.PARAMETER_STRING,
        description='The name of the directory which contains the test data which will be played back',
        read_only=True
    )

    def __init__(self, **kwargs):
        super().__init__('simulated_data', **kwargs)

        self.declare_parameter(name=self.test_dir_param_descriptor.name,
                               value='test1',
                               descriptor=self.test_dir_param_descriptor)

        data_path = (Path(get_package_share_directory('localization_sim'))
                          / 'sim_data'
                          / self.get_parameter(self.test_dir_param_descriptor.name).get_parameter_value().string_value)

        self.__queue = load_data(data_path, imu_info=IMU_INFO, logger=self.get_logger())

        self.__clock_publisher = self.create_publisher(msg_type=Clock,
                                                            topic='/clock',
                                                            qos_profile=10)
        self.__image_publisher = self.create_publisher(msg_type=Image,
                                                                  topic='webcam_image',
                                                                  qos_profile=10)
        self.__imu_publisher = self.create_publisher(msg_type=Imu,
                                                          topic='/localization/imu',
                                                          qos_profile=10)

        self.__clock_publisher.publish(Clock())

        self.run()

    def run(self) -> None:
        self.get_logger().info('Simulation started.')
        while self.__queue:
            event = heapq.heappop(self.__queue)
            self.get_logger().debug(f'Got event at time {event.timestamp}, setting clock to {event.get_clock()}')
            self.__clock_publisher.publish(event.get_clock())
            if isinstance(event.message, Image):
                self.__image_publisher.publish(event.message)
            elif isinstance(event.message, Imu):
                self.__imu_publisher.publish(event.message)
            else:
                self.get_logger().warning(f'Message type "{type(self).__qualname__}" not recognized')
            time.sleep(0.01)
        self.get_logger().info('Simulation finished.')


def main() -> None:
    rclpy.init(args=sys.argv)
    node = SimulatedDataNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
