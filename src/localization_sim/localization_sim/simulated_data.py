import heapq
from pathlib import Path
import sys

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu

from .data import load_data


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

        self.__queue = load_data(data_path, logger=self.get_logger())

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
            self.__clock_publisher.publish(event.get_clock())
            if isinstance(event.message, Image):
                self.__image_publisher.publish(event.message)
            elif isinstance(event.message, Imu):
                self.__imu_publisher.publish(event.message)
            else:
                self.get_logger().info(f'Message type "{type(self).__qualname__}" not recognized')
        self.get_logger().info('Simulation finished.')


def main() -> None:
    rclpy.init(args=sys.argv)
    node = SimulatedDataNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
