import rclpy
import sys

from tf_parser.node import TfParserNode

def main() -> None:
    rclpy.init(args=sys.argv)
    node = TfParserNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()