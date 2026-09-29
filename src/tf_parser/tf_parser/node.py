from rclpy.node import Node

class TfParserNode(Node):
    def __init__(self):
        super().__init__("tf_parser")