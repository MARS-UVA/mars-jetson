from dataclasses import dataclass
from json import load
from rclpy.time import Time
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_parser_msgs.msg import AprilTagPositions, AprilTagPosition, AbsolutePosition
from tf_parser.position_calculator import calculate_absolute_position
import tf2_ros

TAG_PATH = "/home/ws/src/tf_parser/apriltag_setups"
APRILTAG_SET = "arena_nasa"

@dataclass
class KnownAprilTag:
    tagname: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    
class TfParserNode(Node):
    def __init__(self):
        super().__init__('tf_parser')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_subscription(AprilTagDetectionArray, "/detections",self.get_relative_positions_of_seen_apriltags, 10)
        
        self.position_pub_ = self.create_publisher(AprilTagPositions, "/apriltag/positions", 10)
        self.absolute_position_pub_ = self.create_publisher(AbsolutePosition, "/apriltag/absolute_position", 10)
    
        with open(f"{TAG_PATH}/{APRILTAG_SET}.json", 'r') as f:
            data = load(f)

        self.known_tags = {}

        for tagname, p in data.items():
            self.known_tags[tagname] = KnownAprilTag( tagname, p["x"], p["y"], p["z"], p["qx"], p["qy"], p["qz"], p["qw"] )   
        
    def get_absolute_position_from_known(self, positions: AprilTagPositions):
        pub = calculate_absolute_position(self, positions)
        if pub != None:
            self.absolute_position_pub_.publish(pub)
        
    def get_relative_positions_of_seen_apriltags(self, msg):
        positions = AprilTagPositions()
        for detection in msg.detections:
            try:
                tagname = f'{detection.family}:{detection.id}'
                t = self.tf_buffer.lookup_transform(
                    'frame_assembly',   
                    tagname,  
                    Time() 
                )
                translation = t.transform.translation
                rotation = t.transform.rotation
                
                position = AprilTagPosition()
                
                position.tag_name = tagname
                position.x = translation.x
                position.y = translation.y
                position.z = translation.z
                position.qx = rotation.x
                position.qy = rotation.y
                position.qz = rotation.z
                position.qw = rotation.w
                
                positions.positions.append(position)
            except Exception:
                pass
        self.position_pub_.publish(positions)
        
        self.get_absolute_position_from_known(positions)
