from rclpy.time import Time
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_parser_msgs.msg import AprilTagPositions, AprilTagPosition
import tf2_ros
class TfParserNode(Node):
    def __init__(self):
        super().__init__('tf_parser')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_subscription(AprilTagDetectionArray, "/detections",self.get_positions_for_seen_apriltags, 10)
        
        self.position_pub_ = self.create_publisher(AprilTagPositions, "/apriltag/positions", 10)
        
    
    def get_positions_for_seen_apriltags(self, msg):
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

