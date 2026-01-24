import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TIMER_PERIOD = 0.5 ## seconds



class LocalizationNode(Node):

    def __init__(self):

        super().__init__('localization_node')

        ## GLOBAL VARIABLES ##
        ## Types: nav_msgs/Odometry geometry_msgs/PoseWithCovarianceStamped geometry_msgs/TwistWithCovarianceStamped sensor_msgs/Imu
        ## These are the standard types taken by the ekf Filter, an abitrary amount can be taken.

        ## Able to process any data type delivered if it is not in the correct state.
        ## Expected inputs ZED SLAM, ZED IMU, AprilTag Positioning

        ## Output ZED
        self.processed_slam_data: PoseWithCovarianceStamped = PoseWithCovarianceStamped()

        ## Output Apriltag
        self.processed_apriltag_data: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        

        ## PUBLISHER FOR ZED_POSE## TEMPLATE
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'pose_with_covar', 10) ## VERIFY THE TOPIC
        timer_period = TIMER_PERIOD ## how often it will publish
        self.timer = self.create_timer(timer_period, self.zed_pose_callback) ## timer
        self.callback_counter = 0 ##callback counter

        ## SUBSCRIBER FOR SLAM ##
        self.subscription = self.create_subscription(
            PoseStamped, ## type for topic 'pose'
            'zed/zed_node/pose', ### topic name for the Zed2 Localizer
            self.slam_callback,
            10 ## ROS2 Docs told me to do it.
        )

        ## SUBSCRIBER FOR APRILTAG ##
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, ## Verify topic assignment
            '/localization/estimated_pose', ## Find topic name
            self.listener_callback,
            10
        )


    ## SLAM Subscriber Callback ##
    def slam_callback(self, msg: PoseStamped):
        slam_processed = PoseWithCovarianceStamped()
        slam_processed._pose = msg._pose
        slam_processed._header = msg._header
        self.processed_slam_data = slam_processed
        return
    
    ## SLAM Subscriber Callback ##
    def apriltag_callback(self, msg: PoseWithCovarianceStamped):
        apriltag_processed = msg
        self.processed_apriltag_data = apriltag_processed
        return

    
    ## Publisher Callback ##
    def zed_pose_callback(self):
        msg = self.processed_slam_data ## TBI; need to find out how this node is also receiving the information
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.callback_counter += 1
    



def main(args=None):
    rclpy.init(args=args)

    localization_publisher = LocalizationNode()

    rclpy.spin(localization_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()