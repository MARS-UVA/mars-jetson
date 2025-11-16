import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class StreamController(Node):
    def __init__(self):
        super().__init__('opencv_gstreamer_stream_controller')
        self.bridge = CvBridge()

        # Adjustable streaming settings
        self.width = 640
        self.height = 480
        self.framerate = 30
        self.host = "127.0.0.1"
        self.port = 1010

        # Create GStreamer pipeline string for OpenCV
        self.gst_pipeline = (
            f"appsrc is-live=true format=time ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! "
            f"jpegparse ! rtpjpegpay ! "
            f"udpsink host={self.host} port={self.port} sync=false"
        )


        # OpenCV VideoWriter with GStreamer backend
        self.writer = cv2.VideoWriter(
            self.gst_pipeline, cv2.CAP_GSTREAMER, 0, self.framerate, (self.width, self.height)
        )
        if not self.writer.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline!")
            raise RuntimeError("GStreamer pipeline failed")

        # Subscribe to ROS image topic
        self.subscription = self.create_subscription(
            Image, 'webcam_image', self.image_callback, 10
        )

    def image_callback(self, msg):
        self.get_logger().info('got image')
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Resize if needed (optional)
        frame = cv2.resize(frame, (self.width, self.height))

        # Write frame to GStreamer pipeline
        self.writer.write(frame)

    def destroy_node(self):
        # Release the VideoWriter on shutdown
        self.writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StreamController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
