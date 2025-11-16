import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

Gst.init(None)

class StreamController(Node):
    def __init__(self):
        super().__init__('opencv_gstreamer_stream_controller')
        self.bridge = CvBridge()

        # Streaming settings
        self.width = 640
        self.height = 480
        self.framerate = 30
        self.host = "127.0.0.1"
        self.port = 25000

        # Create GStreamer pipeline string for MJPEG over UDP
        self.gst_pipeline = (
            f"appsrc name=mysrc is-live=true block=true format=time ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! "
            f"jpegparse ! rtpjpegpay ! "
            f"udpsink host={self.host} port={self.port} sync=false"
        )

        # Launch pipeline
        self.pipeline = Gst.parse_launch(self.gst_pipeline)
        self.appsrc = self.pipeline.get_by_name("mysrc")
        self.pipeline.set_state(Gst.State.PLAYING)

        self.subscription = self.create_subscription(
            Image, 'webcam_image', self.image_callback, 10
        )

        self.frame_count = 0

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (self.width, self.height))

        # Encode to JPEG
        _, jpeg_frame = cv2.imencode('.jpg', frame)
        data = jpeg_frame.tobytes()

        # Create GStreamer buffer
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = Gst.SECOND // self.framerate
        buf.pts = self.frame_count * Gst.SECOND // self.framerate
        buf.dts = buf.pts
        self.frame_count += 1

        # Push buffer to pipeline
        self.appsrc.emit("push-buffer", buf)

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
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
