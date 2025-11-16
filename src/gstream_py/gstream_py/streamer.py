import rclpy
from rclpy.node import Node
from gstream_msgs.msg import CameraState
import subprocess
import signal

class StreamController(Node):
    def __init__(self):
        super().__init__('gstreamer_stream_controller')

        # Subscriber listens for start/stop commands
        self.subscription = self.create_subscription(
            CameraState,
            'camera/stream_control',
            self.stream_callback,
            10
        )

        self.pipeline_proc = None

    def stream_callback(self, msg: CameraState):
        if msg.camera1 and self.pipeline_proc is None:
            # Start GStreamer pipeline
            gst_cmd = [
                "gst-launch-1.0",
                "v4l2src", "device=/dev/video0",
                "!", "image/jpeg,width=640,height=480",
                "!", "jpegparse",
                "!", "rtpjpegpay",
                # "!", "udpsink", "host=127.0.0.1", "port=5000"
                # "!", "udpsink", "host=172.26.45.189", "port=5000"
                "!", "udpsink", "host=172.27.224.1", "port=1010"
            ]
            self.get_logger().info("Starting camera stream...")
            self.pipeline_proc = subprocess.Popen(gst_cmd, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))

        elif not msg.camera1 and self.pipeline_proc:
            # Stop GStreamer pipeline
            self.get_logger().info("Stopping camera stream...")
            self.pipeline_proc.terminate()
            self.pipeline_proc.wait()
            self.pipeline_proc = None

def main(args=None):
    rclpy.init(args=args)
    node = StreamController()
    rclpy.spin(node)
    # Ensure cleanup
    if node.pipeline_proc:
        node.pipeline_proc.terminate()
        node.pipeline_proc.wait()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
