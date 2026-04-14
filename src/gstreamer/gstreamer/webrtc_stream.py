import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import threading
import asyncio
import json
import websockets
import sys
import gi

# --- GStreamer Imports ---
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib


STUN_SERVER = "stun://stun.l.google.com:19302"
FRAMERATE = 30

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc_node')

        # Declare Parameters
        self.declare_parameter('signaling_host', '')
        self.declare_parameter('signaling_port', 0)
        self.declare_parameter('video_topic', '/camera/image_raw')
        self.declare_parameter('bitrate', 1800000)
        self.declare_parameter('stream_height', 480)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('feed_active', True)

        # Get Parameter Values
        self.signaling_url = f'ws://{self.get_parameter("signaling_host").value}:{self.get_parameter("signaling_port").value}'
        self.video_topic = self.get_parameter('video_topic').value
        self.bitrate = self.get_parameter('bitrate').value
        self.stream_height = self.get_parameter('stream_height').value
        self.stream_width = self.get_parameter('stream_width').value
        self.feed_active = self.get_parameter('feed_active').value

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        
        # Initialize GStreamer
        Gst.init(None)
        self.bridge = CvBridge()
        self.conn = None
        self.loop = None
        self.appsrc = None

        # Start GLib Main Loop
        self.glib_loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.glib_loop.run)
        self.glib_thread.daemon = True
        self.glib_thread.start()
        
        # Setup Pipeline
        self.pipeline_desc = f"""
            appsrc name=ros_source format=time is-live=true do-timestamp=true 
            caps=video/x-raw,format=GRAY8,width={self.stream_width},height={self.stream_height},framerate={FRAMERATE}/1 ! 
            videoconvert ! queue max-size-buffers=1 leaky=downstream ! 
            vp8enc name=encoder deadline=1 keyframe-max-dist=30 target-bitrate={self.bitrate} ! 
            rtpvp8pay ! 
            application/x-rtp,media=video,encoding-name=VP8,payload=96 ! 
            webrtcbin name=sendrecv bundle-policy=max-bundle stun-server={STUN_SERVER}
        """
        
        try:
            self.pipe = Gst.parse_launch(self.pipeline_desc)
        except Exception as e:
            self.get_logger().error(f"FATAL: Pipeline parsing failed: {e}")
            sys.exit(1)

        self.webrtc = self.pipe.get_by_name('sendrecv')
        self.appsrc = self.pipe.get_by_name('ros_source')
        
        # Connect Signals
        self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)
        
        # Camera Subscriber
        self.qos_profile = qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.image_subscription = self.create_subscription(Image, self.video_topic, self.image_callback, self.qos_profile)
        
        # Start Signaling Thread
        self.thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f"WebRTC Node listening on {self.video_topic}...")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'video_topic':
                self.video_topic = param.value
                self.get_logger().info(f'Updated video topic: {self.video_topic}')
                self.destroy_subscription(self.image_subscription)
                self.image_subscription = self.create_subscription(
                    Image, self.video_topic, self.image_callback, self.qos_profile)
            elif param.name == 'feed_active':
                self.feed_active = param.value
                if self.feed_active:
                    self.pipe.set_state(Gst.State.PLAYING)
                else:
                    self.pipe.set_state(Gst.State.PAUSED)
            elif param.name == 'bitrate':
                self.bitrate = param.value
                encoder = self.pipe.get_by_name('encoder')
                if encoder:
                    encoder.set_property('target-bitrate', self.bitrate)
                    self.get_logger().info(f'Updated bitrate: {self.bitrate}')
                else:
                    self.get_logger().error('Encoder not found!')
            else:
                self.get_logger().warning(f'Tried to update {param.name}, but that does not update')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        if self.appsrc is None:
            return

        if self.feed_active:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                if cv_image.shape[1] != self.stream_width or cv_image.shape[0] != self.stream_height:
                    cv_image = cv2.resize(cv_image, (self.stream_width, self.stream_height))
                
                data = cv_image.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.duration = (1000000000 // FRAMERATE)
                
                self.appsrc.emit('push-buffer', buf)
                
            except Exception as e:
                self.get_logger().error(f"Frame error: {e}")

    def start_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_signaling())

    async def connect_signaling(self):
        while True:
            try:
                self.conn = await websockets.connect(self.signaling_url)
                self.get_logger().info("Connected to Signaling Server.")
                await self.conn.send(json.dumps({'cmd': 'HELLO_FROM_STREAMER'}))
                
                async for message in self.conn:
                    data = json.loads(message)
                    if 'cmd' in data and data['cmd'] == 'HELLO_FROM_VIEWER':
                        self.get_logger().info("Viewer detected. Starting Pipeline...")
                        self.start_pipeline()
                    elif 'sdp' in data:
                        self.handle_sdp(data['sdp'])
                    elif 'ice' in data:
                        self.handle_ice(data['ice'])
            except Exception as e:
                self.get_logger().warn(f"Signaling Error (Retrying in 2s): {e}")
                await asyncio.sleep(2)

    def start_pipeline(self):
        self.pipe.set_state(Gst.State.PLAYING)
        # Create Offer
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, self.webrtc, None)
        self.webrtc.emit('create-offer', None, promise)

    def on_offer_created(self, promise, _, __):
        promise.wait()
        reply = promise.get_reply()
        if not reply: return
        
        offer = reply.get_value('offer')
        if not offer: return

        # Send Promise
        promise = Gst.Promise.new_with_change_func(self.on_local_description_set, self.webrtc, None)
        self.webrtc.emit('set-local-description', offer, promise)
        
        # Send Offer
        msg = json.dumps({'sdp': {'type': 'offer', 'sdp': offer.sdp.as_text()}})
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.conn.send(msg), self.loop)

    def on_local_description_set(self, promise, _, __):
        promise.wait()
        self.get_logger().info("Local description set.")

    def on_negotiation_needed(self, element):
        pass 

    def on_ice_candidate(self, _, mlineindex, candidate):
        candidate_str = candidate
        self.get_logger().info(f"Sending ICE Candidate: {candidate_str}")
        
        msg = json.dumps({'ice': {'candidate': candidate_str, 'sdpMLineIndex': mlineindex}})
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.conn.send(msg), self.loop)

    def handle_sdp(self, sdp_data):
        if sdp_data['type'] == 'answer':
            self.get_logger().info("Received Answer. Setting Remote Description...")
            res, sdp_msg = GstSdp.SDPMessage.new()
            GstSdp.sdp_message_parse_buffer(bytes(sdp_data['sdp'].encode()), sdp_msg)
            answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdp_msg)

            promise = Gst.Promise.new_with_change_func(self.on_remote_description_set, self.webrtc, None)
            self.webrtc.emit('set-remote-description', answer, promise)

    def on_remote_description_set(self, promise, _, __):
        promise.wait()
        self.get_logger().info("Remote description set. Connection established!")

    def handle_ice(self, ice_data):
        self.webrtc.emit('add-ice-candidate', ice_data['sdpMLineIndex'], ice_data['candidate'])

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
