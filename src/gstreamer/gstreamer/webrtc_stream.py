import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import threading
import asyncio
import json
import websockets
import time
import sys
import gi

# GStreamer imports
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp

# --- CONFIGURATION ---
SIGNALING_URL = "ws://localhost:8443"
STUN_SERVER = "stun://stun.l.google.com:19302"

# image topic name
TOPIC_NAME = "/camera/image_raw"

# output resolution
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FRAMERATE = 30

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc_node')
        
        # Initialize GStreamer
        Gst.init(None)
        self.bridge = CvBridge()
        self.conn = None
        self.loop = None
        self.appsrc = None

        # Set GStreamer pipeline
        self.pipeline_desc = f"""
            appsrc name=ros_source format=time is-live=true do-timestamp=true 
            caps=video/x-raw,format=BGR,width={IMAGE_WIDTH},height={IMAGE_HEIGHT},framerate={FRAMERATE}/1 ! 
            videoconvert ! queue ! 
            vp8enc deadline=1 keyframe-max-dist=30 target-bitrate=2000000 ! 
            rtpvp8pay ! 
            application/x-rtp,media=video,encoding-name=VP8,payload=96 ! 
            webrtcbin name=sendrecv bundle-policy=max-bundle stun-server={STUN_SERVER}
        """
        
        try:
            self.pipe = Gst.parse_launch(self.pipeline_desc)
        except Exception as e:
            self.get_logger().error(f"FATAL: Pipeline parsing failed: {e}")
            sys.exit(1)

        # Get handles to the important elements
        self.webrtc = self.pipe.get_by_name('sendrecv')
        self.appsrc = self.pipe.get_by_name('ros_source')
        
        # Connect GStreamer Signals
        self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)
        
        # Create ROS Subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Image, TOPIC_NAME, self.image_callback, qos_profile)
        
        # Start the AsyncIO Loop in a separate thread to handle Signaling
        self.thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f"WebRTC Node listening on {TOPIC_NAME}...")

    def image_callback(self, msg):
        """
        Takes a ROS Image message, converts to OpenCV, then pushes to GStreamer appsrc.
        """
        if self.appsrc is None:
            return

        try:
            # Convert from ROS msg to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize to match pipeline caps
            # (may remove if we just make caps the same as our webcam)
            if cv_image.shape[1] != IMAGE_WIDTH or cv_image.shape[0] != IMAGE_HEIGHT:
                cv_image = cv2.resize(cv_image, (IMAGE_WIDTH, IMAGE_HEIGHT))
            
            # Create GStreamer Buffer
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = (1000000000 // FRAMERATE)    # 1 second divided by the framerate (ns)
            
            # Push the GStreamer Buffer to the Pipeline
            self.appsrc.emit('push-buffer', buf)
            
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def start_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_signaling())

    async def connect_signaling(self):
        """
        Connects to the WebSocket Signaling Server.
        """
        while True:
            try:
                self.conn = await websockets.connect(SIGNALING_URL)
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
        # Set pipeline to PLAYING state
        self.pipe.set_state(Gst.State.PLAYING)
        
        # Create Offer
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, self.webrtc, None)
        self.webrtc.emit('create-offer', None, promise)

    def on_offer_created(self, promise, _, __):
        promise.wait()
        reply = promise.get_reply()
        if not reply:
            return
        
        offer = reply.get_value('offer')
        if offer is None:
            self.get_logger().error("Offer is None. Pipeline failed to negotiate.")
            return

        # Set Local Description
        promise = Gst.Promise.new()
        self.webrtc.emit('set-local-description', offer, promise)
        promise.interrupt()
        
        # Send Offer to Signaling Server
        msg = json.dumps({'sdp': {'type': 'offer', 'sdp': offer.sdp.as_text()}})
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.conn.send(msg), self.loop)

    def on_ice_candidate(self, _, mlineindex, candidate):
        msg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.conn.send(msg), self.loop)

    def handle_sdp(self, sdp_data):
        if sdp_data['type'] == 'answer':
            res, sdp_msg = GstSdp.SDPMessage.new()
            GstSdp.sdp_message_parse_buffer(bytes(sdp_data['sdp'].encode()), sdp_msg)
            answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdp_msg)
            promise = Gst.Promise.new()
            self.webrtc.emit('set-remote-description', answer, promise)
            promise.interrupt()

    def handle_ice(self, ice_data):
        self.webrtc.emit('add-ice-candidate', ice_data['sdpMLineIndex'], ice_data['candidate'])

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()