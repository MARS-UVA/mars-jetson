import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import threading
import asyncio
import json
import websockets
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

        # Get Parameter Values
        self.signaling_url = f'ws://{self.get_parameter("signaling_host").value}:{self.get_parameter("signaling_port").value}'
        self.video_topic = self.get_parameter('video_topic').value
        self.bitrate = self.get_parameter('bitrate').value
        self.stream_height = self.get_parameter('stream_height').value
        self.stream_width = self.get_parameter('stream_width').value
        
        # Initialize GStreamer
        Gst.init(None)
        self.bridge = CvBridge()
        self.conn = None
        self.loop = None
        self.appsrc = None
        self.pipe = None
        self.webrtc = None
        self.session_lock = threading.Lock()
        self.session_generation = 0
        self.session_conn = None
        self.offer_started = False
        self.offer_sent = False
        self.pending_local_ice = []
        self.remote_description_set = False
        self.pending_remote_ice = []

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
            vp8enc deadline=1 keyframe-max-dist=30 target-bitrate={self.bitrate} ! 
            rtpvp8pay ! 
            application/x-rtp,media=video,encoding-name=VP8,payload=96 ! 
            webrtcbin name=sendrecv bundle-policy=max-bundle stun-server={STUN_SERVER}
        """
        
        # Camera Subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.create_subscription(Image, self.video_topic, self.image_callback, qos_profile)
        
        # Start Signaling Thread
        self.thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f"WebRTC Node listening on {self.video_topic}...")

    def image_callback(self, msg):
        with self.session_lock:
            appsrc = self.appsrc
            generation = self.session_generation
        if appsrc is None:
            return

        self.get_logger().warn(f"Sending new image data!")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            if cv_image.shape[1] != self.stream_width or cv_image.shape[0] != self.stream_height:
                cv_image = cv2.resize(cv_image, (self.stream_width, self.stream_height))
            
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = (1000000000 // FRAMERATE)
            
            # Hold the lock through the push so teardown cannot race this frame.
            with self.session_lock:
                if generation == self.session_generation and appsrc is self.appsrc:
                    appsrc.emit('push-buffer', buf)
            
        except Exception as e:
            self.get_logger().error(f"Frame error: {e}")

    def start_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_signaling())

    async def connect_signaling(self):
        while True:
            conn = None
            try:
                async with websockets.connect(self.signaling_url) as conn:
                    self.conn = conn
                    self.get_logger().info("Connected to Signaling Server.")
                    await conn.send(json.dumps({'cmd': 'HELLO_FROM_STREAMER'}))

                    async for message in conn:
                        data = json.loads(message)
                        # Serialize session changes and incoming signaling on GLib.
                        GLib.idle_add(self.handle_signaling, conn, data)
            except Exception as e:
                self.get_logger().warn(f"Signaling Error (Retrying in 2s): {e}")
            finally:
                if self.conn is conn:
                    self.conn = None
                GLib.idle_add(self.on_signaling_closed, conn)
            await asyncio.sleep(2)

    def handle_signaling(self, conn, data):
        if conn is not self.conn:
            return False
        if data.get('cmd') == 'HELLO_FROM_VIEWER':
            self.get_logger().info("Viewer detected. Starting fresh pipeline...")
            self.start_pipeline(conn)
        elif conn is self.session_conn and self.webrtc is not None:
            if 'sdp' in data:
                self.handle_sdp(data['sdp'])
            elif 'ice' in data:
                self.handle_ice(data['ice'])
        return False

    def on_signaling_closed(self, conn):
        if conn is self.session_conn:
            self.stop_pipeline()
        return False

    def stop_pipeline(self):
        # Invalidate callbacks and detach appsrc before stopping streaming threads.
        with self.session_lock:
            self.session_generation += 1
            self.appsrc = None
            old_pipe = self.pipe
            self.pipe = None
            self.webrtc = None
            self.session_conn = None
        self.pending_local_ice = []
        self.pending_remote_ice = []
        self.offer_started = False
        self.offer_sent = False
        self.remote_description_set = False
        # Do not hold the frame lock while GStreamer waits for threads to stop.
        if old_pipe is not None:
            old_pipe.set_state(Gst.State.NULL)

    def start_pipeline(self, conn):
        # Called only on the GLib thread, including on every viewer refresh.
        self.stop_pipeline()
        try:
            pipe = Gst.parse_launch(self.pipeline_desc)
            webrtc = pipe.get_by_name('sendrecv')
            appsrc = pipe.get_by_name('ros_source')
            with self.session_lock:
                self.pipe = pipe
                self.webrtc = webrtc
                self.session_conn = conn
            session = (self.session_generation, webrtc, conn)
            webrtc.connect('on-negotiation-needed', self.on_negotiation_needed, session)
            webrtc.connect('on-ice-candidate', self.on_ice_candidate, session)
            if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Pipeline could not enter PLAYING")
            with self.session_lock:
                self.appsrc = appsrc
        except Exception as e:
            self.get_logger().error(f"Failed to start WebRTC session: {e}")
            self.stop_pipeline()

    def session_is_current(self, session):
        generation, webrtc, conn = session
        with self.session_lock:
            return (generation == self.session_generation
                    and webrtc is self.webrtc
                    and conn is self.session_conn
                    and conn is self.conn)

    def on_negotiation_needed(self, element, session):
        GLib.idle_add(self.create_offer, session)

    def create_offer(self, session):
        if not self.session_is_current(session) or self.offer_started:
            return False
        self.offer_started = True
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, session, None)
        session[1].emit('create-offer', None, promise)
        return False

    def on_offer_created(self, promise, session, _):
        GLib.idle_add(self.finish_offer, promise, session)

    def promise_succeeded(self, promise, operation):
        if promise.wait() != Gst.PromiseResult.REPLIED:
            self.get_logger().error(f"{operation}: promise did not reply")
            return False
        reply = promise.get_reply()
        if reply is not None and reply.has_field('error'):
            self.get_logger().error(f"{operation}: {reply.get_value('error')}")
            return False
        return True

    def finish_offer(self, promise, session):
        if not self.session_is_current(session):
            return False
        if not self.promise_succeeded(promise, "Create offer"):
            return False
        reply = promise.get_reply()
        offer = reply.get_value('offer') if reply is not None else None
        if offer is None:
            self.get_logger().error("Create offer returned no offer")
            return False
        message = {'sdp': {'type': 'offer', 'sdp': offer.sdp.as_text()}}
        local_promise = Gst.Promise.new_with_change_func(
            self.on_local_description_set, (session, message), None)
        session[1].emit('set-local-description', offer, local_promise)
        return False

    def on_local_description_set(self, promise, context, _):
        session, message = context
        GLib.idle_add(self.finish_local_description, promise, session, message)

    def finish_local_description(self, promise, session, message):
        if not self.session_is_current(session):
            return False
        if not self.promise_succeeded(promise, "Set local description"):
            return False
        self.get_logger().info("Local description set. Sending offer.")
        self.send_signaling(session, message)
        self.offer_sent = True
        for message in self.pending_local_ice:
            self.send_signaling(session, message)
        self.pending_local_ice = []
        return False

    def on_ice_candidate(self, element, mlineindex, candidate, session):
        GLib.idle_add(self.send_ice_candidate, session, mlineindex, candidate)

    def send_ice_candidate(self, session, mlineindex, candidate):
        if not self.session_is_current(session):
            return False
        message = {'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}}
        if self.offer_sent:
            self.send_signaling(session, message)
        else:
            self.pending_local_ice.append(message)
        return False

    def send_signaling(self, session, message):
        if not self.session_is_current(session) or self.loop is None:
            return
        coroutine = self.send_if_current(session, message)
        try:
            future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        except Exception as e:
            coroutine.close()
            self.get_logger().error(f"Could not schedule signaling send: {e}")
            return
        future.add_done_callback(self.on_signaling_sent)

    async def send_if_current(self, session, message):
        # Recheck when the asyncio loop actually runs this queued send. Use the
        # captured socket, never a replacement connection in self.conn.
        if self.session_is_current(session):
            await session[2].send(json.dumps(message))
            if 'ice' in message:
                self.get_logger().info(
                    f"Session {session[0]}: sent ICE candidate")

    def on_signaling_sent(self, future):
        if future.cancelled():
            return
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Signaling send failed: {e}")

    def handle_sdp(self, sdp_data):
        if sdp_data.get('type') != 'answer':
            return
        self.get_logger().info("Received Answer. Setting Remote Description...")
        res, sdp_msg = GstSdp.SDPMessage.new()
        if res != GstSdp.SDPResult.OK:
            self.get_logger().error("Could not allocate remote SDP")
            return
        res = GstSdp.sdp_message_parse_buffer(sdp_data['sdp'].encode(), sdp_msg)
        if res != GstSdp.SDPResult.OK:
            self.get_logger().error("Could not parse remote SDP")
            return
        answer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.ANSWER, sdp_msg)
        session = (self.session_generation, self.webrtc, self.session_conn)
        promise = Gst.Promise.new_with_change_func(
            self.on_remote_description_set, session, None)
        session[1].emit('set-remote-description', answer, promise)

    def on_remote_description_set(self, promise, session, _):
        GLib.idle_add(self.finish_remote_description, promise, session)

    def finish_remote_description(self, promise, session):
        if not self.session_is_current(session):
            return False
        if not self.promise_succeeded(promise, "Set remote description"):
            return False
        self.get_logger().info("Remote description set.")
        self.remote_description_set = True
        for ice in self.pending_remote_ice:
            self.handle_ice(ice)
        self.pending_remote_ice = []
        return False

    def handle_ice(self, ice_data):
        if not self.remote_description_set:
            self.pending_remote_ice.append(ice_data)
            return
        self.webrtc.emit('add-ice-candidate',
                         ice_data['sdpMLineIndex'], ice_data['candidate'])

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
