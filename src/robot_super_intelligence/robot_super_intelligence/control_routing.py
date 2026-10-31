"""Command routing, robot modes, and stop/teleop services for one ROS node."""

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from robot_control_msgs.msg import RobotState
from std_msgs.msg import Bool, Float64MultiArray, UInt8
from std_srvs.srv import SetBool
from topic_tools_interfaces.srv import MuxSelect

from .outcomes import Outcome


class ControlRouting:
    def __init__(self, node, is_busy, is_stopping, stop_run):
        self.node = node
        self.is_busy = is_busy
        self.is_stopping = is_stopping
        self.stop_run = stop_run
        self.estopped = True
        self.mode = RobotState.ESTOP
        self.safety_epoch = 0
        self.changing_mode = False
        self.services_group = ReentrantCallbackGroup()
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = node.create_publisher(RobotState, 'robot_state', latched)
        lock_topic = node.declare_parameter('stop_lock_topic', 'control/locks/stop').value
        self.lock_pub = node.create_publisher(Bool, lock_topic, latched)
        self.outputs_locked = True
        self.stop_future = None
        self.transition_sec = node.declare_parameter('source_transition_sec', 0.5).value
        self.arm_stop_topic = node.declare_parameter('arm_stop_topic', 'arm_drum_control/stop').value
        self.arm_topics = {source: node.declare_parameter(
            f'arm_topics.{source}', f'arm_drum_control/{source}').value
            for source in ('teleop', 'dig', 'dump')}
        stop_topic = node.declare_parameter('stop_velocity_topic', 'cmd_vel/stop').value
        self.stop_velocity_pub = node.create_publisher(Twist, stop_topic, 1)
        self.stop_arm_pub = node.create_publisher(Float64MultiArray, self.arm_stop_topic, 1)
        self.mux = node.create_client(MuxSelect, 'arm_drum_mux/select')
        self.estop_service = node.create_service(SetBool, 'robot_state/estop', self.on_estop, callback_group=self.services_group)
        self.teleop_service = node.create_service(SetBool, 'robot_state/teleop', self.on_teleop, callback_group=self.services_group)
        # Legacy mode inputs can request teleop/idle/stop, never start a routine.
        self.set_sub = node.create_subscription(UInt8, 'robot_state/set', self.on_mode, 1)
        self.toggle_sub = node.create_subscription(UInt8, 'robot_state/toggle', self.on_mode, 1)
        self.cancel_sub = node.create_subscription(UInt8, 'cancel_command', self.on_manual_cancel, 1)
        self.heartbeat_timer = node.create_timer(
            node.declare_parameter('heartbeat_period_sec', 0.1).value,
            self.heartbeat, clock=self.steady_clock)
        self.lock_pub.publish(Bool(data=True))
        self.publish_mode(RobotState.ESTOP)
        self.heartbeat()

    def heartbeat(self):
        self.lock_pub.publish(Bool(data=self.outputs_locked))
        if self.outputs_locked:
            self.stop_velocity_pub.publish(Twist())
            self.stop_arm_pub.publish(Float64MultiArray(data=[0.0] * 4))

    def publish_mode(self, mode):
        self.mode = mode
        self.state_pub.publish(RobotState(state=mode))

    def hold_outputs(self):
        self.outputs_locked = True
        self.heartbeat()
        if self.mux.service_is_ready():
            self.stop_future = self.mux.call_async(MuxSelect.Request(topic=self.arm_stop_topic))
        else:
            self.stop_future = None

    def select(self, source, on_complete):
        """Select a source, reporting None on success or an exception on failure."""
        done = False

        def finish(error=None):
            nonlocal done
            if not done:
                done = True
                on_complete(error)

        def interrupted():
            return self.estopped or (self.is_stopping() and source != 'teleop')

        def selected(future):
            try:
                if not future.result().success:
                    raise RuntimeError('Arm/drum mux rejected the requested input')
                if interrupted():
                    self.hold_outputs()
                    raise RuntimeError('Operation interrupted during source transition')
                self.outputs_locked = False
                self.heartbeat()
                finish()
            except Exception as error:
                finish(error)

        def request_source():
            try:
                if interrupted():
                    raise RuntimeError('Operation interrupted during source transition')
                topic = self.arm_topics.get(source, self.arm_stop_topic)
                self.mux.call_async(MuxSelect.Request(topic=topic)).add_done_callback(selected)
            except Exception as error:
                finish(error)

        def stopped(future):
            try:
                if not future.result().success:
                    raise RuntimeError('Arm/drum mux rejected the stop input')
                if not source:
                    finish()
                elif self.transition_sec <= 0:
                    request_source()
                else:
                    # Drain old velocity inputs without a polling loop or Future.
                    def elapsed():
                        timer.cancel()
                        self.node.destroy_timer(timer)
                        request_source()
                    timer = self.node.create_timer(
                        self.transition_sec, elapsed, clock=self.steady_clock)
            except Exception as error:
                finish(error)

        try:
            if not self.mux.service_is_ready():
                raise RuntimeError('Arm/drum mux unavailable')
            if source not in ('', 'teleop', 'navigation', 'dig', 'dump'):
                raise ValueError(f'Unknown operation source: {source}')
            self.hold_outputs()
            if self.stop_future is None:
                raise RuntimeError('Arm/drum mux unavailable')
            self.stop_future.add_done_callback(stopped)
        except Exception as error:
            finish(error)

    async def select_response(self, source=''):
        """Bridge callback routing to ROS service handlers that must return a response."""
        response = Future(executor=self.node.executor)
        self.select(source, response.set_result)
        error = await response
        if error is not None:
            raise error

    def lock_outputs(self):
        self.safety_epoch += 1
        self.hold_outputs()

    def engage_stop(self):
        self.lock_outputs()
        self.estopped = True
        self.publish_mode(RobotState.ESTOP)

    async def on_estop(self, request, response):
        if request.data:
            self.stop_run('ESTOP')
            if self.stop_future is None:
                response.message = 'Drive stopped, but arm/drum mux is unavailable'
                return response
            stopped = await self.stop_future
            if not stopped.success:
                response.message = 'Arm/drum mux rejected the stop input'
                return response
            response.success = True
            response.message = 'E-stop engaged; active work is being canceled'
            return response
        if self.is_busy() or self.changing_mode:
            response.message = 'Waiting for the active operation to stop'
            return response
        self.changing_mode = True
        epoch = self.safety_epoch
        try:
            await self.select_response()
            if epoch != self.safety_epoch:
                response.message = 'A newer stop request superseded this clear'
                return response
            self.estopped = False
            self.publish_mode(RobotState.IDLE)
            response.success = True
            response.message = 'E-stop cleared; robot is idle'
        except Exception as error:
            response.message = str(error)
        finally:
            self.changing_mode = False
        return response

    async def on_teleop(self, request, response):
        if self.estopped or self.is_busy() or self.changing_mode:
            response.message = 'Clear E-stop and finish the active operation first'
            return response
        self.changing_mode = True
        epoch = self.safety_epoch
        try:
            await self.select_response('teleop' if request.data else '')
            if epoch != self.safety_epoch:
                response.message = 'A newer stop request superseded this mode change'
                return response
            self.publish_mode(RobotState.TELEOP if request.data else RobotState.IDLE)
            response.success = True
        except Exception as error:
            self.engage_stop()
            response.message = str(error)
        finally:
            self.changing_mode = False
        return response

    def on_mode(self, message):
        if message.data == RobotState.ESTOP:
            self.stop_run('ESTOP')
        elif message.data in (RobotState.TELEOP, RobotState.IDLE):
            if self.is_busy():
                self.stop_run(Outcome.MODE_EXITED.value)
            else:
                self.node.executor.create_task(self.on_teleop,
                    SetBool.Request(data=message.data == RobotState.TELEOP), SetBool.Response())
        else:
            self.node.get_logger().warn('Start autonomous modes through /manage_robot, not mode topics')

    def on_manual_cancel(self, message):
        if message.data == 1:
            # Legacy topic acts as a stop; only the action cancel service can
            # produce the protocol's CANCELED status.
            self.stop_run(Outcome.MODE_EXITED.value)

