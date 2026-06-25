import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from teleop_msgs.msg import HumanInputState, GamepadState

from .control import DriveControlStrategy, ArcadeDrive, GamepadAxis
from .signal_processing import Deadband
from .motor_queries import raise_arms, stop_drum_spin, increment_drum_spin
from geometry_msgs.msg import Twist
from control_msgs.msg import RobotState, ArmDrumControl, ArmControlMode

class TeleopNode(Node):
    """A ROS node which converts inputs from a human at the control station into motor current commands."""
    
    linear_axis_param_descriptor = ParameterDescriptor(
        name='linear_axis',
        type=ParameterType.PARAMETER_STRING,
        description='The axis of the gamepad\'s inputs which controls linear velocity.',
        additional_constraints=f'Must be one of {", ".join(axis.name.lower() for axis in GamepadAxis)}',
        read_only=True,
        dynamic_typing=True,
    )

    turn_axis_param_descriptor = ParameterDescriptor(
        name='turn_axis',
        type=ParameterType.PARAMETER_STRING,
        description='The axis of the gamepad\'s inputs which controls angular velocity.',
        additional_constraints=f'Must be one of {", ".join(axis.name.lower() for axis in GamepadAxis)}',
        read_only=True,
        dynamic_typing=True,
    )

    full_forward_magnitude_param_descriptor = ParameterDescriptor(
        name='full_forward_magnitude',
        type=ParameterType.PARAMETER_DOUBLE,
        description='The magnitude of both wheel\'s speeds when the user inputs '
                    'completely forward. This affects the amount of wheel speed which will '
                    'be devoted to turning. 0 indicates the robot can only spin in place, '
                    'and 1 indicates the robot can only move forward and backward.',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=1.0)],
        dynamic_typing=True,
    )

    shape_param_descriptor = ParameterDescriptor(
        name='shape',
        type=ParameterType.PARAMETER_DOUBLE,
        description='A parameter describing the shape of the curve that converts axis inputs into speeds. '
                    'The axis input is raised to this power (keeping the sign), so 1 is linear (default: 1).',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=float('inf'))]
    )

    deadband_param_descriptor = ParameterDescriptor(
        name='deadband',
        type=ParameterType.PARAMETER_DOUBLE,
        description='Minimum gamepad axis input below which the input is assumed to be 0 (default: 0.0).',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=1.0)]
    )

    def __init__(self, **kwargs):
        super().__init__('teleop', **kwargs)
        self.prev_gamepad_state : GamepadState = GamepadState()
        self.front_arm_control = True
        self.back_arm_control = True
        self.arms_raising = False
        self.MAX_EMPTY_UPDATES = 30
        self.emptyUpdatesSent = 0
        self.declare_parameter(self.linear_axis_param_descriptor.name,
                               descriptor=self.linear_axis_param_descriptor)
        self.declare_parameter(self.turn_axis_param_descriptor.name,
                               descriptor=self.turn_axis_param_descriptor)
        self.declare_parameter(self.full_forward_magnitude_param_descriptor.name,
                               descriptor=self.full_forward_magnitude_param_descriptor)
        self.declare_parameter(self.shape_param_descriptor.name,
                               value=1.0,
                               descriptor=self.shape_param_descriptor)
        self.declare_parameter(self.deadband_param_descriptor.name,
                               value=0.0,
                               descriptor=self.deadband_param_descriptor)
        linear_axis_value = (self.get_parameter(self.linear_axis_param_descriptor.name)
                                            .get_parameter_value()
                                            .string_value
                                            .upper())
        if not linear_axis_value:
            raise ValueError('linear_axis parameter is missing')
        turn_axis_value = (self.get_parameter(self.turn_axis_param_descriptor.name)
                                          .get_parameter_value()
                                          .string_value
                                          .upper())
        if not turn_axis_value:
            raise ValueError('turn_axis parameter is missing')
        full_forward_magnitude_value = (self.get_parameter(self.full_forward_magnitude_param_descriptor.name)
                                                         .get_parameter_value()
                                                         .double_value)
        if full_forward_magnitude_value <= 0:
            raise ValueError(f'full_forward_magnitude parameter is non-positive or missing (got {full_forward_magnitude_value})')
        elif full_forward_magnitude_value >= 1:
            raise ValueError(f'full_forward_magnitude parameter must be less than 1 (got {full_forward_magnitude_value})')

        self.__drive_control_strategy = ArcadeDrive(
            linear_axis=getattr(GamepadAxis, linear_axis_value),
            turn_axis=getattr(GamepadAxis, turn_axis_value),
            full_forward_magnitude=full_forward_magnitude_value,
            shape=self.get_parameter(self.shape_param_descriptor.name)
                      .get_parameter_value()
                      .double_value,
            deadband=Deadband(min_magnitude=self.get_parameter(self.deadband_param_descriptor.name)
                                                .get_parameter_value()
                                                .double_value)
        )
        self.__human_input_state_subscription = self.create_subscription(
            msg_type=HumanInputState,
            topic='human_input_state', 
            callback=self.__on_receive_human_input_state,
            qos_profile=10,
        )
        self._cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel/teleop',
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self._arm_drum_control_publisher = self.create_publisher(
            msg_type=ArmDrumControl,
            topic='arm_drum_control/teleop',
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self._arm_control_mode_publisher = self.create_publisher(
            msg_type=ArmControlMode,
            topic='arm_control_mode',
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self._robot_state_subscriber = self.create_subscription(
            msg_type=RobotState,
            topic='robot_state',
            callback=self.__on_robot_state,
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self.__add_parameter_event_handlers()
        self.timer = self.create_timer(2, self.__stopped_motors)
        self.cruise_control = False
        self.cmd_vel = Twist()
        self.arm_drum_control = ArmDrumControl()
        self.robot_state = RobotState()

        self.get_logger().info(f'linear axis: {self.__drive_control_strategy.linear_axis}')
        self.get_logger().info(f'turn axis: {self.__drive_control_strategy.turn_axis}')
        self.get_logger().info(f'full forward magnitude: {self.__drive_control_strategy.full_forward_magnitude}')
        self.get_logger().info(f'shape: {self.__drive_control_strategy.shape}')
        self.get_logger().info(f'deadband: {self.__drive_control_strategy.deadband.min_magnitude}')

    @property
    def drive_control_strategy(self) -> DriveControlStrategy:
        return self.__drive_control_strategy

    @drive_control_strategy.setter
    def drive_control_strategy(self, value: DriveControlStrategy) -> None:
        if not isinstance(value, DriveControlStrategy):
            raise ValueError('drive control strategy must be of type DriveControlStrategy')

        self.__drive_control_strategy = copy.copy(value)
    
    def __on_robot_state(self, robot_state: RobotState) -> None:
        self.robot_state = robot_state
        self.__stopped_motors()

    def __on_receive_human_input_state(self, human_input_state: HumanInputState) -> None:
        if self.robot_state.state != RobotState.TELEOP:
            self.get_logger().info("Robot is not in teleop mode, ignoring human input state.")
            return

        # self.get_logger().warn(f"Got message, dpad down: {human_input_state.gamepad_state.dd_pressed}")
        self.timer.reset()
        gamepad_state : GamepadState = human_input_state.gamepad_state

        if gamepad_state.back_pressed:
            self.prev_gamepad_state = gamepad_state
            self.get_logger().info("SOFT STOP")
            self.__stopped_motors()

            self.cruise_control = False
            self.arms_raising = False
            return

        if not self.cruise_control:
            self.cmd_vel = self.__drive_control_strategy.get_twist(human_input_state.gamepad_state) #spin wheels
        
        # Set states for control of bucket drums
        if gamepad_state.y_pressed and not self.prev_gamepad_state.y_pressed:
            self.front_arm_control = True
            self.back_arm_control = True
            stop_drum_spin(self.front_arm_control, self.back_arm_control, self.arm_drum_control)
        elif gamepad_state.x_pressed and not self.prev_gamepad_state.x_pressed:
            self.front_arm_control = True
            self.back_arm_control = False
            stop_drum_spin(self.front_arm_control, self.back_arm_control, self.arm_drum_control)
        elif gamepad_state.b_pressed and not self.prev_gamepad_state.b_pressed:
            self.front_arm_control = False
            self.back_arm_control = True
            stop_drum_spin(self.front_arm_control, self.back_arm_control, self.arm_drum_control)

        self._arm_control_mode_publisher.publish(ArmControlMode(front_arm_control = self.front_arm_control, back_arm_control = self.back_arm_control))

        # Spin Bucket Drum(s)
        if gamepad_state.lb_pressed and not self.prev_gamepad_state.lb_pressed: #spin bucket drum backwards
            self.get_logger().info("bucket drum -15")
            increment_drum_spin(-0.1, self.front_arm_control, self.back_arm_control, self.arm_drum_control)
            
        elif gamepad_state.rb_pressed and not self.prev_gamepad_state.rb_pressed: #spin bucket drum forward
            self.get_logger().info("bucket drum +15")
            increment_drum_spin(+0.1, self.front_arm_control, self.back_arm_control, self.arm_drum_control)

        if gamepad_state.dl_pressed and not self.prev_gamepad_state.dl_pressed:
            self.get_logger().info("bucket drum full throttle backwards")
            max_drum_spin(front_arm = self.front_arm_control, back_arm = self.back_arm_control, msg = motor_msg, forward = False)
        elif gamepad_state.dr_pressed and not self.prev_gamepad_state.dr_pressed:
            self.get_logger().info("bucket drum full throttle forward")
            max_drum_spin(front_arm = self.front_arm_control, back_arm = self.back_arm_control, msg = motor_msg, forward = True)
        # Stop Bucket Drum(s)
        if gamepad_state.a_pressed:
            stop_drum_spin(True, True, self.arm_drum_control)
        self.get_logger().info(f'Calculated: {wheel_speeds}')
        
        
        rightStickY = gamepad_state.right_stick.y


        if self.arms_raising:
            raise_arms(120, True, True, motor_msg)

        if gamepad_state.du_pressed and not self.prev_gamepad_state.du_pressed:
            self.arms_raising = not self.arms_raising
        
        # Raise Bucket Drum Arm(s)
        if not self.arms_raising:
            if rightStickY > 0.2:
                raise_arms(+1.0, self.front_arm_control, self.back_arm_control, self.arm_drum_control)
            elif rightStickY < -0.2:
                raise_arms(-1.0, self.front_arm_control, self.back_arm_control, self.arm_drum_control)
            else:
                raise_arms(0.0, self.front_arm_control, self.back_arm_control, self.arm_drum_control)
        else:
            if abs(rightStickY) > 0.2:
                self.arms_raising = False
                raise_arms(0.0, True, True, motor_msg)

        if human_input_state.gamepad_state.start_pressed and not self.prev_gamepad_state.start_pressed:
            self.cruise_control = not self.cruise_control

        self._cmd_vel_publisher.publish(self.cmd_vel)
        self.get_logger().info(f'Published: {self.cmd_vel.linear.x}, {self.cmd_vel.angular.z}')
        self._arm_drum_control_publisher.publish(self.arm_drum_control)

        self.prev_gamepad_state = gamepad_state
    
    def __stopped_motors(self) -> None:
        self._cmd_vel_publisher.publish(Twist())
        self._arm_drum_control_publisher.publish(ArmDrumControl())

    def __add_parameter_event_handlers(self) -> None:
        try:
            from rclpy.parameter_event_handler import ParameterEventHandler
        except ImportError:
            self.get_logger().warning('ParameterEventHandler requires ROS 2 Jazzy. Updates to mutable parameters on '
                                      'this node will have no effect.')
            return
        self.__parameter_event_handler = ParameterEventHandler(self)
        self.__full_forward_magnitude_change_handler = self.__parameter_event_handler.add_parameter_callback(
            parameter_name=self.full_forward_magnitude_param_descriptor.name,
            node_name=self.get_name(),
            callback=self.__on_full_forward_magnitude_changed
        )
        self.__shape_change_handler = self.__parameter_event_handler.add_parameter_callback(
            parameter_name=self.shape_param_descriptor.name,
            node_name=self.get_name(),
            callback=self.__on_shape_changed
        )
        self.__deadband_change_handler = self.__parameter_event_handler.add_parameter_callback(
            parameter_name=self.deadband_param_descriptor.name,
            node_name=self.get_name(),
            callback=self.__on_deadband_changed
        )

    def __on_full_forward_magnitude_changed(self, full_forward_magnitude: rclpy.parameter.Parameter) -> None:
        self.__drive_control_strategy.full_forward_magnitude = full_forward_magnitude.get_parameter_value().double_value

    def __on_shape_changed(self, shape: rclpy.parameter.Parameter) -> None:
        self.__drive_control_strategy.shape = shape.get_parameter_value().double_value

    def __on_deadband_changed(self, deadband: rclpy.parameter.Parameter) -> None:
        self.__drive_control_strategy.deadband.min_magnitude = deadband.get_parameter_value().double_value


def main() -> None:
    rclpy.init(args=sys.argv)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
