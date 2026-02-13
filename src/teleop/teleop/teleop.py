import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, Duration, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from teleop_msgs.msg import HumanInputState, MotorChanges, SetMotor, GamepadState, AddMotor

from .control import DriveControlStrategy, ArcadeDrive, GamepadAxis
from .signal_processing import Deadband
from .motor_queries import wheel_speed_to_motor_queries, raise_arms, stop_motors, stop_drum_spin, increment_drum_spin#, bucket_drum_speed_cruise_control


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
        self._motor_publisher = self.create_publisher(
            msg_type=MotorChanges,
            topic='teleop',
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self.__add_parameter_event_handlers()
        self.timer = self.create_timer(2, self.__stopped_motors)
        self.cruise_control = False

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

    def __on_receive_human_input_state(self, human_input_state: HumanInputState) -> None:
        # self.get_logger().warn(f"Got message, dpad down: {human_input_state.gamepad_state.dd_pressed}")
        self.timer.reset()
        gamepad_state : GamepadState = human_input_state.gamepad_state
        wheel_speeds = self.__drive_control_strategy.get_wheel_speeds(human_input_state.gamepad_state) #spin wheels

        if gamepad_state.back_pressed:
            self.prev_gamepad_state = gamepad_state
            self.get_logger().info("SOFT STOP")
            self._motor_publisher.publish(stop_motors())
            return

        if not self.cruise_control: motor_msg = wheel_speed_to_motor_queries(wheel_speeds)
        elif self.cruise_control:   motor_msg = MotorChanges(changes = [], adds = [])
        
        # Set states for control of bucket drums
        if gamepad_state.y_pressed and not self.prev_gamepad_state.y_pressed:
            self.front_arm_control = True
            self.back_arm_control = True
            # stop_drum_spin(motor_msg)
        elif gamepad_state.x_pressed and not self.prev_gamepad_state.x_pressed:
            self.front_arm_control = True
            self.back_arm_control = False
            # stop_drum_spin(motor_msg)
        elif gamepad_state.b_pressed and not self.prev_gamepad_state.b_pressed:
            self.front_arm_control = False
            self.back_arm_control = True
            # stop_drump_spin(motor_msg)
        

        # Spin Bucket Drum(s)
        if gamepad_state.lb_pressed and not self.prev_gamepad_state.lb_pressed: #spin bucket drum backwards
            self.get_logger().info("bucket drum -15")
            increment_drum_spin(-15, self.front_arm_control, self.back_arm_control, motor_msg)
            
        elif gamepad_state.rb_pressed and  not self.prev_gamepad_state.rb_pressed: #spin bucket drum forward
            self.get_logger().info("bucket drum +15")
            increment_drum_spin(+15, self.front_arm_control, self.back_arm_control, motor_msg)

        # Stop Bucket Drum(s)
        if gamepad_state.a_pressed:
            stop_drum_spin(self.front_arm_control, self.back_arm_control, motor_msg)
        self.get_logger().info(f'Calculated: {wheel_speeds}')
        
        rightStickY = gamepad_state.right_stick.y
        # Raise and Lower Bucket Drum Arm(s)
        if rightStickY > 0.2:
            raise_arms(-15, self.front_arm_control, self.back_arm_control, motor_msg)     
        if rightStickY < -0.2:
            raise_arms(+15, self.front_arm_control, self.back_arm_control, motor_msg)
        
        if human_input_state.gamepad_state.start_pressed and not self.prev_gamepad_state.y_pressed:
            self.cruise_control = not self.cruise_control
            # self._motor_publisher.publish(stop_motors()) #this happens on the next tick anyway

        if not motor_msg.changes and not motor_msg.adds:
            self.emptyUpdatesSent += 1
            self.emptyUpdatesSent %= self.MAX_EMPTY_UPDATES

        if motor_msg.changes or motor_msg.adds or self.emptyUpdatesSent == 0:
            self._motor_publisher.publish(motor_msg)
        # self.get_logger().warn(f"Published to serial node")
        self.prev_gamepad_state = gamepad_state
    
    def __stopped_motors(self) -> None:
        no_wheel_speed_msg = stop_motors()
        self._motor_publisher.publish(no_wheel_speed_msg)

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
