from inspect import Parameter
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from teleop_msgs.msg import SetMotor, MotorChanges
from teleop.motor_queries import wheel_speed_to_motor_queries
from teleop.control.base import WheelSpeeds
from teleop import motor_queries

from autonomy_msgs.action import AutonomousActions


class AutonomousActionServer(Node):
    """ Example Variable
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
    """

    # Dump Parameters
    dump_forward_magnitude_param_descriptor = ParameterDescriptor(
        name='dump_forward_magnitude',
        type=ParameterType.PARAMETER_INTEGER,
        description='The magnitude of the wheel\'s speeds when starting dumping',
        dynamic_typing=True
    )
    
    dump_raise_drums_magnitude_param_descriptor = ParameterDescriptor(
        name='dump_raise_drums_magnitude',
        type=ParameterType.PARAMETER_INTEGER,
        description='The magnitude the drums should be raised when dumping',
        dynamic_typing=True
    )
    dump_spin_drums_magnitude_param_descriptor = ParameterDescriptor(
        name='dump_spin_drums_magnitude',
        type=ParameterType.PARAMETER_INTEGER,
        description='The magnitude the drums should spin when dumping',
        dynamic_typing=True
    )
    # Dig Parameters
    drum_dig_lowering_time_param_descriptor = ParameterDescriptor(
        name='drum_dig_lowering_time',
        type=ParameterType.PARAMETER_INTEGER,
        description='The time (ms) the drum should be lowered before the drum lowering motor is told to stop.'
    )
    drum_dig_raising_time_param_descriptor = ParameterDescriptor(
        name='drum_dig_raising_time',
        type=ParameterType.PARAMETER_INTEGER,
        description='The time (ms) the drum should be lowered before the drum lowering motor is told to stop.'
    )
    dig_wheel_speed_param_descriptor = ParameterDescriptor(
        name='dig_wheel_speed',
        type=ParameterType.PARAMETER_DOUBLE,
        description='The speed of wheel speed when doing autonomous dig actions',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=1.0)],
        dynamic_typing=True,
    )
    dig_spin_drums_speed_param_descriptor = ParameterDescriptor(
        name='dig_spin_drums_speed',
        type=ParameterType.PARAMETER_DOUBLE,
        description='The speed the drums should spin while digging',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=1.0)],
        dynamic_typing=True,
    )
    dig_drum_arm_magnitude_param_descriptor = ParameterDescriptor(
        name='drum_arm_speed',
        type=ParameterType.PARAMETER_DOUBLE,
        description='The speed the drum should be raised and lowered.',
        floating_point_range=[FloatingPointRange(from_value=0.0,
                                                 to_value=1.0)],
        dynamic_typing=True,
    )
    dig_time_param_descriptor = ParameterDescriptor(
        name='dig_time',
        type=ParameterType.PARAMETER_INTEGER,
        description='Time in seconds that the robot should spend digging.'
    )

    def __init__(self):
        super().__init__('autonomous_action_server')
        self._action_server = ActionServer(
            self,
            AutonomousActions,
            'autonomous_actions',
            self.execute_callback)
        self.serial_publisher = self.create_publisher(
            msg_type=MotorChanges,
            topic='teleop',
            qos_profile=1
        )
        self.declare_parameter(self.dump_forward_magnitude_param_descriptor.name,
                               descriptor=self.dump_forward_magnitude_param_descriptor)
        self.dump_forward_wheel_speeds = WheelSpeeds(self.get_parameter(
            self.dump_forward_magnitude_param_descriptor.name).value,
            self.get_parameter(
            self.dump_forward_magnitude_param_descriptor.name).value)
        self.declare_parameter(self.dump_raise_drums_magnitude_param_descriptor.name,
                               descriptor=self.dump_raise_drums_magnitude_param_descriptor)
        self.declare_parameter(self.dump_spin_drums_magnitude_param_descriptor.name,
                               descriptor=self.dump_spin_drums_magnitude_param_descriptor)
        self.declare_parameter(self.drum_dig_lowering_time_param_descriptor.name,
                               descriptor=self.drum_dig_lowering_time_param_descriptor)
        self.declare_parameter(self.drum_dig_raising_time_param_descriptor.name,
                               descriptor=self.drum_dig_raising_time_param_descriptor)
        self.declare_parameter(self.dig_wheel_speed_param_descriptor.name,
                               descriptor=self.dig_wheel_speed_param_descriptor)
        self.declare_parameter(self.dig_spin_drums_speed_param_descriptor.name,
                               descriptor=self.dig_spin_drums_speed_param_descriptor)
        self.declare_parameter(self.dig_drum_arm_magnitude_param_descriptor.name,
                               descriptor=self.dig_drum_arm_magnitude_param_descriptor)
        self.declare_parameter(self.dig_time_param_descriptor.name,
                               descriptor=self.dig_time_param_descriptor)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        match goal_handle.request.index:
            case 0:
                # teleop
                return
            case 1:
                # Dig Autonomy
                drum_speed = self.dig_spin_drums_speed_param_descriptor.value
                actuator_speed = self.dig_drum_arm_magnitude_param_descriptor.value
                wheel_speed = self.dig_wheel_speed_param_descriptor.value
                drum_lowering_delay = self.drum_dig_lowering_time_param_descriptor.value
                dig_time = self.dig_time_param_descriptor.value
                # Stop all motors currently moving on the robot
                self.serial_publisher.publish(motor_queries.stop_motors())
                # Create msg to send initial state
                msg = MotorChanges(changes=[], adds=[])
                # Set Drums to start digging
                msg.adds.append(SetMotor(index=SetMotor.SPIN_FRONT_DRUM, velocity=drum_speed))
                msg.adds.append(SetMotor(index=SetMotor.SPIN_BACK_DRUM, velocity=drum_speed))
                # Start Lowering of Drums
                motor_queries.raise_arms(actuator_speed, True, True, msg)
                # Send initial msg to serial node
                self.serial_publisher.publish(msg)
                # Sleep while drums lower
                time.sleep(drum_lowering_delay)
                # Start Driving Forward
                msg = motor_queries.wheel_speed_to_motor_queries(wheel_speed)
                # Stop drum lowering
                motor_queries.raise_arms(127, True, True, msg)
                # Send message to drive and dig
                self.serial_publisher.publish(msg)
                # Sleep while digging and driving forward
                time.sleep(dig_time)
                # Stop Digging
                self.serial_publisher.publish(motor_queries.stop_motors())

                # if action = move+dig
                # set the 4 wheel motors and also the drums
                # spin drums
                # lower drums
                # sleep(some time)
                # stop lowering drum
                # start driving
                # sleep(some time)
                # stop everything
                # raise drums

                # motor command 5?
                return
            case 2:
                # Dump Autonomy
                
                # Drive forward
                wheel_speed_to_motor_queries(self.dump_forward_wheel_speeds)
                # slleep(some time)
                wheel_speed_to_motor_queries(WheelSpeeds(0,0))
                # Raise Drums (?)
                # Spin Drums to Dump
                # sleep(some time)                
                return

        """
                for i in range(1, goal_handle.request.order):
                    feedback_msg.partial_sequence.append(
                        feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
                    self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
                    goal_handle.publish_feedback(feedback_msg)
                    time.sleep(0.1)

                goal_handle.succeed()

                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result
        """
        goal_handle.succeed()

def main(args=None):
    rclpy.init(args=args)

    autonomous_action_server = AutonomousActionServer()

    rclpy.spin(autonomous_action_server)


if __name__ == '__main__':
    main()