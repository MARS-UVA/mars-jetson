from inspect import Parameter
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


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
        type=Parameter.Type.PARAMETER_INT,
        description='The magnitude of the wheel\'s speeds when starting dumping',
        dynamic_typing=True
    )
    
    dump_raise_drums_magnitude_param_descriptor = ParameterDescriptor(
        name='dump_raise_drums_magnitude',
        type=Parameter.Type.PARAMETER_INT,
        description='The magnitude the drums should be raised when dumping',
        dynamic_typing=True
    )
    dump_spin_drums_magnitude_param_descriptor = ParameterDescriptor(
        name='dump_spin_drums_magnitude',
        type=Parameter.Type.PARAMETER_INT,
        description='The magnitude the drums should spin when dumping',
        dynamic_typing=True
    )
    # Dig Parameters
    drum_lowering_time_param_descriptor = ParameterDescriptor(
        name='drum_lowering_time',
        type=Parameter.Type.PARAMETER_INT,
        description='The time the drum should be lowered before the drum lowering motor is told to stop.'
    )

    # Shared Parameters
    drum_arm_magnitude = ParameterDescriptor(
        name='drum_arm_magnitude',
        type=Paramter.Type.PARAMETER_INT,
        description='The apeed the drum should be raised and lowered.'
    )

    sleep_time_param_descriptor = ParameterDescriptor(
        name='sleep_time',
        type=Parameter.Type.PARAMETER_INT,
        description='The time the r'
    )

    def __init__(self):
        super().__init__('autonomous_action_server')
        self._action_server = ActionServer(
            self,
            AutonomousActions,
            'autonomous_actions',
            self.execute_callback)
        self._publisher = self.create_publisher(
            msg_type=MotorChanges,
            topic='teleop',
            qos_profile=1
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        match goal_handle.request.index:
            case 0:
                # teleop
                return
            case 1:
                # Dig Autonomy
                
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
                # slleep(some time)
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