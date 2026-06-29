from robot_control_msgs.msg import ArmDrumControl
from geometry_msgs.msg import Twist

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between a minimum and maximum."""
    return max(min(value, max_value), min_value)

# Raises the arms of the robot
def raise_arms(velocity, front_arm: bool, back_arm: bool, msg: ArmDrumControl) -> None:
    velocity = clamp(velocity, -1.0, 1.0)
    if front_arm:
        msg.front_arm_speed = velocity
    if back_arm:
        msg.back_arm_speed = velocity

# Stops spinning the bucket drum(s) selected
def stop_drum_spin(front_arm: bool, back_arm: bool, arm_drum_control: ArmDrumControl) -> None:
    if front_arm:
        arm_drum_control.front_drum_speed = 0.0
    if back_arm:
        arm_drum_control.back_drum_speed = 0.0

# Increments speed of the bucket drum(s) selected
def increment_drum_spin(velocity_increment: float, front_arm: bool, back_arm: bool, arm_drum_control: ArmDrumControl) -> None:
    if front_arm:
        arm_drum_control.front_drum_speed = clamp(arm_drum_control.front_drum_speed + velocity_increment, -1.0, 1.0)
    if back_arm:
        arm_drum_control.back_drum_speed = clamp(arm_drum_control.back_drum_speed + velocity_increment, -1.0, 1.0)
