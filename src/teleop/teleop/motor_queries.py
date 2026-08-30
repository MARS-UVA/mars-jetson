from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

FRONT_ARM_INDEX = 0
BACK_ARM_INDEX = 1
FRONT_DRUM_INDEX = 2
BACK_DRUM_INDEX = 3

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between a minimum and maximum."""
    return max(min(value, max_value), min_value)

# Raises the arms of the robot
def raise_arms(velocity, front_arm: bool, back_arm: bool, msg: Float64MultiArray) -> None:
    velocity = clamp(velocity, -2.0, 2.0)
    if front_arm:
        msg.data[FRONT_ARM_INDEX] = velocity
    if back_arm:
        msg.data[BACK_ARM_INDEX] = velocity

# Stops spinning the bucket drum(s) selected
def stop_drum_spin(front_arm: bool, back_arm: bool, arm_drum_control: Float64MultiArray) -> None:
    if front_arm:
        arm_drum_control.data[FRONT_DRUM_INDEX] = 0.0
    if back_arm:
        arm_drum_control.data[BACK_DRUM_INDEX] = 0.0

def max_drum_spin(front_arm: bool, back_arm: bool, arm_drum_control: Float64MultiArray, forward: bool) -> None:
    if forward: 
        v = 2.0
    else: 
        v = 0.0

    if front_arm:
        arm_drum_control.data[FRONT_DRUM_INDEX] = v
    if back_arm:
        arm_drum_control.data[BACK_DRUM_INDEX] = v

# Increments speed of the bucket drum(s) selected
def increment_drum_spin(velocity_increment: float, front_arm: bool, back_arm: bool, arm_drum_control: Float64MultiArray) -> None:
    if front_arm:
        arm_drum_control.data[FRONT_DRUM_INDEX] = clamp(arm_drum_control.data[FRONT_DRUM_INDEX] + velocity_increment, -1.0, 1.0)
    if back_arm:
        arm_drum_control.data[BACK_DRUM_INDEX] = clamp(arm_drum_control.data[BACK_DRUM_INDEX] + velocity_increment, -1.0, 1.0)
