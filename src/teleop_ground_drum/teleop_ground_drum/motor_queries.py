from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

ARM_LINEAR_INDEX = 0
ARM_TILT_INDEX = 1
DRUM_INDEX = 2

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between a minimum and maximum."""
    return max(min(value, max_value), min_value)

# Raises the arms of the robot
def raise_arms(velocity, linear: bool, tilt: bool, msg: Float64MultiArray) -> None:
    velocity = clamp(velocity, -2.0, 2.0)
    if linear:
        msg.data[ARM_LINEAR_INDEX] = velocity
    if tilt:
        msg.data[ARM_TILT_INDEX] = velocity

# Stops spinning the bucket drum(s) selected
def stop_drum_spin(arm_drum_control: Float64MultiArray) -> None:
    arm_drum_control.data[DRUM_INDEX] = 0.0

def max_drum_spin(arm_drum_control: Float64MultiArray, forward: bool) -> None:
    if forward: 
        v = 2.0
    else: 
        v = -2.0

    arm_drum_control.data[DRUM_INDEX] = v

# Increments speed of the bucket drum(s) selected
def increment_drum_spin(velocity_increment: float,  arm_drum_control: Float64MultiArray) -> None:
    arm_drum_control.data[DRUM_INDEX] = clamp(arm_drum_control.data[DRUM_INDEX] + velocity_increment, -1.0, 1.0)
