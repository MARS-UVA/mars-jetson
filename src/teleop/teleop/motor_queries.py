from sensor_msgs.msg import JointState
from .joint_config import FRONT_ARM_NAME, BACK_ARM_NAME, FRONT_DRUM_NAME, BACK_DRUM_NAME, JOINT_MAP

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between a minimum and maximum."""
    return max(min(value, max_value), min_value)

def create_joint_state_message() -> JointState:
    """Create a JointState message with initialized names and zero velocities."""
    msg = JointState()
    msg.name = list(JOINT_MAP.keys())
    msg.velocity = [0.0] * len(msg.name)
    return msg

def update_joint_velocity(joint_name: str, velocity: float, msg: JointState) -> None:
    """Update the velocity of a joint in a JointState message."""
    joint_index = JOINT_MAP.get(joint_name)
    if joint_index is not None:
        msg.velocity[joint_index] = velocity
    else:
        raise ValueError(f"Joint name '{joint_name}' not found in joint_map.")

# Raises the arms of the robot
def raise_arms(velocity, front_arm: bool, back_arm: bool, msg: JointState) -> None:
    velocity = clamp(velocity, -1.0, 1.0)
    if front_arm:
        update_joint_velocity(FRONT_ARM_NAME, velocity, msg)
    if back_arm:
        update_joint_velocity(BACK_ARM_NAME, velocity, msg)

# Stops spinning the bucket drum(s) selected
def stop_drum_spin(front_arm: bool, back_arm: bool, msg: JointState) -> None:
    if front_arm:
        update_joint_velocity(FRONT_DRUM_NAME, 0.0, msg)
    if back_arm:
        update_joint_velocity(BACK_DRUM_NAME, 0.0, msg)

# Increments speed of the bucket drum(s) selected
def increment_drum_spin(velocity_increment: float, front_arm: bool, back_arm: bool, msg: JointState) -> None:
    if front_arm:
        update_joint_velocity(FRONT_DRUM_NAME, clamp(msg.velocity[msg.name.index(FRONT_DRUM_NAME)] + velocity_increment, -1.0, 1.0), msg)
    if back_arm:
        update_joint_velocity(BACK_DRUM_NAME, clamp(msg.velocity[msg.name.index(BACK_DRUM_NAME)] + velocity_increment, -1.0, 1.0), msg)
