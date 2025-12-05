# Motor query message which sets a motor's velocity to the given value.

# Motor index
uint8 index 4
# Biased integer representing the motor's desired velocity (127 = 0%, 255 = 100%)
int8 vel_increment 0


uint8 FRONT_LEFT_DRIVE_MOTOR=0
uint8 BACK_LEFT_DRIVE_MOTOR=1
uint8 FRONT_RIGHT_DRIVE_MOTOR=2
uint8 BACK_RIGHT_DRIVE_MOTOR=3

uint8 SPIN_LEFT_MOTOR=4
uint8 ARM_LEFT_ACTUATOR=5

uint8 SPIN_RIGHT_MOTOR=6
uint8 ARM_RIGHT_ACTUATOR=7