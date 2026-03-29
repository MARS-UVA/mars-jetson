# Motor query message which sets a motor's velocity to the given value.

# Motor index
uint8 index
# Biased integer representing the motor's desired velocity (127 = 0%, 255 = 100%)
uint8 velocity 127

uint8 FRONT_LEFT_DRIVE_MOTOR=0
uint8 BACK_LEFT_DRIVE_MOTOR=1
uint8 FRONT_RIGHT_DRIVE_MOTOR=2
uint8 BACK_RIGHT_DRIVE_MOTOR=3

uint8 SPIN_FRONT_DRUM=4
uint8 ARM_FRONT_ACTUATOR=5

uint8 SPIN_BACK_DRUM=6
uint8 ARM_BACK_ACTUATOR=7