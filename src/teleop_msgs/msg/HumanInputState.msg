GamepadState gamepad_state
#keeps trck of the drive mode we are in
uint8 drive_mode
#Booleans for advanved and emergency stopping
bool a_stop
bool e_stop
#different drive mode constants
uint8 DRIVEMODE_TELEOP=0
uint8 DRIVEMODE_AUTONOMOUS=1
uint8 DRIVEMODE_ADVANCED_TELEOP=2