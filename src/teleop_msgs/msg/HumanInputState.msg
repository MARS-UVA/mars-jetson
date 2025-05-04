GamepadState gamepad_state
#keeps trck of the drive mode we are in
uint8 drive_mode
#Booleans for advanved and emergency stopping
bool a_stop
bool e_stop
#different drive mode constants
uint8 DRIVEMODE_NORMAL=0
uint8 DRIVEMODE_REVERSE=1
uint8 DRIVEMODE_IDLE=2