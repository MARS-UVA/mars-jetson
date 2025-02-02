## Serial Communication to Nucleo

### Setup
1. Install Python
2. `pip install -r requirements.txt`

### Usage
Run `sudo python send.py [motorCurrent1] [motorCurrent2] ...` up to a max of 7 motor currents (configurable).

### Details
Unspecified values get set to a default of 65 (A).\
Each motor current should be between 0 and 255, inclusive. \
A parity byte is sent as the 8th byte (the XOR of the 7 motor currents)
