## Serial Communication to Nucleo

### Setup
1. Install Python
2. `pip install -r requirements.txt`

### Usage
Run `sudo python send.py [motorCurrent1] [motorCurrent2] ...` up to a max of 7 motor currents (configurable).

### Details
Unspecified values get set to a default of 65 (A).\
Each motor current should be between 0 and 255, inclusive. \
A header of 0 (1 byte) is used to denote motor currents, included at the start of the message. \
The 6 motor commands follow.