# To run this script on Jetson independently, you can do
# eg. python3 100 100 100 100 100 100

import serial
import serial.tools.list_ports
import time
#ports = [port.name for port in serial.tools.list_ports.comports()]
#print(ports)

#s = serial.Serial("/dev/ttyTHS1", 115200, timeout = None)
s = serial.Serial("/dev/ttyUSB0", 115200, timeout = None)
numMotors = 6
bytesPerMotor = 1
totalDataBytes = numMotors * bytesPerMotor

# array format: [tl wheel, bl wheel, tr, br, drum, actuator]
def send(header,data): #messageType can be anything
	global bytesPerMotor
	mnum = (1<<8*bytesPerMotor)-1 #make sure each send is within maxbyte
	assert 0 <= header <= mnum
	s.write(header.to_bytes(bytesPerMotor,byteorder="big"))
	s.write(bytes(data)) # write the data to serial port


if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	data = [ord('A') for i in range(totalDataBytes)] # populate a list with A's as default values
	for i in range(1, len(sys.argv)): # populate data with (up to 6) args passed into command line
		data[i-1] = int(sys.argv[i])
	print(f"Data {data}")
	while True:
		send(header,data)
		time.sleep(0.1)
		
