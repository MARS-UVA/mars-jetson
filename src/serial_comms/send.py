import serial
import serial.tools.list_ports
#ports = [port.name for port in serial.tools.list_ports.comports()]
#print(ports)

s = serial.Serial("/dev/ttyTHS1", 115200, timeout = None)
numMotors = 6
bytesPerMotor = 1
totalDataBytes = numMotors * bytesPerMotor

# array format: [tl wheel, bl wheel, tr, br, drum, actuator]
def send(header,data): #messageType can be anything
	global bytesPerMotor
	mnum = (1<<8*bytesPerMotor)-1 #make sure each send is within maxbyte
	assert 0 <= header <= mnum
	s.write(header.to_bytes(bytesPerMotor,byteorder="big"))

	for i in data: 
		assert 0 <= i <= mnum
		s.write(i.to_bytes(bytesPerMotor,byteorder="big"))

if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	data = [ord('A') for i in range(totalDataBytes)]
	for i in range(len(sys.argv)):
		data[i] = int(sys.argv[i])
	print(f"Data {data}")
	send(header,data)