import serial
import serial.tools.list_ports
#ports = [port.name for port in serial.tools.list_ports.comports()]
#print(ports)

s = serial.Serial("/dev/ttyTHS1", 115200, timeout = None)
numMotors = 6
bytesPerMotor = 1
totalDataBytes = numMotors * bytesPerMotor
gCommands = [0,0,0,0,0,0] # global: [tl wheel, bl wheel, tr, br, drum, actuator]

def send(header): #messageType can be anything
	global gCommands, bytesPerMotor
	mnum = (1<<8*bytesPerMotor)-1 #make sure each send is within maxbyte
	assert 0 <= header <= mnum
	s.write(header.to_bytes(bytesPerMotor,byteorder="big"))

	for i in gCommands: 
		assert 0 <= i <= mnum
		s.write(i.to_bytes(bytesPerMotor,byteorder="big"))

if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	for i in range(gCommands): gCommands[i] = 69
	for i in range(1,len(sys.argv)):
		gCommands[i-1] = int(sys.argv[i])
	print(f"Data {gCommands}")
	send(header)