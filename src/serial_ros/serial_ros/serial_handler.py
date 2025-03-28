import serial
from time import sleep
import struct

# To run this script on Jetson independently, you can do
# eg. python3 100 100 100 100 100 100

class SerialHandler:
	def __init__(self):
		try:
			self.SER = serial.Serial("/dev/ttyUSB0", 115200, timeout = None)
		except serial.SerialException as e:
			print(f"Error: Could not open or close serial port: {e}")

		self.numMotors = 6
		self.bytesPerMotor = 1
		self.totalDataBytes = self.numMotors * self.bytesPerMotor

	def setBytes(self, b):
		self.totalDataBytes = b
		
	def setPort(self,port,baud):
		self.SER.close()
		self.SER = serial.Serial(port,baud,timeout = None)

	# array format: [tl wheel, bl wheel, tr, br, drum, actuator]
	def send(self,header,data): #messageType can be anything
		# print("sending")
		mnum = (1<<8*self.bytesPerMotor)-1 #make sure each send is within maxbyte
		# assert 0 <= header <= mnum
		self.SER.write(header.to_bytes(self.bytesPerMotor,byteorder="big"))
		self.SER.write(bytes(data)) # write the data to serial port
	
	def read(self):
		while(self.SER.in_waiting < 1): hiausten = 1
		header = self.SER.read(1)[0]
		match header:
			case 1:
				#motor feedback
				feedback = [1,0,0,0,0,0] #fl, fr, bl, br, drum
				for i in range(1,6):
					while(self.SER.in_waiting < 6): hisurya = 1
					feedback[i] = struct.unpack("f", self.ser.read(4))[0] #floats
				return feedback
			case 2:
				#potentiometer idk
				pass
			case _:
				print("unrecognized header")


if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	totalDataBytes = 6
	data = [ord('A') for i in range(totalDataBytes)] # populate a list with A's as default values
	for i in range(1, len(sys.argv)): # populate data with (up to 6) args passed into command line
		data[i-1] = int(sys.argv[i])
	print(f"Data {data}")
	handler = SerialHandler()
	while True:
		handler.send(header,data)
		sleep(0.1)