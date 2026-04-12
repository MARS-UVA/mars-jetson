import serial
from time import sleep
import struct
from collections import deque

# To run this script on Jetson independently, you can do
# eg. python3 100 100 100 100 100 100
START = 255 # start byte preceding every message
class SerialHandler:
	def __init__(self):
		try:
			self.SER = serial.Serial("/dev/ttyUSB0", 2000000, timeout = None)
		except serial.SerialException as e:
			print(f"Error: Could not open or close serial port: {e}")

		self.serial_buffer = deque()
		self.numMotors = 8
		self.bytesPerMotor = 1
		self.totalDataBytes = self.numMotors * self.bytesPerMotor
		self.currentHeader = 0

	def setBytes(self, b):
		self.totalDataBytes = b
		
	def setPort(self, port, baud):
		self.SER.close()
		self.SER = serial.Serial(port,baud,timeout = None)

	# array format: [tl wheel, bl wheel, tr, br, drum, actuator]
	def send(self, header, data, logger = None): #messageType can be anything
		# print("sending")
		mnum = (1<<8*self.bytesPerMotor)-1 #make sure each send is within maxbyte
		# assert 0 <= header <= mnum
		self.SER.write(bytes([START]))
		self.SER.write(header.to_bytes(self.bytesPerMotor,byteorder="big"))
		# logger.warn(f"Wrate {data}")
		self.SER.write(bytes(data)) # write the data to serial port
		#logger.warn(f"Wrote {data} to Nucleo")

	def readMsg(self, logger=None):
		logger.info(f'Serial bytes in waiting: {self.SER.in_waiting}')
		self.serial_buffer.extend(self.SER.read(self.SER.in_waiting))

		while (len(self.serial_buffer) >= 4 and not (self.serial_buffer[0] == 0xFF and self.serial_buffer[2] == 0x00 and self.serial_buffer[3] == 0x00)): # check first three bytes
			self.serial_buffer.popleft()
		
		if (len(self.serial_buffer) >= 4):
			header = self.serial_buffer[1]
		else:
			header = 0
	
		num_data_bytes = 0
		match header:
			case 0x00:
				logger.info("no data")
				return (0, [])
			case 0x01:
				num_data_bytes = 40 # fl, fr, bl, br, ldrum, rdrum, fa, ba, mbat, auxbat
			case 0x02:
				num_data_bytes = 24 # fl, bl, fr, br, fdrum, bdrum
			case 0x03:
				num_data_bytes = 8 # fa, ba
			case _:
				logger.info(f"unknown header: {header}")
				self.serial_buffer.popleft()
				return (0, [])
	
		if len(self.serial_buffer) < 4 + num_data_bytes: return (0, [])
		# consume the 4-byte header
		for _ in range(4):
			self.serial_buffer.popleft()

		# extract data bytes and unpack as floats
		data_bytes = bytes(self.serial_buffer.popleft() for _ in range(num_data_bytes))
		feedback = [f[0] for f in struct.iter_unpack("f", data_bytes)]
		return (header, feedback)
		
		# deprecated readMSG code:
		"""
		if(self.SER.in_waiting<40): return []
		elif(self.SER.in_waiting>80): self.SER.read((self.SER.in_waiting//40)*40)
		
		feedback = list(struct.iter_unpack("f",self.SER.read(36))) # tuple of: fl, fr, bl, br, ldrum, rdrum, la, ra, actuator height
		feedback = [i[0] for i in feedback]
		return (header, feedback)
		"""


if __name__ == "__main__":
	import sys
	header = 0 # 0 for motor commands
	totalDataBytes = 8
	data = [ord('A') for i in range(totalDataBytes)] # populate a list with A's as default values
	for i in range(1, len(sys.argv)): # populate data with (up to 8) args passed into command line
		data[i-1] = int(sys.argv[i])
	print(f"Data {data}")
	handler = SerialHandler()
	while True:
		handler.send(header,data)
		sleep(0.1)
