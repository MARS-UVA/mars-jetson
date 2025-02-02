import serial
import serial.tools.list_ports
#ports = [port.name for port in serial.tools.list_ports.comports()]
#print(ports)
s = serial.Serial("/dev/ttyTHS1", 115200, timeout = None)
numMotors = 7
bytesPerMotor = 1
totalDataBytes = numMotors * bytesPerMotor
import sys

data = [0x00 for i in range(totalDataBytes)]
for i in range(1,len(sys.argv)):
        data[i-1] = int(sys.argv[i])
for i in range(len(sys.argv)-1, totalDataBytes):
        data[i] = ord('A')

parity = 0x00
for b in data:
        parity ^= b
print(f"Data {data}, parity {parity}")
s.write(bytes(data + [parity]))
#s.write(0b0010010001)