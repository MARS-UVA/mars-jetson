import serial
import serial.tools.list_ports
#ports = [port.name for port in serial.tools.list_ports.comports()]
#print(ports)
s = serial.Serial("/dev/ttyTHS1", 115200, timeout = None)
numMotors = 6
bytesPerMotor = 1
totalDataBytes = numMotors * bytesPerMotor


def send(data : list[int], messageType : int = 0):
        s.write(bytes([messageType] + data))

if __name__ == "__main__":
        import sys
        header = 0 # 0 for motor commands
        data = [ord('A') for i in range(totalDataBytes)]
        for i in range(1,len(sys.argv)):
                data[i-1] = int(sys.argv[i])
        print(f"Data {data}")
        send(data)