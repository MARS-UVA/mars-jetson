import serial
ser = serial.Serial("COM1")
ser.write(b'dog')
print(ser.read())