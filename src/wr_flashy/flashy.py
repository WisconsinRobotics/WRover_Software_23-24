import serial
import time

s = serial.Serial('/dev/ttyUSB0', 9600)

p = bytearray()
p.append(100)
p.append(0)
p.append(150)

s.write(p)