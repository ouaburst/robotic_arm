import serial
ser = serial.Serial('COM4',19200)

import time
file = open('c:\Git_repos\samples.csv')
while 1:
	line = file.readline()
	if not line:
		break
		ser.write(line)
file.close