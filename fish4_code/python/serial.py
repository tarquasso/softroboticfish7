#!/usr/bin/python
import serial, os, sys
'''
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.0)
while True:
    port.write("Hi I'm raspi!\n")
'''

class MbedSerial:
	def __init__(self, port, baud):
		self.comms=serial.Serial(port,baudrate=baud,timeout=None)
	
	#get single byte back from serial port and cast it to an int
	def getchar(self):
		return ord(self.comms.read(1))

	#get a \n terminated string back from port
	def getline(self):
		out=''
		byte=self.comms.read(1)
		while (byte!='\n'):
			out+=byte
			byte=self.comms.read(1)
		return out

	#get n bytes from serial port
	def getbytes(self, n):
		return self.comms.read(n)

	def __del__(self):
		self.comms.close()

if (__name__=="__main__"):
	mbed=MbedSerial("/dev/ttyAMA0", 9600)
	while 1:
		print mbed.getbytes(1)
