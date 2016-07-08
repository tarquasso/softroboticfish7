from __future__ import print_function
import serial
import time

# TODO: EVERYTHING

inp = raw_input("Enter space separated: port testname\n")

input_list = inp.split(" ")
params = [i.strip() for i in input_list]
port = params[0]
testname = params[1]

ser = serial.Serial(port, 9600)
print(ser.name)

logfile = open("logs/" + testname, "w")
ser.flush()

while True:
    x = ser.readline()
    print(x, end="")
    logfile.write(x)
    # time.sleep(0.25)
    logfile.flush()
    time.sleep(0.25)
