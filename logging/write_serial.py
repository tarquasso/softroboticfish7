import serial

inp = raw_input("Enter serial port: ")

ser = serial.Serial(inp)
print("")
while True:
    params = raw_input()
    ser.write(params + "\n")
    print("")
