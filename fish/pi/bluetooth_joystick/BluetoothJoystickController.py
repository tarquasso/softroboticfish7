import serial # see http://pyserial.readthedocs.org/en/latest/pyserial_api.html
from time import time, sleep
from FishJoystick import FishJoystick


class BluetoothJoystickController:
  # @param mbedPort [str] The serial port to use on the mbed
  # @param mbedBaud [int] The baud rate to use with the mbed
  # @param mbedUpdateInterval [float] Time in ms to wait between sending updated state to the mbed
  def __init__(self, mbedPort='/dev/ttyAMA0', mbedBaud=115200, mbedUpdateInterval=0.05, useLJ = True):
    self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=None) # timeout = None means blocking, timeout = 0 means non-blocking
    self._mbedUpdateInterval = mbedUpdateInterval
    self._joystick = FishJoystick(joystick='bluetooth', useLJ=useLJ)

  # The main loop to update state from joystick and send to mbed
  # @param runTime [flaot or None] The time to run for, or None for infinite loop
  def run(self, runTime=None):
    self._mbedSerial.flushInput()
    self._mbedSerial.flushOutput()
    lastSendTime = 0
    lastPrintTime = 0
    startTime = time()    
    while runTime is None or time() - startTime < runTime:
      self._joystick.scan()
      sleep(0.01) # not sure if this is needed, but seems nice to reduce polling load on device?
      if time() - lastSendTime > self._mbedUpdateInterval:
        stateBytes = self._joystick.getStateBytes('bytearray')
        self._mbedSerial.write(stateBytes)
        if stateBytes[-1] != 0:
          self._mbedSerial.write(bytearray([0]))
        self._mbedSerial.flush() # wait until everything is written
        lastSendTime = time()
      if time() - lastPrintTime > 1:
        print self._joystick.getStateBytes('dict')
        lastPrintTime = time()


if __name__ == '__main__':
  import sys
  
  try:
    mbedUpdateInterval = float(sys.argv[1])
  except:
    mbedUpdateInterval = 0.01
  controller = BluetoothJoystickController(mbedUpdateInterval = mbedUpdateInterval, useLJ = False)
  print '\nStarting controller'
  print 'using update interval of ', mbedUpdateInterval, 's'
  controller.run()
  print '\nAll done!'
