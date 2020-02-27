import serial # see http://pyserial.readthedocs.org/en/latest/pyserial_api.html
from time import time, sleep
from FishJoystick import FishJoystick
import sys


class AutoController:
  def __init__(self, mbedPort='/dev/ttyAMA0', mbedBaud=115200, mbedUpdateInterval=1.25):
     self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)

  def getStateBytes(self):
     #states = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
     #states = [1, 3, 6, 3, 1]
     states = [1, 3, 6, 0, 0]
     return bytearray(states)

  def run(self):
     self._mbedSerial.flushInput()
     self._mbedSerial.flushOutput()

     while(True):
       state_bytes = self.getStateBytes()
       self._mbedSerial.write(state_bytes)
       self._mbedSerial.write(bytearray([8]))
       self._mbedSerial.flush()
       sleep(0.02)
     

class BluetoothJoystickController:
  # @param mbedPort [str] The serial port to use on the mbed
  # @param mbedBaud [int] The baud rate to use with the mbed
  # @param mbedUpdateInterval [float] Time in ms to wait between sending updated state to the mbed
  def __init__(self, mbedPort='/dev/ttyAMA0', mbedBaud=115200, mbedUpdateInterval=1.25 , useLJ = True):
    self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE) # timeout = None means blocking, timeout = 0 means non-blocking
    self._mbedUpdateInterval = mbedUpdateInterval
    self._joystick = FishJoystick(joystick='snes', useLJ=useLJ)

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
        #self._mbedSerial.write(bytearray([1, 1, 1, 1, 1, 0]))
        if stateBytes[-1] != 8:
          self._mbedSerial.write(bytearray([8]))
        self._mbedSerial.flush() # wait until everything is written
        lastSendTime = time()
      if time() - lastPrintTime > 1:
       print self._joystick.getStateBytes('dict')
       lastPrintTime = time()


if __name__ == '__main__':
  import sys
  
  #try:
  #  mbedUpdateInterval = float(sys.argv[1])
  #except:
  #  mbedUpdateInterval = 0.01
  
  controller = AutoController()
  print '\nStarting controller'
  print 'using update interval of ', 0.05, 's'
  controller.run()
  print '\nAll done!'
