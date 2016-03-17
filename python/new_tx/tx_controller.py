from time import time, sleep
from multiprocessing import Process, Value, Array, Manager
from FishJoystick import FishJoystick
import apa102 # LED strip

class AcousticJoystickController:

  def __init__(self):
    self._leds = apa102.APA102(11)
  
  def joystickReader(self, joystickState, joystickStateNum):
    scanInterval = 0.01 # seconds
    printInterval = 2   # seconds
    lastPrintTime = 0
    joystick = FishJoystick(joystick='snes')
    # read state and update joystickState
    while True:
      # get the latest joystick events
      joystick.scan()
      state = joystick.getStateBytes('dict')
      # convert the state to a state index
      stateNum = 0
      stateNum += state['start']
      stateNum += state['pitch']     << 1
      stateNum += state['yaw']       << 4
      stateNum += state['thrust']    << 7
      stateNum += state['frequency'] << 9
      # share state with other processes
      for (k, v) in state.iteritems():
        joystickState[k] = v
      joystickStateNum.value = stateNum
      # wait until the next scan time
      sleep(scanInterval)
      if time() - lastPrintTime > printInterval:
        print joystickState, '\t', joystickStateNum
        lastPrintTime = time()

  def joystickUpdater(self, joystickState, joystickStateNum, acousticStateNum, heartbeatValue):
    updateInterval = 0.01
    self._leds.clearStrip()

    def setValue(pixel, value):
      if value == 3:
        self._leds.setPixelRGB(pixel, 0xff0000)
      elif value == 2:
        self._leds.setPixelRGB(pixel, 0xff00)
      elif value == 1:
        self._leds.setPixelRGB(pixel, 0xff)
      else:
        self._leds.setPixel(pixel, 2, 2, 2)
        
    def setStateLED(stateName, value):
      if stateName == 'pitch':
        setValue(10, value-3)  # upper button
        setValue(8, 3-value)   # lower button
      if stateName == 'yaw':
        setValue(7, value-3)  # left button
        setValue(9, 3-value)  # right button
      if stateName == 'thrust':
        setValue(2, value) # upper button?
        setValue(4, value) # lower button?
      if stateName == 'frequency':
        setValue(3, value) # left button?
        setValue(5, value) # right button?
        
    while True:
      # Turn on LEDs to indicate joystickState
      state = joystickState.copy()
      for (stateName, value) in state.iteritems():
        setStateLED(stateName, value)
      # Turn on indicator LED according to acousticState == joystickState
      if acousticStateNum.value  == joystickStateNum.value:
        self._leds.setPixel(1, 0, 0, 15)
      else:
        self._leds.setPixel(1, 15, 0, 0)
      self._leds.setPixel(0, *heartbeatValue)
      self._leds.show()
      sleep(updateInterval)

  def acousticPlayer(self, joystickStateNum, acousticStateNum):
      interWordDelay = 200 # milliseconds
      import os
      while True:
        # get the appropriate wav file and play it
        stateNum = int(joystickStateNum.value)
        wavFile = '/home/pi/fish/python/wav_files_20bps/' + '{0:04d}'.format(stateNum) + '.wav'
        if stateNum != acousticStateNum.value:
          print 'playing new state:', stateNum
        if os.system("aplay -q " + wavFile) != 0:
          print '*** aplay command terminated'
        # indicate that we are now playing this state
        acousticStateNum.value = stateNum
        # wait until we should play the next word
        sleep(interWordDelay/1000)

  def run(self):
    manager = Manager()
    joystickState = manager.dict()
    joystickStateNum = Value('d', 0.0)
    acousticStateNum = Value('d', 0.0)
    heartbeatValue = Array('i', [0]*3)

    acousticProcess = Process(target=self.acousticPlayer, args=(joystickStateNum, acousticStateNum))
    joystickReaderProcess = Process(target=self.joystickReader, args=(joystickState, joystickStateNum))
    joystickUpdaterProcess = Process(target=self.joystickUpdater, args=(joystickState, joystickStateNum, acousticStateNum, heartbeatValue))

    try:
      # Start the processes
      joystickReaderProcess.start()
      joystickUpdaterProcess.start()
      acousticProcess.start()

      # control heartbeat
      maxBrightness = 50
      increment = 1
      increasing = True
      brightness = 0
      while True:
        # If all processes are running, show blue/green heartbeat
        if acousticProcess.is_alive() and joystickReaderProcess.is_alive() and joystickUpdaterProcess.is_alive():
          heartbeatValue[0] = 0
          heartbeatValue[1] = maxBrightness-brightness
          heartbeatValue[2] = brightness
        # Otherwise show red/orange heartbeat
        else:
          heartbeatValue[0] = 25
          heartbeatValue[1] = 0
          heartbeatValue[2] = brightness/2
        # if joystick updater is running, then it will set the LEDs
        if not joystickUpdaterProcess.is_alive():
          self._leds.setPixel(0, *heartbeatValue)
          self._leds.show()
        
        brightness += increment * (1 if increasing else -1)
        if brightness > maxBrightness-2:
          increasing = False
        if brightness < 2:
          increasing = True
          
        sleep(0.05)
    except:
      pass

    # Something went wrong: kill the process
    acousticProcess.terminate()
    joystickReaderProcess.terminate()
    joystickUpdaterProcess.terminate()
    # show a red LED then exit
    self._leds.clearStrip()
    self._leds.setPixel(0, 25, 0, 0)
    self._leds.show()

if __name__ == '__main__':
  controller = AcousticJoystickController()
  controller.run()

      
    
