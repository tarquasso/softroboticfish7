# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from evdev import InputDevice, categorize, ecodes, list_devices, KeyEvent
from time import time, sleep
import traceback

'''
305 A
306 B
304 X
307 Y
308 LB
310 LT
309 RB
311 RT
312 back
313 start

1.1 up
1.2 down
0.1 left
0.2 right
'''

class FishJoystick:
  # useLJ is whether to alter state using the left joystick (as opposed to just using the D-pad)
  def __init__(self, device=0, useLJ = True):
    self.dev = InputDevice(list_devices()[device])
    self._state = {}
    self._resetState()
    self._useLJ = useLJ
    
    self._keynames = {
      'BTN_B'    : 'A',
      'BTN_A'    : 'X',
      'BTN_X'    : 'Y',
      'BTN_C'    : 'B',
      'BTN_Z'    : 'RB',
      'BTN_TR'   : 'RT',
      'BTN_Y'    : 'LB',
      'BTN_TL'   : 'LT',
      'BTN_TL2'  : 'BACK',
      'BTN_TR2'  : 'START',
      'ABS_HAT0Y': 'DY',
      'ABS_HAT0X': 'DX',
      'ABS_Y'    : 'LJY',
      'ABS_X'    : 'LJX',
      'ABS_RZ'   : 'RJY',
      'ABS_Z'    : 'RJX'
      }

  def _resetState(self):
    self._state['start'] = 1
    self._state['pitch'] = 128
    self._state['yaw'] = 128
    self._state['frequency'] = 128
    self._state['thrust'] = 1
    
  def scan(self):
    try:
      for event in self.dev.read():
        keyname = None
        keyvalue = None
        if event.type == ecodes.EV_KEY:
          if isinstance(categorize(event).keycode, list):
            keyname = str(categorize(event).keycode[0])
          else:
            keyname = str(categorize(event).keycode)
          keyname = self._keynames[keyname]
          keyvalue = event.value
          keydown = (categorize(event).keystate == KeyEvent.key_down)
          #print 'KEY: %s (%s)' %  (keyname, ('down' if keydown else 'up'))
        elif event.type == ecodes.EV_ABS:
          keyname = str(categorize(event)).split(',')[-1].strip()
          keyname = self._keynames[keyname]
          keyvalue = event.value
          keydown = (keyvalue != 0)
          #print 'ABS: %s (%s)' % (keyname, str(event.value))
        if None not in [keyname, keyvalue]:
          # Thrust
          if keyname == 'LJY' and self._useLJ:
            self._state['thrust'] = max(255 - keyvalue, 1)
          elif keyname == 'DY' and keydown:
            self._state['thrust'] -= keyvalue*32
            self._state['thrust'] = max(self._state['thrust'], 1)
            self._state['thrust'] = min(self._state['thrust'], 255)
          # Frequency
          if keyname == 'LJX' and self._useLJ:
            self._state['frequency'] = min(keyvalue+1, 255)
          elif keyname == 'DX' and keydown:
            self._state['frequency'] += keyvalue*32
            self._state['frequency'] = max(self._state['frequency'], 1)
            self._state['frequency'] = min(self._state['frequency'], 255)
          # Pitch
          if keyname == 'RJY':
            self._state['pitch'] = max(255 - keyvalue, 1)
          elif keyname == 'Y' and keydown:
            self._state['pitch'] += 32
            self._state['pitch'] = max(self._state['pitch'], 1)
            self._state['pitch'] = min(self._state['pitch'], 255)
          elif keyname == 'A' and keydown:
            self._state['pitch'] -= 32
            self._state['pitch'] = max(self._state['pitch'], 1)
            self._state['pitch'] = min(self._state['pitch'], 255)
          # Yaw
          if keyname == 'RJX':
            self._state['yaw'] = min(keyvalue+1, 255)
          elif keyname == 'B' and keydown:
            self._state['yaw'] += 32
            self._state['yaw'] = max(self._state['yaw'], 1)
            self._state['yaw'] = min(self._state['yaw'], 255)
          elif keyname == 'X' and keydown:
            self._state['yaw'] -= 32
            self._state['yaw'] = max(self._state['yaw'], 1)
            self._state['yaw'] = min(self._state['yaw'], 255)
          # Select
          elif keyname == 'START':
            self._state['start'] = 255 if keydown else 1
          # Reset
          elif keyname == 'BACK':
            self._resetState()
    except IOError:
      pass
    return

  def getState(self):
    return self._state.copy()

  # resType can be 'bytearray' or 'list' or 'dict'
  def getStateBytes(self, resType='byteArray', nullTerminate=True):
    stateKeys = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
    if resType == 'dict':
      res = [(key, int(self._state[key])) for key in stateKeys]
      return dict(res)
    res = []
    res = [int(self._state[key]) for key in stateKeys]
    if nullTerminate:
      res.append(0)
    if resType == 'bytearray':
      res = bytearray(res)
    return res



if __name__ == "__main__":
 
  joystick = FishJoystick()
  
  lastSendTime = 0
  sendInterval = 0.5 # s
  
  while True:
    # check for joystick events
    joystick.scan()
    sleep(0.01)
    # periodically send the desired state
    if time() - lastSendTime > sendInterval:
      print joystick._state
      lastSendTime = time()










