import serial # see http://pyserial.readthedocs.org/en/latest/pyserial_api.html
#import fishcamera
from time import time, sleep
from FishJoystick import FishJoystick
#from camera import FishCamera
import cv2

CAM_OUTPUT_DIR="/home/pi/fish_recordings"
DISPLAY_IMAGES = False

class NaiveColorTargetTracker():
  def __init__(self, camera, target_color):
    self.target_color = target_color
    self.camera = camera

    # red stretches 2 bands in hsv
    # these values are for yellow, keeping the 2 bands for red in the future
    self.hsv_lower_lower = (14,55,55)
    self.hsv_lower_upper = (30,255,235)
    self.hsv_upper_lower = self.hsv_lower_lower
    self.hsv_upper_upper = self.hsv_lower_upper

    # amount to shrink image, maintains aspect ratio
    self.subsample_ratio = 0.25

    #cv2.namedWindow('Mask')

  def get_image_size(self):
    #image = self.camera.capture()
    success, image = self.camera.read()
    return image.shape
    
  def find_target(self):
    #image = self.camera.capture()
    success, image = self.camera.read()
    target_found, target_centroid = self.process_image(image)
    return (target_found, target_centroid)
    
  def process_image(self, image):
    """
    Processes a single image, looks for a circle, return the target centroid in the camera frame
    Reference: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    """
    target_found = False
    target_centroid = None

    img_small = cv2.resize(image, None, fx=self.subsample_ratio, fy=self.subsample_ratio, interpolation=cv2.INTER_LINEAR)
    img_small = cv2.GaussianBlur(img_small, (5,5), 0)
    hsv_small = cv2.cvtColor(img_small, cv2.COLOR_BGR2HSV)
    mask_l = cv2.inRange(hsv_small, self.hsv_lower_lower, self.hsv_lower_upper)
    mask_u = cv2.inRange(hsv_small, self.hsv_upper_lower, self.hsv_upper_upper)
    mask = cv2.bitwise_or(mask_l, mask_u)
    #mask = cv2.erode(mask, None, iterations=2)
    #mask = cv2.dilate(mask, None, iterations=2)

    #print(mask)
    cnts, cnt_hier = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) == 0:
      return (False, None)

    cnt = max(cnts, key=cv2.contourArea)
    ((x,y),radius) = cv2.minEnclosingCircle(cnt)

    if DISPLAY_IMAGES:
      mask = cv2.circle(mask,(int(x),int(y)), int(radius), (255,0,0))
      cv2.imshow('Mask',mask)
      cv2.waitKey(1)
    #print(img_small.shape)
    #print("%d, %d, %d"%(hsv_small[60,80,0],hsv_small[60,80,1],hsv_small[60,80,2]))
    #print(radius)
    if radius < 5:
      return (False, None)

    target_centroid = ((int(x/self.subsample_ratio),int(y/self.subsample_ratio)),int(radius/self.subsample_ratio))
    target_found = True
    return (target_found, target_centroid)

class FishMbed():
  def __init__(self, mbedPort='/dev/ttyAMA0', mbedBaud=115200, mbedUpdateInterval=1.25):
    self.cmd_arr_order = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
    
    self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
    self._mbedUpdateInterval = mbedUpdateInterval

    self.CMD_MAX = 6
    self.CMD_MIN = 0

  def writeCmdArray(self, cmd):
    bytecmds = self.safeCmdToBytes(cmd)
    self.writeBytes(bytecmds)
    
  def writeBytes(self, bytecmds):
    self._mbedSerial.write(bytecmds)
    if bytecmds[-1] != 8:
      self._mbedSerial.write(bytearray([8]))
    self._mbedSerial.flush()

  def safeCmdToBytes(self, cmd, cmdType='byteArray', nullTerminate=False):

    if cmdType == "byteArray":
      for i,val in enumerate(cmd):
        cmd[i] = max(min(cmd[i],self.CMD_MAX),self.CMD_MIN)
    elif cmdType == "dict":
      for k in self.self.cmd_arr_order:
        cmd[k] = max(min(cmd[k],self.CMD_MAX),self.CMD_MIN)

    return self.cmdToBytes(cmd, cmdType, nullTerminate)
    
  def cmdToBytes(self, cmd, cmdType='byteArray', nullTerminate=False):
    """
    Turns a fish mbed command to bytearray (for sending to mbed)
    """
    if cmdType == "dict":
      res = [cmd[cmd_key] for cmd_key in self.cmd_arr_order]
    else:
      res = cmd
      
    assert(len(res) == len(self.cmd_arr_order))
    if nullTerminate:
      res.append(8)
    return bytearray(res)

class FishStateController():
  def __init__(self, update_interval):
    """
    update_interval in seconds
    """
    self.states = ("INIT","SEARCH","ADJUST","FOLLOW")
    self.state = None
    self.state_init_time = None
    self.follow_timeout = 5

    #['start', 'pitch', 'yaw', 'thrust', 'frequency']
    #[pitch: 0-6, yaw:0-6, thrust: 0-3, frequency: 0-3]
    self.HARD_LEFT = [1,3,6,3,2]
    self.HARD_RIGHT = [1,3,0,3,2]
    self.SOFT_LEFT = [1,3,5,3,2]
    self.SOFT_RIGHT = [1,3,1,3,2]
    self.DO_NOTHING = [1,3,3,0,1]
    self.GO_FORWARD = [1,3,3,3,3]
    
    self.transitionTo("INIT")
    
    self.update_interval = update_interval

    self.mbed = FishMbed()
    self.camera = cv2.VideoCapture(0)
    #self.camera = FishCamera(CAM_OUTPUT_DIR)
    target_rgb = (255,0,0)
    self.tracker = NaiveColorTargetTracker(self.camera, target_rgb)
    self.image_size = self.tracker.get_image_size()
    
  def run(self):
    lastTime = time()
    
    while True:
      if time() - lastTime > self.update_interval:
        lastTime = time()
        self.runOnce()

  def transitionTo(self, state_name):
    self.state = state_name
    self.state_init_time = time()
    print(self.state)

  def runOnce(self):
    # Mbed command order: ['start', 'pitch', 'yaw', 'thrust', 'frequency']
    target_found, target_centroid = self.tracker.find_target()

    if self.state == "INIT":
      self.mbed.writeCmdArray(self.DO_NOTHING)
      self.transitionTo("SEARCH")

    elif self.state == "SEARCH":

      if target_found:
        self.transitionTo("ADJUST")
        return

      self.mbed.writeCmdArray(self.HARD_LEFT)

    elif self.state == "ADJUST":
      # higher yaw is right, lower is left
      #target_found, target_centroid = self.tracker.find_target()
      
      if target_found and target_centroid[0][0] >= self.image_size[1]*(2./3.):
        print("SOFT RIGHT: %d, %d"%(target_centroid[0][0], self.image_size[1]*(2./3.)))
        self.mbed.writeCmdArray(self.SOFT_RIGHT)
      elif target_found and target_centroid[0][0] <= self.image_size[1]*(1./3.):
        print("SOFT LEFT: %d, %d"%(target_centroid[0][0], self.image_size[1]*(1./3.)))
        self.mbed.writeCmdArray(self.SOFT_LEFT)
      elif target_found:
        self.transitionTo("FOLLOW")
      else:
        self.transitionTo("SEARCH")

    elif self.state == "FOLLOW":

      if time() - self.state_init_time > self.follow_timeout:
        self.transitionTo("SEARCH")
        return
      self.mbed.writeCmdArray(self.GO_FORWARD)
    
if __name__ == '__main__':
  import sys
  update_hz = 30
  controller = FishStateController(1./update_hz)
  print '\nStarting Fish State Controller'
  print 'using update interval of ', 1./update_hz, 's'
  controller.run()
  print '\nAll done!'
