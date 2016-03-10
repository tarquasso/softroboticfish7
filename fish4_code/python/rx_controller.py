
import RPi.GPIO as GPIO
import time
import os
from camera import FishCamera

# Set up camera
outputDir = '/home/pi/fish/recordings'
if not os.path.exists(outputDir):
    os.mkdir(outputDir)
fishCam = FishCamera(outputDir=outputDir)
videoLength = 60 # seconds

# Set up low battery monitor
batteryPin = 5 # BCM pin 5, actually pin 29
GPIO.setmode(GPIO.BCM)
GPIO.setup(batteryPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
batteryDied = False

def lowBatteryCallback(channel):
    print 'LOW BATTERY DETECTED'
    batteryDied = True
    
#GPIO.add_event_detect(batteryPin, GPIO.FALLING, callback=lowBatteryCallback, bouncetime=500)

# Start new video every few minutes
try:
    fishCam.take_picture('_STARTING')
    while not batteryDied and GPIO.input(batteryPin):
        print 'start video...'
        fishCam.take_video()
        time.sleep(videoLength)
        fishCam.stop_video()
        fishCam.take_picture()
except:
    pass
finally:
    try:
        fishCam.stop_video();
    except:
        pass
    fishCam.take_picture('_STOPPING')
    try:
        fishCam.cleanup()
    except:
        pass
    try:
        GPIO.cleanup()
    except:
        pass
    os.system('sudo /home/pi/fish/shutdown.sh')
    os.system('sudo shutdown -h now')
    
    
