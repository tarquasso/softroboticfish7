
import RPi.GPIO as GPIO
import time
import os

# Set up low battery monitor
batteryPin = 5 # BCM pin 5, actually pin 29
GPIO.setmode(GPIO.BCM)
GPIO.setup(batteryPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
batteryDied = False

def lowBatteryCallback(channel):
    print 'LOW BATTERY DETECTED'
    batteryDied = True
    
#GPIO.add_event_detect(batteryPin, GPIO.FALLING, callback=lowBatteryCallback, bouncetime=500)

# Poll the battery
try:
    while not batteryDied and GPIO.input(batteryPin):
        time.sleep(10)
except:
    pass
finally:
    try:
        GPIO.cleanup()
    except:
        pass
    os.system('sudo /home/pi/fish/shutdown.sh')
    os.system('sudo shutdown -h now')
    
    
