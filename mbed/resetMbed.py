import RPi.GPIO as GPIO
import time
import sys

resetPin = 3 # BCM pin 3, actually pin 5

hold = False
for arg in sys.argv:
    if 'hold' in arg:
        hold = True

def reset():
    # set up the pin as output
    # will generate a warning about pull-up resistor but that's ok
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(resetPin, GPIO.OUT) # ignore the warning

    # reset the mbed
    GPIO.output(resetPin, GPIO.LOW)
    time.sleep(0.5)
    if not hold:
        GPIO.output(resetPin, GPIO.HIGH)

        # clean up
        GPIO.cleanup()

if __name__ == '__main__':
    reset()
