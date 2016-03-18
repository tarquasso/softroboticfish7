import sys
import os
import time
import resetMbed
import serialMonitor

# Program the mbed, restart it, launch a serial monitor to record streaming logs
def runMbedProgramWithLogging(argv):
    for arg in argv:
        if 'startup=1' in arg:
            time.sleep(10)
    # If a bin file was given as argument, program it onto the mbed
    remount = True
    for arg in argv:
        if '.bin' in arg:
            print 'Copying bin file...'
            #os.system('sudo rm /media/MBED/*.bin')
            #time.sleep(1)
            #os.system('sudo cp /home/pi/Downloads/*.bin /media/MBED')
            #time.sleep(1)
            os.system('sudo /home/pi/fish/mbed/programMbed.sh ' + arg)
        if 'remount' in arg:
            remount=int(arg.split('=')[1].strip())==1
    # Remount mbed
    if remount:
        os.system("sudo /home/pi/fish/mbed/remountMbed.sh")
    # Start mbed program and serial monitor
    print 'Resetting mbed and starting serial monitor'
    print ''
    resetMbed.reset()
    print '============'
    serialMonitor.run(argv)

if __name__ == '__main__':
    runMbedProgramWithLogging(sys.argv)
    
    
