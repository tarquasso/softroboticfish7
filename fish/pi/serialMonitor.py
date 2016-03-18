"""
Usage:
  run "sudo python serialMonitor print=1 file=out.txt"
  where the print and file arguments are optional
  while running:
    type p and then enter to toggle screen printing (will store in buffer though)
    press enter to print just next line of buffer when not streaming to screen
    type q and then enter to quit (or ctrl-c)
"""

import os
import sys
import serial
import select
import time

def run(argv):
    # Get the COM port of the mbed
    p = os.popen('ls /dev/ttyACM*', "r")
    port = p.readline().strip()

    print 'Connecting to ' + port
    ser = serial.Serial(port, 921600, timeout=1)
    #while ser.inWaiting() > 0:
    #    ser.read()
    print '\tconnected to', ser.name
    print 'reading serial data (ctrl+c or q-enter to end)'

    dataFile = ''
    dataPrint = ''
    printData = True
    filenameBase = ''
    filenameEnd = ''
    if not os.path.exists('/home/pi/fish/data'):
        os.mkdir('/home/pi/fish/data')
    outDir = '/home/pi/fish/data/' + str(time.time()).replace('.','_')
    os.mkdir(outDir)
    fout = None
    fileNum = 0
    # get options from command line
    for arg in argv:
        if 'print' in arg:
            printData = int((arg.split('=')[1]).strip()) == 1
        if 'file' in arg:
            filenameBase = arg.split('=')[1].strip()
            if '.' in filenameBase:
                filenameEnd = filenameBase[filenameBase.find('.'):]
                filenameBase = filenameBase[0:filenameBase.find('.')]
	    print 'opening file', os.path.join(outDir, filenameBase + '-' + str(fileNum) + filenameEnd)
            fout = open(os.path.join(outDir, filenameBase + '-' + str(fileNum) + filenameEnd), 'w+')

    # begin monitoring
    ser.flushInput()
    terminate = False
    fileWriteTime = time.time()
    try:
        while(not terminate):
            nextData = ser.read(size=1500)
            if printData:
                print nextData
            if fout is not None:
                fout.write(nextData)
                if time.time() - fileWriteTime > 30:
                    fileWriteTime = time.time()
                    fout.close()
                    fileNum += 1
                    print 'opening file', os.path.join(outDir, filenameBase + '-' + str(fileNum) + filenameEnd)
                    fout = open(os.path.join(outDir, filenameBase + '-' + str(fileNum) + filenameEnd), 'w+')
                pass
    except KeyboardInterrupt:
        pass
    finally:
        # print/write remainder
        try:
            #if printData:
            #    print dataPrint
            #if fout is not None:
            #    fout.write(dataFile)
            pass
        except:
            pass
        # clean up
        ser.close()
        if fout is not None:
            fout.close()

if __name__ == '__main__':
    run(sys.argv)
