print "Loading modules (1)..."

from evjs import Joystick
from leds import LEDs
import fileLock
import os
import time

j = Joystick()
leds = LEDs(j)

delay = 0.25
count = 1
fishFileStatePending = 'fishStatePending.txt'
fishFileStateCurrent = 'fishStateCurrent.txt'
fishFileAlive = 'fishAlive.txt'
print 'seeing if files exist'
if not os.path.exists(fishFileStatePending):
  print '\tcreating pending'
  fout = open(fishFileStatePending, 'w+')
  fout.write('0')
  fout.close()
  os.chmod(fishFileStatePending, 111)
if not os.path.exists(fishFileStateCurrent):
  print '\tcreating current'
  fout = open(fishFileStateCurrent, 'w+')
  fout.write('0')
  fout.close()
  os.chmod(fishFileStateCurrent, 111)
print 'releasing locks'
fileLock.releaseLock(fishFileStatePending)
fileLock.releaseLock(fishFileStateCurrent)
fileLock.releaseLock(fishFileAlive)
time.sleep(0.1) # let master heartbeat have time to restart reading alive file

# Indicate to heartbeat program that we are running
print 'getting alive lock'
fileLock.acquireLock(fishFileAlive)
print 'writing count'
fout = open(fishFileAlive, 'w+')
fout.write(str(count))
fout.close()
print 'releasing lock'
fileLock.releaseLock(fishFileAlive)

leds.go(1, color=0xffaa00)
print "Loading modules (2)..."

#from rpitx import gettx, oqpsktx, ooktx2, ooktx
import time
import os, sys, select

try:
  # set highest priority
  os.nice(-20)
except OSError:
  # not running as root
  pass 

leds.go(1, color=0xffaa)
print "Initializing hardware..."

#tx = gettx(carrier=32000, bw=100, samp_rate=192000)
#tx = gettx(carrier=32000, bw=1000, samp_rate=192000, block=ooktx2)
#tx = gettx(carrier=32000, bw=500, samp_rate=192000, block=oqpsktx)

leds.go(1, color=0xff00)
print "Fish control started."

try:
  while True:
    fileLock.acquireLock(fishFileAlive)
    j.scan()
    #print j
    # Get state index and write it to file
    stateNum = j.toStateNum()
    fileLock.acquireLock(fishFileStatePending)
    fout = open(fishFileStatePending, 'w+')
    fout.write(str(stateNum))
    fout.close()
    fileLock.releaseLock(fishFileStatePending)
    # See if fish has received this state yet
    fileLock.acquireLock(fishFileStateCurrent)
    fin = open(fishFileStateCurrent, 'r')
    try:
      curState = int(fin.read())
    except:
      print '*** fishcontrol read bad currentState from file'
      curState = -1
    fin.close()
    fileLock.releaseLock(fishFileStateCurrent)
    if curState == stateNum:
      leds.strip.setPixel(1, 0, 0, 15)
    else:
      leds.strip.setPixel(1, 15, 0, 0)
    leds.strip.show()
    
    leds.go(count)
    #print "Running tx.send()"
    #tx.send('a_h' + chr(count & 0xff) + j.toString() + 'x')
    #tx.send(j.toString())
    count += 1

    # Update fish file to indicate liveness
    fout = open(fishFileAlive, 'w+')
    fout.write(str(count))
    fout.close()
    fileLock.releaseLock(fishFileAlive)
    
    time.sleep(delay)

    if count > 100000:
      count = 0
except KeyboardInterrupt:
  print 'user interrupted fish control'
  pass

print "Fish control ended."
leds.end()



    












  
