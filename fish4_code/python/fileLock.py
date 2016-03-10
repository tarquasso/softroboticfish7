import os
import time

def lockFile(filename):
  if not os.path.exists(filename + '_LOCK'):
    touch = open(filename + '_LOCK', 'w+')
    touch.write('locked')
    touch.close()
    os.chmod(filename + '_LOCK', 111)
    return True
  lock = open(filename + '_LOCK', 'r')
  locked = len(lock.read()) > 0
  lock.close()
  if locked:
    return False
  lock = open(filename + '_LOCK', 'w')
  lock.write('locked')
  lock.close()
  return True
  

def releaseFile(filename):
  if not os.path.exists(filename + '_LOCK'):
    touch = open(filename + '_LOCK', 'w+')
    touch.write('')
    touch.close()
    os.chmod(filename + '_LOCK', 111)
  else:
    fout = open(filename + '_LOCK', 'w')
    fout.write('')
    fout.close()
  
def acquireLock(filename, waitTime = 0.05, timeout=None):
  if timeout is not None:
    timeout = timeout/waitTime;
  count = 0
  while not lockFile(filename):
    time.sleep(waitTime)
    count += 1
    if timeout is not None and count > timeout:
      return False
  return True
    
    
def releaseLock(filename):
  releaseFile(filename)
