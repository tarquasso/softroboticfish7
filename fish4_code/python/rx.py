#!/bin/python
from __future__ import print_function

import numpy as np
from rtlsdr import RtlSdr, limit_calls
from codes import manchester, mycode, codes154, codes2corr
import matplotlib.pyplot as plt

import sys, select

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

def find_all(a_str, sub):
    start = 0
    while True:
        start = a_str.find(sub, start)
        if start == -1: return
        yield start
        start += len(sub) # use start += 1 to find overlapping matches

Mods = enum("MAGNITUDE", "PHASE", "DPHASE", "VECTOR")

HEADER = '0001101111101000010110' # a_h
FOOTER = '0001111' # x
PKTBYTES = 3
PKTLEN = 8*PKTBYTES + len(FOOTER)

class Demod:
  SAMP_RATE = 256000.
  SAMP_WINDOW = 1024*40

  def __init__(self):
    self.sdr = RtlSdr()
    # Sampling rate
    self.sdr.rs = Demod.SAMP_RATE
    # Pins 1 and 2
    self.sdr.set_direct_sampling(1)
    # I don't think this is used?
    self.sdr.gain = 1

  def run(self, limit=None, callback=lambda x: print(x),
                carrier=32000, bw=1000, sps=8, 
                codes=manchester, mod=Mods.MAGNITUDE, 
                header=HEADER, footer=FOOTER, pktlen=PKTBYTES):
    # Center frequency
    self.sdr.fc = carrier

    self.mod = mod
    self.header = header
    self.footer = footer
    self.pktlen = 8*pktlen + len(footer)

    self.rxcallback = callback

    decim = Demod.SAMP_RATE/bw/sps
    assert decim == int(decim)
    self.decim = int(decim)
    assert Demod.SAMP_WINDOW % self.decim == 0

    self.sampchips = Demod.SAMP_WINDOW / self.decim 
    self.corr = codes2corr(codes, sps)
    self.codelen = len(self.corr[0])

    self.last = np.zeros(Demod.SAMP_WINDOW)
    self.index = 0

    self.tocheck = [None] * self.codelen
    for i in range(self.codelen):
      self.tocheck[i] = dict()
      self.tocheck[i]['last'] = ''.join(range(0))
      self.tocheck[i]['pkts'] = range(0)

    if limit is None:
      def byte_callback(samp, sdr):
        if select.select([sys.stdin], [], [], 0)[0]:
          sdr.cancel_read_async()
        self.ddc(samp, sdr)
    else:
      @limit_calls(limit)
      def byte_callback(samp, sdr):
        if select.select([sys.stdin], [], [], 0)[0]:
          sdr.cancel_read_async()
        self.ddc(samp, sdr)

    self.sdr.read_bytes_async(byte_callback, Demod.SAMP_WINDOW*2)
    print (self.index, "samples read")
    sys.stdout.flush()
    sys.stdin.readline()

  def bb2c(self, baseband):
    mag = np.abs(baseband)
    phase = np.angle(baseband)
    dp = np.mod(np.ediff1d(phase)+np.pi, 2*np.pi)-np.pi
    return mag[1:], phase[1:], dp

  def decode(self, chips):
    corrs = []
    for c in self.corr:
      corrs.append(np.correlate(chips, c))
    # Vector correlations
    if np.iscomplex(corrs).any():
      corrs = np.abs(corrs)
    maxes = np.max(np.array(corrs), 0)
    codes = np.argmax(np.array(corrs), 0)
    return maxes, codes

  def debounce(self, i, l, rxstr):
    try:
      if i != self.debounce_i or abs(l - self.debounce_l) > 1:
        self.rxcallback(rxstr)
    except AttributeError:
      self.rxcallback(rxstr)
    self.debounce_i = i
    self.debounce_l = l

  def extract(self, nc):
    for codeoffset in range(self.codelen):
      pkts = []
      codestr = "".join(map(repr, map(int, nc[codeoffset::self.codelen])))

      for p in self.tocheck[codeoffset]['pkts']:
        pkt = p + codestr[0:self.pktlen-len(p)]
        if len(pkt) < self.pktlen:
          pkts.append(pkt)
        elif len(self.footer) == 0 or pkt[-len(self.footer):] == self.footer:
          str = ""
          for j in range(0,len(pkt)-1,8):
            str += chr(int(pkt[j:j+8][::-1], 2))
          self.debounce(self.index, -len(p), str)
          sys.stdout.flush()

      codestr = self.tocheck[codeoffset]['last'] + codestr
      for ind in find_all(codestr, self.header):
        pkt = codestr[ind+len(self.header):ind+len(self.header)+self.pktlen]
        if len(pkt) < self.pktlen:
          pkts.append(pkt)
        elif len(self.footer) == 0 or pkt[-len(self.footer):] == self.footer:
          str = ""
          for j in range(0,len(pkt)-1,8):
            str += chr(int(pkt[j:j+8][::-1], 2))
          self.debounce(self.index, ind, str)
          sys.stdout.flush()
      self.tocheck[codeoffset]['pkts'] = [] + pkts
      self.tocheck[codeoffset]['last'] = "" + codestr[-len(self.header)+1:]

  def ddc(self, samp, sdr):
    s = np.asarray(samp)
    i, q = s[::2], s[1::2]
    i = np.mean(i.reshape(-1,self.decim), 1) # poor man's decimation
    q = np.mean(q.reshape(-1,self.decim), 1) # poor man's decimation
    iq = np.empty(len(i), 'complex')
    iq.real, iq.imag = i, q
    iq /= (255/2)
    iq -= (1 + 1j)

    baseband = np.concatenate((self.last, iq))
    self.last = iq
    mag, phase, dp  = self.bb2c(baseband)
    if self.mod == Mods.MAGNITUDE:
      sig = mag
    elif self.mod == Mods.PHASE:
      sig = phase
    elif self.mod == Mods.DPHASE:
      sig = dp
    else:
      sig = baseband
    corrs, codes = self.decode(sig)

    nc = codes[self.codelen:self.codelen+self.sampchips]
    self.extract(nc)

    self.index += 1

  def end(self):
    self.sdr.close()

  def plot(self):
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(311)
    ax1.plot(self.chips)
    ax2 = fig.add_subplot(312, sharex=ax1)
    ax2.plot(self.corrs)
    ax3 = fig.add_subplot(313, sharex=ax1)
    ax3.plot(self.demod)
    plt.show()

  def checkdemod(self, index, demod=None, packetlen=5):
    if demod is None:
      demod = self.demod
    l = packetlen*8+6+7
    b = self.demod[index:index+l*self.codelen:self.codelen]
    return chipsToString(np.concatenate(([1,0], b, [0])))

  def findstring(self, demod = None, packetlen=5):
    if demod is None:
      demod = self.demod
    find = []
    for i in range(len(demod)):
      s, c = self.checkdemod(i, demod, packetlen)
      if len(s) and s[0] == 'a' and s[-1] == 'x':
        find.append((i, s[1:-1]))
    return find

def chipsToString(bits):
  char = 0
  rxstr = ""
  for i, b in enumerate(bits):
    char += (1 << (i%8)) if b else 0
    if i % 8 == 7:
      rxstr += chr(char)
      char = 0
  return rxstr, char

if __name__ == "__main__":
  d = Demod()
  #d.run(carrier=32000, bw=1000, sps=1, codes=manchester)
  d.run(carrier=32000, bw=1000, sps=1, mod=Mods.DPHASE, codes=mycode)
