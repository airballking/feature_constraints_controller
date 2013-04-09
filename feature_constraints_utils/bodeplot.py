#!/usr/bin/python


import sys
from math import sqrt,pi,sin,cos,atan2

def iir(old_value, new_value, innovation):
  return old_value*(1.0 - innovation) + new_value*innovation

class TemporalFilter:
  def __init__(self, num_bands):
    self.state = [None]+[0.0]*(num_bands-1)
    self.num_bands = num_bands
    self.innovation = 1.0 / (2 ** num_bands)

  def reset(self):
    num_bands = len(self.state)
    self.state = [None]+[0.0]*(num_bands-1)

  def __call__(self, value):
    """ filters a sample """
    if self.state[0] == None:
      self.state[0] = value
    else:
      innovation = self.innovation
      rest = value
      for i in xrange(self.num_bands-1):
        self.state[i] = iir(self.state[i], rest, innovation)
        rest -= self.state[i]
        innovation *= 2
      self.state[self.num_bands-1] = rest
    return self.state


def frange(start, stop, step):
     return (start + step*i for i in range(int(round((stop - start) / step))))

def analyze(filter_bank, f_sample, f_min, T_settle=10, T_measure=3):
  T = 1.0 / f_sample
  f_max = f_sample / 2.0
  f_step = f_min
  freqs, gains, phases = [], [], []
  num_bands = len(filter_bank(0))

  for f in frange(f_min, f_max, f_step):
    sys.stderr.write('f = %f\r' % f)
    filter_bank.reset()
    a = [0.0]*num_bands
    b = [0.0]*num_bands
    settle_time = int(T_settle*f+1)/f
    measure_time = settle_time + int(T_measure*f+1)/f

    for t in frange(0, settle_time, T):
      signal = filter_bank(sin(f*t*2*pi))

    for t in frange(settle_time, measure_time, T):
      signal = filter_bank(sin(f*t*2*pi))
      cos_T = cos(f*t*2*pi)*T
      sin_T = sin(f*t*2*pi)*T
      for i in xrange(num_bands):
        a[i] += cos_T*signal[i]
        b[i] += sin_T*signal[i]

    T_int = measure_time - settle_time
    phase = [atan2(bi/T_int, ai/T_int) for ai,bi in zip(a,b)]
    gain  = [sqrt(ai*ai + bi*bi)/T_int for ai,bi in zip(a,b)]
    freqs.append(f) ; gains.append(gain) ; phases.append(phase)
  return freqs, gains, phases


def write_gnuplot(hfile, freqs, gains):
  for b in range(len(gains[0])):
    for f in range(len(freqs)):
      hfile.write(str(freqs[f])+' '+str(gains[f][b])+'\n')
    hfile.write('\n\n')

def write_gnuplots(prefix, freqs, gains, phases):
  f = open(prefix+'_gain', 'w')
  write_gnuplot(f, freqs, gains)
  f.close()

  f = open(prefix+'_phase', 'w')
  write_gnuplot(f, freqs, phases)
  f.close()


def main():
  filt = TemporalFilter(8)
  f,g,p = analyze(filt, 833, 0.1, 1.0, 0.3)
  write_gnuplots('/tmp/bode', f, g, p)

  import os
  os.system(""" gnuplot -persist -e  "plot '/tmp/bode_gain' index 0:7 with lines " """)

if __name__ == "__main__":
    main()
