#!/usr/bin/python

from math import *

def iir(old_value, new_value, innovation):
  return old_value*(1.0 - innovation) + new_value*innovation

class TemporalFilter:
  def __init__(self, num_bands):
    self.state = [None]+[0.0]*(num_bands-1)


  def process_sample(self, value):
    """ filters a sample """
    if self.state[0] == None:
      self.state[0] = value
    else:
      innovation = 0.0015
      rest = value
      for i in range(len(self.state)-1):
        self.state[i] = iir(self.state[i], rest, innovation)
        rest -= self.state[i]
        innovation *= 2
      self.state[len(self.state)-1] = rest


  def _get_bands(self):
    """ return the filtered data """
    return self.state

  bands = property(_get_bands)


#main

import yarp
yarp.Network.init()

basename = '/finger_filter'

port_hand_in = yarp.BufferedPortBottle()
port_hand_in.open(basename + '/hand_in')

port_filtered_torques = yarp.BufferedPortBottle()
port_filtered_torques.open(basename + '/filtered_torques')

port_hf = yarp.BufferedPortBottle()
port_hf.open(basename + '/hf')

port_touch = yarp.BufferedPortBottle()
port_touch.open(basename + '/touch')

port_touch_filtered = yarp.BufferedPortBottle()
port_touch_filtered.open(basename + '/touch_filtered')

filters = [None]*(3*4)

for i in range(len(filters)):
  filters[i] = TemporalFilter(8)

#template = [0.15, -0.20, 0.08,  0.0, 0.5, -0.9,  0.20, 0.05, 0.05,  0, 0, 0]
#template = [0.2, -0.70, 0.28,  0.0, -0.18, 0.0,  0.24, 0.12, 0.2,  0, 0, 0]
template = [0.2, -0.40, 0.15,  0.0, 0.3, -0.7,  0.20, 0.05, 0.05,  0, 0, 0]
template_l2 = sum([v*v for v in template])
filtered = [0.0]*(3*4)
jumps = [0.0]*(3*4)

last_hf = yarp.Time_now()
last_val_filtered = yarp.Time_now()

while True:
  bottle = port_hand_in.read(True)
  values = [bottle.get(i).asList().get(j).asDouble() for i in [2, 3, 4, 5] for j in [6, 7, 8]]

  for i in range(len(values)):
    filters[i].process_sample(values[i])


  bottle = port_filtered_torques.prepare()
  bottle.clear()

  # take a bandpass approach to filter out both noise and bias
  # the s2**b term gives more weight to the higher frequencies
  s2 = sqrt(2)
  for i in range(len(filters)):
    filtered[i] = sum([filters[i].bands[b]*(s2**b) for b in range(2,5)])
    bottle.addDouble(filtered[i])

  port_filtered_torques.write()


  # compute measure for touch: project onto a 'template' torque vector
  val = sum([v*t for v, t in zip(filtered, template)])/template_l2

  # compute how much the measurement deviates from the desired vector
  # (didn't work that well...)
  rest = [f - sqrt(template_l2)*t*val for f,t in zip(filtered,template)]
  diff = sqrt(sum([r*r for r in rest]))/sqrt(template_l2)

  bottle = port_touch.prepare()
  bottle.clear()
  bottle.addDouble(val)
  bottle.addDouble(diff)
  port_touch.write()
 

  # try to detect jumps using only the highest frequencies
  for i in range(len(filters)):
    jump = sum([filters[i].bands[b]*(s2**b) for b in range(6,8)])
    jumps[i] = abs(jump)

  bottle = port_hf.prepare()
  bottle.clear()
  bottle.addDouble(jumps[0] + jumps[1] + jumps[2])
  bottle.addDouble(jumps[3] + jumps[4] + jumps[5])
  bottle.addDouble(jumps[6] + jumps[7] + jumps[8])
  bottle.addDouble(jumps[9] + jumps[10] + jumps[11])
  port_hf.write()

  if (jumps[0] + jumps[1] + jumps[2] > 1.7 or
     jumps[3] + jumps[4] + jumps[5] > 1.7 or
     jumps[6] + jumps[7] + jumps[8] > 1.7):
    last_hf = yarp.Time_now()


  # try to determine touch in the presence of finger jumps
  if val > 0.3 and yarp.Time_now() - last_hf > 0.8:
    val_filtered = 10.0
    last_val_filtered = yarp.Time_now()
  else:
    val_filtered = 0

  bottle = port_touch_filtered.prepare()
  bottle.clear()
  bottle.addDouble(val_filtered)
  bottle.addDouble(last_val_filtered)
  port_touch_filtered.write()
