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
      innovation = 0.001
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

filters = [None]*9

for i in range(len(filters)):
  filters[i] = TemporalFilter(8)

template = [-0.15, 0.32, -0.35, 0.08, 1.2, -0.47, 0, 0, 0]
template_l2 = sum([v*v for v in template])
filtered = [0.0]*9


while True:
  bottle = port_hand_in.read(True)
  values = [bottle.get(i).asList().get(j).asDouble() for i in [2, 3, 4] for j in [6, 7, 8]]

  for i in range(9):
    filters[i].process_sample(values[i])

  bottle = port_filtered_torques.prepare()
  bottle.clear()

  s2 = sqrt(2)

  #for i in range(9):
  #  bottle.addList()
  #  b2 = bottle.get(i).asList()
  #  for j in range(8):
  #    b2.addDouble(filters[i].bands[j]*(s2**j))

  for i in range(9):
    filtered[i] = sum([filters[i].bands[b]*(s2**b) for b in range(2,7)])

  val = sum([v*t for v, t in zip(filtered, template)])/template_l2

  rest = [f - sqrt(template_l2)*t*val for f,t in zip(filtered,template)]

  diff = sqrt(sum([r*r for r in rest]))/sqrt(template_l2)

  for i in range(9):
    bottle.addDouble(filtered[i])

  bottle.addDouble(val)
  bottle.addDouble(diff)

  port_filtered_torques.write()
