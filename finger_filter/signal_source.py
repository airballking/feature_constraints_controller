#!/usr/bin/python

import yarp
from math import *

def open_port(name):
  port = yarp.BufferedPortBottle()
  port.open(name)
  return port

def send_value(port, value):
  bottle = port.prepare()
  bottle.clear()
  bottle.addDouble(value)
  port.write()

yarp.Network.init()

basename = '/signal_source'

port_rect = open_port(basename + '/rect')
port_tri  = open_port(basename + '/triangle')
port_saw  = open_port(basename + '/sawtooth')
port_sine = open_port(basename + '/sine')

period = 500
amplitude = 2.5
i = 0

while True:
  i = (i + 1) % period

  if i < period / 2:
    rect = -amplitude
  else:
    rect = amplitude

  tri  = 2.0*rect*(2.0*i/period - 1.0) - amplitude
  saw  = (2.0*i/period - 1.0) * amplitude
  sine = amplitude*cos(2*pi*i/period)

  send_value(port_rect, rect)
  send_value(port_tri,  tri)
  send_value(port_saw,  saw)
  send_value(port_sine, sine)

  yarp.Time_delay(0.005)
