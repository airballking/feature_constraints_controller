#!/usr/bin/python

import yarp
yarp.Network.init()

basename = '/signal_source'

port_rect = yarp.BufferedPortBottle()
port_rect.open(basename+'/rect')

port_tri = yarp.BufferedPortBottle()
port_tri.open(basename+'/triangle')

port_saw = yarp.BufferedPortBottle()
port_saw.open(basename+'/sawtooth')

i = 0

period = 500
amplitude = 1.0

while True:
  if i > period:
    i = 0

  if i < period / 2:
    rect = -amplitude
  else:
    rect = amplitude

  tri = 2.0*rect*(2.0*i/period - 1.0) - amplitude

  saw = (2.0*i/period - 1.0) * amplitude

  i += 1

  bottle = port_rect.prepare()
  bottle.clear()
  bottle.addDouble(rect)
  port_rect.write()

  bottle = port_tri.prepare()
  bottle.clear()
  bottle.addDouble(tri)
  port_tri.write()

  bottle = port_saw.prepare()
  bottle.clear()
  bottle.addDouble(saw)
  port_saw.write()

  yarp.Time_delay(0.005)
