#!/usr/bin/python
# Copyright (c) 2010 Ingo Kresse <kresse at cs.tum.edu>
# Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.


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
