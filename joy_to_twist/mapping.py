#!/usr/bin/env python
from math import *

# Performs mapping from joystick axes onto 6-vectors
# (intended for sending twists)
#
# rosparam mapping<n>:="[ lin.x, lin.y, lin.z, ang.x, ang.y, ang.z ]"
# where n is a (optional) button number and each entry can be:
# - a positive number: use that joystick axis
# - a negative number: use that joystick axis (inverted)
# - a string: always send 0
# for inverting axis 0 use -0.0
# defaults to "[1, 0, _, _, _, 3]"

import roslib; roslib.load_manifest('joy_to_twist')
import rospy

from joy.msg import Joy
from geometry_msgs.msg import Twist,Vector3


default_mapping = [1, 0, '_',   '_', '_', 3]


class Mapping:
  """ map a single set of joystick axes to twist axes """
  def __init__(self, param):
    """ param: a ROS param (the list of the six joystick axes) """
    self.axis = [0]*6
    self.sign = [0.0]*6
    for (i,a) in enumerate(param):
      if type(a) in (int,long,float):
        self.sign[i] = copysign(1.0, a)
        self.axis[i] = int(abs(a))
      else:
        self.sign[i] = 0.0

  def __repr__(self):
    ss = [['_','+', '-'][cmp(x,0)] for x in self.sign]
    aa = [[s, s+str(a)][s != '_'] for (s,a) in zip(ss, self.axis)]
    lin = 'lin: [%s, %s, %s]' % (aa[0], aa[1], aa[2])
    rot = 'rot: [%s, %s, %s]' % (aa[3], aa[4], aa[5])
    return '['+lin+'  '+rot+']'

  def __call__(self, msg):
    """ map joystick axes onto a 6-vector """
    if len(msg.axes) < max(self.axis):
      return [0.0]*6
    return [s*msg.axes[a] for (s,a) in zip(self.sign, self.axis)]


class MultiMapping:
  """ map a joystich button onto an axis mapping """
  # mapping:   button  |-->  ([sign]*6, [axis]*6)

  def __init__(self):
    """ perform the mapping of joystick axes, with button modifiers """
    self.mappings = self.read_params()

    # default mapping
    if len(self.mappings.keys()) == 0:
      self.mappings['_'] = Mapping(default_mapping)


  def read_params(self):
    name = rospy.get_name()
    params = [n for n in rospy.get_param_names() if n.startswith(name+'/mapping')]
    mappings = {}
    for n in params:
      # parse trailing button number in param name
      import re
      btn = re.search('(\d*)$', n).group(0)
      p = rospy.get_param(n, None)
      if btn != '':
        mappings[int(btn)] = Mapping(p)
      else:
        mappings['_'] = Mapping(p)

    return mappings

  def __repr__(self):
    s = ''
    for k in self.mappings.keys():
      s += '%s -> %s' % (str(k), str(self.mappings[k]))
    return s

  def __call__(self, msg):
    """ handle joystick messages """
    for b, pressed in enumerate(msg.buttons):
      if pressed and self.mappings.has_key(b):
        return self.mappings[b](msg)
    if self.mappings.has_key('_'):
      return self.mappings['_'](msg)

