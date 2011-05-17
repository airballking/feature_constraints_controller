#!/usr/bin/env python
from math import *

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

# mapping:   button  |-->  ([sign]*6, [axis]*6)

class Mapping:
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
    """ map joystick axes onto a twist """
    if len(msg.axes) < max(self.axis):
      return Twist()

    values = [s*msg.axes[a] for (s,a) in zip(self.sign, self.axis)]
    return Twist(Vector3(*values[0:3]), Vector3(*values[3:6]))

# handle joystick messages
def callback(msg):
  global mappings
  for b, pressed in enumerate(msg.buttons):
    if pressed and mappings.has_key(b):
      pub.publish(mappings[b](msg))
      return
  if mappings.has_key('_'):
    pub.publish(mappings['_'](msg))


mappings = {}

rospy.init_node('joy_to_twist')

name = rospy.get_name()
params = [n for n in rospy.get_param_names() if n.startswith(name+'/mapping')]

for n in params:
  import re
  btn = re.search('(\d*)$', n).group(0)
  p = rospy.get_param(n, None)
  if btn != '':
    mappings[int(btn)] = Mapping(p)
  else:
    mappings['_'] = Mapping(p)

# default mapping
if len(mappings.keys()) == 0:
  mappings['_'] = Mapping([1, 0, '_',   '_', '_', 3])

# send some log output
for k in mappings.keys():
  rospy.loginfo('%s -> %s' % (str(k), str(mappings[k])))

# connect and start
rospy.Subscriber("/joy", Joy, callback)
pub = rospy.Publisher('/twist', Twist)
rospy.spin()

