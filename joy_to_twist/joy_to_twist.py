#!/usr/bin/env python
from math import *

# rosparam mapping:="[ lin.x, lin.y, lin.z, ang.x, ang.y, ang.z ]"
# where each entry can be:
# - a positive number: use that joystick axis
# - a negative number: use that joystick axis (inverted)
# - a string: always send 0
# for inverting axis 0 use -0.0
# defaults to "[1, 0, _, _, _, 3]"

import roslib; roslib.load_manifest('joy_to_twist')
import rospy

from joy.msg import Joy
from geometry_msgs.msg import Twist

axis = [0]*6
sign = [0.0]*6

def parse_mapping(param):
  global axis, sign
  for (i,a) in enumerate(param):
    if type(a) in (int,long,float):
      sign[i] = copysign(1.0, a)
      axis[i] = int(abs(a))
    else:
      sign[i] = 0.0

  l = ['']*6
  for i in range(6):
    names = ['lin.x', 'lin.y', 'lin.z', 'rot.x', 'rot.y', 'rot.z']
    if sign[i] != 0.0:
      l[i] = "%s = %saxis[%d]" % (names[i], ['-',' '][sign[i] > 0], axis[i])
    else:
      l[i] = "%s = _" % (names[i])

  rospy.loginfo('['+', '.join(l)+']')


def callback(msg):
  global axis, sign
  if len(msg.axes) < max(axis):
    return
  tw = Twist()
  tw.linear.x  = sign[0] * msg.axes[axis[0]]
  tw.linear.y  = sign[1] * msg.axes[axis[1]]
  tw.linear.z  = sign[2] * msg.axes[axis[2]]
  tw.angular.x = sign[3] * msg.axes[axis[3]]
  tw.angular.y = sign[4] * msg.axes[axis[4]]
  tw.angular.z = sign[5] * msg.axes[axis[5]]
  pub.publish(tw)


rospy.init_node('joy_to_twist')
p = rospy.get_param('~mapping', [1, 0, '_',   '_', '_', 3])
parse_mapping(p)


rospy.Subscriber("/joy", Joy, callback)
pub = rospy.Publisher('/twist', Twist)
rospy.spin()

