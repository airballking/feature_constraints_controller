#!/usr/bin/env python
from math import *

import roslib; roslib.load_manifest('joy_to_twist')
import rospy

from joy.msg import Joy
from geometry_msgs.msg import Twist,Vector3

import mapping
mapping.default_mapping = [1, 0, '_',   '_', '_', 3]


rospy.init_node('joy_to_twist')

button_mapping = mapping.MultiMapping()

def callback(msg):
  values = button_mapping(msg)
  twist = Twist(Vector3(*values[0:3]), Vector3(*values[3:6]))
  pub.publish(twist)

# send some log output
for line in str(button_mapping).splitlines():
  rospy.loginfo(line)

# connect and start
rospy.Subscriber("/joy", Joy, callback)
pub = rospy.Publisher('/twist', Twist)
rospy.spin()

