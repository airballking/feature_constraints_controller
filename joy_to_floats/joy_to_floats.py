#!/usr/bin/env python
from math import *

import roslib; roslib.load_manifest('joy_to_floats')
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

import mapping

rospy.init_node('joy_to_floats')

button_mapping = mapping.MultiMapping(default_mapping=[1,2,3])

def callback(msg):
  values = button_mapping(msg)
  floats = Float64MultiArray(data=values)
  pub.publish(floats)


if __name__ == "__main__":

  # send some log output
  for line in str(button_mapping).splitlines():
    rospy.loginfo(line)

  # connect and start
  rospy.Subscriber("/joy", Joy, callback)
  pub = rospy.Publisher('/floats', Float64MultiArray)
  rospy.spin()

