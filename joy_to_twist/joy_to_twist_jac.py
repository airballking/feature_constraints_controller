#!/usr/bin/env python
from math import *

import roslib
roslib.load_manifest('joy_to_twist')
roslib.load_manifest('motion_viz')
import rospy
import PyKDL as kdl

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist,Vector3
from motion_viz.msg import Jacobian

import mapping
mapping.default_mapping = [1, 0, '_',   '_', '_', 3]


# main

rospy.init_node('joy_to_twist_jac')

button_mapping = mapping.MultiMapping()
jacobian = [] # vector of kdl twists

# some helper functions for conversion
def vector_to_kdl(v):
  return kdl.Vector(v.x, v.y, v.z)

def twist_to_kdl(tw):
  return kdl.Twist(vector_to_kdl(tw.linear), vector_to_kdl(tw.angular))

def twist_to_msg(tw):
  return Twist(Vector3(tw[0], tw[1], tw[2]), Vector3(tw[3], tw[4], tw[5]))


def callback_jac(msg):
  global jacobian
  jac = []
  for j in msg.columns:
    jac.append(twist_to_kdl(j))
  jacobian = jac


def callback(msg):
  values = button_mapping(msg)
  twist = kdl.Twist()
  for (s,j) in zip(values, jacobian):
    twist += s*j
  pub.publish(twist_to_msg(twist))


# send some log output
for line in str(button_mapping).splitlines():
  rospy.loginfo(line)

# connect and start
rospy.Subscriber("/joy", Joy, callback)
rospy.Subscriber("/jacobian", Jacobian, callback_jac)
pub = rospy.Publisher('/twist', Twist)
rospy.spin()
