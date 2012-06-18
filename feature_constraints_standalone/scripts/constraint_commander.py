#!/usr/bin/python

from math import *

import roslib; roslib.load_manifest('feature_constraints_standalone')
import rospy

rospy.init_node('constraint_commander')

from std_msgs.msg import Float64MultiArray
from constraint_msgs.msg import ConstraintCommand

eps = rospy.get_param('~eps', 0.02)
scale = rospy.get_param('~scale', 0.03)

command = ConstraintCommand()

def command_init(values):
  command.pos_lo = [x-eps for x in values]
  command.pos_hi = [x+eps for x in values]
  command.weight = [1.0]*len(values)


def callback_change(msg):
  global command

  if len(command.weight) != len(msg.data):
    command_init([0.0]*len(msg.data))

  values = msg.data
  for i in range(len(values)):
    command.pos_lo[i] += values[i]*scale
    command.pos_hi[i] += values[i]*scale

  pub.publish(command)


def callback_set(msg):
  global command

  command_init(msg.data)

  pub.publish(command)


if __name__ == "__main__":
  # connect and start
  sub_change = rospy.Subscriber("/velocity", Float64MultiArray, callback_change)
  sub_set    = rospy.Subscriber("/position", Float64MultiArray, callback_set)
  pub = rospy.Publisher('/command', ConstraintCommand)
  rospy.spin()

