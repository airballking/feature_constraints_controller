#!/usr/bin/python

from math import *

import roslib
roslib.load_manifest('feature_constraints_standalone')
roslib.load_manifest('joy_to_twist')

import rospy

rospy.init_node('constraint_commander')

from sensor_msgs.msg import Joy
from constraint_msgs.msg import ConstraintCommand, ConstraintState

eps = rospy.get_param('~eps', 0.02)
scale = rospy.get_param('~scale', 0.03)
reset_button = rospy.get_param('~reset_button', 0)

chi = []
command = ConstraintCommand()

import mapping
button_mapping = mapping.MultiMapping(default_mapping=[0, 1, 2, 3])

def command_init(values):
  command.pos_lo = [x-eps for x in values]
  command.pos_hi = [x+eps for x in values]
  command.weight = [1.0]*len(values)


def callback_joy(msg):
  global command
  global chi

  if msg.buttons[reset_button] == 1:
    command_init(chi)

  values = button_mapping(msg)

  for i in range(min(len(values), len(chi))):
    command.pos_lo[i] += values[i]*scale
    command.pos_hi[i] += values[i]*scale

  pub.publish(command)


def callback_state(msg):
  global chi
  global command
  chi = msg.chi

  if len(command.weight) == 0:
    command_init(chi)

if __name__ == "__main__":
  # connect and start
  sub_change = rospy.Subscriber("/joy", Joy, callback_joy)
  sub_state    = rospy.Subscriber("/state", ConstraintState, callback_state)
  pub = rospy.Publisher('/command', ConstraintCommand)
  rospy.spin()

