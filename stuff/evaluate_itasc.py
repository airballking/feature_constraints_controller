#!/usr/bin/python

from math import *

import roslib ; roslib.load_manifest('motionControl')
import rospy

from std_msgs.msg import Int8

import time

import thing_tools

rospy.init_node('evaluate_itasc')


state = 0

def callback_state(msg):
  global state
  state = msg.data

sub_state = rospy.Subscriber("/state", Int8, callback_state)

itasc = thing_tools.handleItasc.HandleItasc(side='right')

trace = []

while True:
  if state == 0:
    trace = []

  if state == 1:
    # recording
    chi = itasc.getAngles()
    chi_des = itasc.chi_desired
    weights = itasc.weightsll

    trace.append( (chi, chi_des, weights) )

  if state == 2:
    # stopped. evaluate trace
    errors = []
    for chi, chi_des, weights in trace:
      error = [abs(c - c_des)*w for c,c_des,w in zip(chi, chi_des, weights)]
      error_rot = sqrt(error[0]**2 + error[3]**2 + error[4]**2 + error[5]**2)
      error_lin = sqrt(error[1]**2 + error[2]**2)
      errors.append((error_lin, error_rot))

    f = open('/tmp/motion_trace', 'w')
    for e_lin, e_rot in errors:
      f.write('%f %f\n' % (e_lin, e_rot) )
    f.close()

  time.sleep(0.005)
