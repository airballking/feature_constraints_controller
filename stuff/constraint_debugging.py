#!/usr/bin/python

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, ConstraintCommand, ConstraintState

from math import *
import numpy

def jacmsg2matrix(jac):
  m = numpy.matrix(numpy.zeros((6, len(jac))))
  for (i, col) in enumerate(jac):
    m[0,i] = jac[i].linear.x
    m[1,i] = jac[i].linear.y
    m[2,i] = jac[i].linear.z
    m[3,i] = jac[i].angular.x
    m[4,i] = jac[i].angular.y
    m[5,i] = jac[i].angular.z
  return m

def recover_ydot(state, q_dot):
  H = jacmsg2matrix(state.interaction_matrix).T
  J_R = jacmsg2matrix(state.robot_jacobian)
  q_dot = numpy.matrix(q_dot).T

  return H * J_R * q_dot


def extract_qdot(msg, joint_names):
  q_dot = [0.0]*len(joint_names)
  for i in range(len(msg.name)):
    if msg.name[i] in joint_names:
      q_dot[joint_names[msg.name[i]]] = msg.velocity[i]
  return q_dot


def joint_names(prefix):
  roslib.load_manifest('urdf_parser_py')
  import urdf_parser_py.urdf as urdf

  tool_frame = rospy.get_param(prefix+'/tool_frame')
  base_frame = rospy.get_param(prefix+'/base_frame')

  model = urdf.URDF.load_from_parameter_server()
  names_unicode = model.get_chain(base_frame, tool_frame, True, False, False)
  names = [n.encode('ascii', 'ignore') for n in names_unicode]
  name_map = {}
  for (i, name) in enumerate(names):
    name_map[name] = i
  return name_map

