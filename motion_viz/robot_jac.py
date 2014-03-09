#!/usr/bin/python

# extract the robot Jacobian from constraint state message

import roslib
roslib.load_manifest('motion_viz')
import rospy

from motion_viz.msg import Jacobian
from constraint_msgs.msg import ConstraintState

jac = Jacobian()

def callback(msg):
  jac.columns = msg.robot_jacobian
  pub.publish(jac)

rospy.init_node('robot_jac')

pub = rospy.Publisher('jacobian', Jacobian)
sub = rospy.Subscriber('constraint_state', ConstraintState, callback)

rospy.spin()

