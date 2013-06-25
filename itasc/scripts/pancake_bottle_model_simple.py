#!/usr/bin/python

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from math import *

rospy.init_node('pancake_bottle_model_simple')
pub = rospy.Publisher('visualization_marker', Marker)

# default: /l_gripper_tool_frame
import sys
frame_name = sys.argv[1]

# some convenience color definitions
yellow_white = ColorRGBA(255.0/255.0, 224.0/255.00, 150.0/255.0, 1)
dark_blue = ColorRGBA(4.0/255.0, 29.0/255.00, 138.0/255.0, 1)



def publishCube(pos, ori, scale, color, id):
  m = Marker()

  m.id = id
  m.ns = frame_name
  m.action = m.ADD
  m.type = m.CUBE

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name

  m.lifetime = rospy.Duration(4.0)

  m.color = color

  m.pose.position = Point(*pos)
  m.pose.orientation = Quaternion(*ori)
  m.scale = Vector3(*scale)

  m.frame_locked = True

  pub.publish(m)

def publishCylinder(pos, ori, scale, color, id):
  m = Marker()

  m.id = id
  m.ns = frame_name
  m.action = m.ADD
  m.type = m.CYLINDER

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name

  m.lifetime = rospy.Duration(4.0)

  m.color = color

  m.pose.position = Point(*pos)
  m.pose.orientation = Quaternion(*ori)
  m.scale = Vector3(*scale)

  m.frame_locked = True

  pub.publish(m)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
  publishCylinder([0, 0, -0.055], [0, 0, 0, 1], [0.075, 0.04, 0.11], yellow_white, 1)
  publishCylinder([0, 0, 0.03], [cos(90), 0, 0, cos(90)], [0.08, 0.08, 0.038], yellow_white, 2)
  publishCylinder([0, 0, 0.075], [0, 0, 0, 1], [0.035, 0.035, 0.015], dark_blue, 3)
  rate.sleep()

