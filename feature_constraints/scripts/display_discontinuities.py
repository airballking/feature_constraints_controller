#!/usr/bin/python

import sys
import roslib; roslib.load_manifest('motion_viz')
import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import PyKDL as kdl
import marker

rospy.init_node('display_discontinuities')

file_name = sys.argv[1]
f = open(file_name)

s = 0.07
alpha = 0.5

mrk = marker.create(type=marker.Marker.CUBE_LIST)
mrk.color = ColorRGBA(0,0,0,alpha)
mrk.scale = Vector3(s, s, s)

for line in f:
  data = map(float, line.split())
  ori = kdl.Rotation.Quaternion(*data[0:4])
  amplitude = data[4]
  mrk.points.append(Point(*ori.UnitX()))
  mrk.points.append(Point(*ori.UnitY()))
  mrk.points.append(Point(*ori.UnitZ()))
  mrk.colors.append(ColorRGBA(1,0,0,alpha))
  mrk.colors.append(ColorRGBA(0,1,0,alpha))
  mrk.colors.append(ColorRGBA(0,0,1,alpha))

f.close()

marker.publish(mrk)

