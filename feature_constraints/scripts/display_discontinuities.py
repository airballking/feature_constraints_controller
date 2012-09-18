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

s = 0.015
radius = 0.13
alpha = 0.02

mrk = marker.create(type=marker.Marker.POINTS)
mrk.header.frame_id = '/cylinder_pose'
mrk.ns = 'discontinuities'
mrk.color = ColorRGBA(0,0,0,alpha)
mrk.scale = Vector3(s, s, s)
mrk.frame_locked = True

for line in f:
  data = map(float, line.split())
  ori = kdl.Rotation.Quaternion(*data[0:4])
  amplitude = data[4]
  mrk.points.append(Point(*(ori.UnitX()*radius)))
  mrk.points.append(Point(*(ori.UnitY()*radius)))
  mrk.points.append(Point(*(ori.UnitZ()*radius)))
  mrk.colors.append(ColorRGBA(1,0,0,alpha))
  mrk.colors.append(ColorRGBA(0,1,0,alpha))
  mrk.colors.append(ColorRGBA(0,0,1,alpha))

f.close()

marker.publish(mrk)

