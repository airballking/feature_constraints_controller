#!/usr/bin/python

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench

def make_marker(id=0, ns='marker', type=Marker.CUBE):
  m = Marker()

  m.id = id
  m.ns = ns
  m.type = type
  m.action = m.ADD

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = '/base_link'

  # live forever
  m.lifetime.secs = 0
  m.lifetime.nsecs = 0

  m.color = ColorRGBA(0.3, 0.3, 0.3, 1)

  m.pose.position = Point(0, 0, 0)
  m.pose.orientation = Quaternion(0, 0, 0, 1)
  m.scale = Vector3(1, 1, 1)

  return m


def align(marker, frm, to, width):
  """ aligns and scales a marker to connect two given points in space
      (works for arrows, cylinders and cubes
  """
  eps = 1e-10 # some small number for numeric stability test

  d = [a-b for a,b in zip(to, frm)]
  scale = sqrt(d[0]**2 + d[1]**2 + d[2]**2)

  if marker.type == Marker.ARROW:
    marker.pose.position = Point(*frm)
    marker.scale = Vector3(x=0.771*scale, y=width, z=width)

    axis = [0, -d[2], d[1]]; # cross([1 0 0]', dir);
    angle = d[0]
  else:
    midpoint = [(a+b)/2.0 for (a,b) in zip(frm, to)]
    marker.pose.position = Point(*midpoint)
    marker.scale = Vector3(x=width, y=width, z=scale)

    axis = [-d[1], d[0], 0]; # cross([0 0 1]', dir);
    angle = d[2]

  laxis = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)

  if laxis > eps and scale > eps:
    l  = sqrt((1 - angle / scale) / 2) / laxis
    qu = sqrt((1 + angle / scale) / 2)
    q = [qu, axis[0]*l, axis[1]*l, axis[2]*l]
  else:
    # aligned with x/z-axis or zero-length: no rotation needed
    q = [1, 0, 0, 0]

  marker.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

  return marker


def arrow(frm, to, width, ns=''):
  marker = make_marker(id=10, ns=ns, type=Marker.ARROW)
  marker = align(marker, frm, to, width)
  return [marker]


