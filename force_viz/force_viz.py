#!/usr/bin/python

# show a force in rviz

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench

rospy.init_node('force_viz')

# playground for now

def make_marker(id=0, ns='force', type=Marker.CUBE):
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
  m.pose.orientation = Quaternion(0, 0, 0, 0)
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


def axis_areas(frm, to, ns=''):
  m1 = make_marker(11, ns=ns, type=Marker.TRIANGLE_LIST)
  m1.color = ColorRGBA(1, 0, 0, 0.3)

  m1.points.append(Point(to[0], to[1], to[2]))
  m1.points.append(Point(to[0], frm[1], frm[2]))
  m1.points.append(Point(frm[0], frm[1], frm[2]))

  m2 = make_marker(12, ns=ns, type=Marker.TRIANGLE_LIST)
  m2.color = ColorRGBA(0, 1, 0, 0.3)

  m2.points.append(Point(to[0], to[1], to[2]))
  m2.points.append(Point(frm[0], to[1], frm[2]))
  m2.points.append(Point(frm[0], frm[1], frm[2]))

  m3 = make_marker(13, ns=ns, type=Marker.TRIANGLE_LIST)
  m3.color = ColorRGBA(0, 0, 1, 0.3)

  m3.points.append(Point(to[0], to[1], to[2]))
  m3.points.append(Point(frm[0], frm[1], to[2]))
  m3.points.append(Point(frm[0], frm[1], frm[2]))

  return [m1, m2, m3]


def cylinders(frm, to, diameter, ns=''):

  m1 = make_marker(21, ns=ns, type=Marker.CYLINDER)
  m1.color = ColorRGBA(1, 0, 0, 0.7)
  m1 = align(m1, frm, [to[0], frm[1], frm[2]], diameter)

  m2 = make_marker(22, ns=ns, type=Marker.CYLINDER)
  m2.color = ColorRGBA(0, 1, 0, 0.7)
  m2 = align(m2, frm, [frm[0], to[1], frm[2]], diameter)

  m3 = make_marker(23, ns=ns, type=Marker.CYLINDER)
  m3.color = ColorRGBA(0, 0, 1, 0.7)
  m3 = align(m3, frm, [frm[0], frm[1], to[2]], diameter)

  return [m1, m2, m3]


def wrench_markers(w):
  markers = []
  zero = [0.0, 0.0, 0.0]
  force = [w.force.x, w.force.y, w.force.z]
  torque = [w.torque.x, w.torque.y, w.torque.z]

  markers.extend(arrow(zero, force, 0.5, 'force_arrow'))
  markers.extend(axis_areas(zero, force, 'force_areas'))
  markers.extend(cylinders(zero, force, 0.1, 'force_cylinders'))

  markers.extend(arrow(zero, torque, 0.5, 'torque_arrow'))
  markers.extend(axis_areas(zero, torque, 'torque_areas'))
  markers.extend(cylinders(zero, torque, 0.1, 'torque_cylinders'))

  return markers

# main #
pub = rospy.Publisher('/visualization_marker_array', MarkerArray)
rospy.sleep(1)

w = Wrench
w.force = Vector3(1, 0.3, 0.6)
w.torque = Vector3(-0.5, 0.6, 0.3)

mrk = MarkerArray()
mrk.markers.extend(wrench_markers(w))

pub.publish(mrk)

rospy.sleep(1)
