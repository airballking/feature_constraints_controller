#!/usr/bin/python

# show a force in rviz

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Twist

rospy.init_node('force_viz')


class Vec:
  def __init__(self, x, y, z):
    self.x = float(x)
    self.y = float(y)
    self.z = float(z)

  def __str__(self):
    return '%.3f %.3f %.3f' % (self.x, self.y, self.z)
  def __repr__(self):
    return '<' + str(self) + '>'

  def __add__(self, other):
    return Vec(self.x + other.x,  self.y + other.y, self.z + other.z)
  def __sub__(self, other):
    return Vec(self.x - other.x,  self.y - other.y, self.z - other.z)
  def __neg__(self):
    return Vec(-self.x, -self.y, -self.z)

  def __mul__(self, s):
    return Vec(self.x*s, self.y*s, self.z*s)
  def __rmul__(self, s):
    return self*s
  def __div__(self, s):
    return self*(1.0/s)
  def __rdiv__(self, s):
    return (1.0/s)*self

  def norm(self):
    from math import sqrt
    return sqrt(self.x**2 + self.y**2 + self.z**2)
  def cross(a, b):
    x = a.y*b.z - a.z*b.y
    y = a.z*b.x - a.x*b.z
    z = a.x*b.y - a.y*b.x
    return Vec(x, y, z)
  cross = staticmethod(cross)


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
      (works for arrows, cylinders and cubes.

      expected types:
      * marker: visualzation_msgs/Marker
      * frm, to: Vec
      * width: scalar
  """
  eps = 1e-10 # some small number for numeric stability test

  direction = to - frm
  scale = direction.norm()

  if marker.type == Marker.ARROW:
    marker.pose.position = Point(frm.x, frm.y, frm.z)
    marker.scale = Vector3(x=0.771*scale, y=width, z=width)

    axis = Vec.cross(Vec(1,0,0), direction)
    angle = direction.x
  else:
    midpoint = (frm + to)*0.5
    marker.pose.position = Point(midpoint.x, midpoint.y, midpoint.z)
    marker.scale = Vector3(x=width, y=width, z=scale)

    axis = Vec.cross(Vec(0,0,1), direction)
    angle = direction.z

  laxis = axis.norm()
  if laxis > eps and scale > eps:
    l  = sqrt((1 - angle / scale) / 2) / laxis
    qu = sqrt((1 + angle / scale) / 2)
    q = [qu, axis.x*l, axis.y*l, axis.z*l]
  else:
    # aligned with x/z-axis or zero-length: no rotation needed
    q = [1, 0, 0, 0]

  marker.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
  return marker


def axis_marker(tw):
  """make a marker message showing the instantaneous rotation axis of a twist message"""
  direction = Vec(tw.angular.x, tw.angular.y, tw.angular.z)
  s0 = Vec(tw.linear.x, tw.linear.y, tw.linear.z)
  location = Vec.cross(s0, direction)

  min_length = 0.08

  if direction.norm() < min_length:
    direction = direction /direction.norm() * min_length

  m = make_marker(12, 'twist', Marker.CYLINDER)
  m = align(m, location - direction, location + direction, 0.03)

  return [m]

# main #

def callback(tw):
  pub.publish(*axis_marker(tw))

pub = rospy.Publisher('/visualization_marker', Marker)
rospy.Subscriber("twist", Twist, callback)
rospy.spin()
