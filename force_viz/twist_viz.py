#!/usr/bin/python

# show a twist in rviz

from math import *

import roslib
roslib.load_manifest('force_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as posemath
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Twist


def cross(a, b):
  """ cross product of two kdl vectors """
  x = a.y()*b.z() - a.z()*b.y()
  y = a.z()*b.x() - a.x()*b.z()
  z = a.x()*b.y() - a.y()*b.x()
  return kdl.Vector(x, y, z)


def make_marker(id=0, ns='twist', type=Marker.CUBE, action=Marker.ADD):
  """ create a marker message for rviz """
  m = Marker()

  m.id = id
  m.ns = ns
  m.type = type
  m.action = action

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
      * frm, to: kdl.Vector
      * width: scalar
  """
  eps = 1e-10 # some small number for numeric stability test

  direction = to - frm
  scale = direction.Norm()

  if marker.type == Marker.ARROW:
    marker.pose.position = Point(frm.x(), frm.y(), frm.z())
    marker.scale = Vector3(x=0.771*scale, y=width, z=width)

    axis = cross(kdl.Vector(1,0,0), direction)
    angle = direction.x()
  else:
    midpoint = (frm + to)*0.5
    marker.pose.position = Point(midpoint.x(), midpoint.y(), midpoint.z())
    marker.scale = Vector3(x=width, y=width, z=scale)

    axis = cross(kdl.Vector(0,0,1), direction)
    angle = direction.z()

  laxis = axis.Norm()
  if laxis > eps and scale > eps:
    l  = sqrt((1 - angle / scale) / 2) / laxis
    qu = sqrt((1 + angle / scale) / 2)
    q = [axis.x()*l, axis.y()*l, axis.z()*l, qu]
  else:
    # aligned with x/z-axis or zero-length: no rotation needed
    q = [0, 0, 0, 1]

  marker.pose.orientation = Quaternion(*q)
  return marker


def axis_marker(tw):
  """ make a marker message showing the instantaneous
      rotation axis of a twist message"""

  t = kdl.Twist(kdl.Vector(tw.linear.x,  tw.linear.y,  tw.linear.z),
                kdl.Vector(tw.angular.x, tw.angular.y, tw.angular.z))

  try:
    (x,     rot)  = listener.lookupTransform(target_frame, ref_frame, rospy.Time(0))
    (trans, rotp) = listener.lookupTransform(target_frame, ref_point, rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException):
    print 'tf exception!'
    return make_marker(id=12, ns='twist', action=Marker.DELETE)

  # put the twist in the right frame
  f = posemath.fromTf( (trans, rot) )
  t = f*t

  direction = t.rot
  location = cross(t.rot, t.vel) / kdl.dot(t.rot, t.rot)

  min_length = 0.08
  l = direction.Norm()

  if l == 0:
    # remove this marker
    return make_marker(id=12, ns='twist', action=Marker.DELETE)
  elif l < min_length:
    direction = direction / l * min_length

  m = make_marker(12, 'twist', Marker.CYLINDER)
  m.header.frame_id = target_frame
  m.frame_locked = True
  m = align(m, location - direction, location + direction, 0.02)

  return m

# main #

def callback(tw):
  pub.publish(axis_marker(tw))


rospy.init_node('twist_viz')

target_frame = rospy.get_param('~target_frame', '/base_link')
ref_frame = rospy.get_param('~ref_frame', '/base_link')
ref_point = rospy.get_param('~ref_point', ref_frame)

listener = tf.TransformListener()

pub = rospy.Publisher('/visualization_marker', Marker)
rospy.Subscriber("twist", Twist, callback)
rospy.spin()
