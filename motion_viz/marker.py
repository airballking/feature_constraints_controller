#!/usr/bin/python

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
import PyKDL as kdl

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench

def create(**args):
  m = Marker(**args)
  # make shure the marker is displayable
  if not args.has_key('header'):
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = '/base_link'
  if not args.has_key('color'):
    m.color = ColorRGBA(0.3, 0.3, 0.3, 1)
  if not args.has_key('orientation'):
    m.pose.orientation = Quaternion(0, 0, 0, 1)
  if not args.has_key('scale'):
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
  eps = 1e-10 # some small number for alignment test

  direction = to - frm
  scale = direction.Norm()

  if marker.type == Marker.ARROW:
    marker.pose.position = Point(frm.x(), frm.y(), frm.z())
    marker.scale = Vector3(x=0.771*scale, y=width, z=width)

    axis = kdl.Vector(1,0,0) * direction
    angle = direction.x()
  else:
    midpoint = (frm + to)*0.5
    marker.pose.position = Point(midpoint.x(), midpoint.y(), midpoint.z())
    marker.scale = Vector3(x=width, y=width, z=scale)

    axis = kdl.Vector(0,0,1) * direction
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


def arrow(frm, to, width, ns=''):
  marker = create(ns=ns, type=Marker.ARROW)
  marker = align(marker, frm, to, width)
  return [marker]


def publish(marker):
  if publish.publisher == None:
    publish.publisher = rospy.Publisher('visualization_marker', Marker)
    rospy.sleep(0.5) # hack to wait for the connections to establish
  publish.publisher.publish(marker)
publish.publisher = None

publish.publisher = None
