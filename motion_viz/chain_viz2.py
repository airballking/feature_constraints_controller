#!/usr/bin/python

# send marker messages to visualize a KDL chain

# (specialized for pancake baking for now)

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy

import PyKDL as kdl
from tf_conversions import posemath as pm

from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3
from motion_viz.msg import ChainState, ChainInfo, Segment

# Note: we don't use kdl chain because
# 1) it does not store enough information for the "special" chains handled here
# 2) it is too restricted w.r.t. the axes that it can store (only RotX, RotY, ...)
#    there is an implementation in kdl now, but not in the python bindings


import marker


class ChainDrawer:

  def __init__(self):
    self.chain_info = None
    self.next_id = 0
    self.angles = []
    self.desired_angles = []
    self.angle_errors = []
    self.chain_state = None
    # configuration
    self.axisWidth = 0.02
    self.axisLength = 2.5 * self.axisWidth
    self.linkWidth = self.axisWidth / 2.0
    self.linkColor = ColorRGBA(0.5, 0.5, 0.5, 1.0)
    self.error_length = 0.2
    self.error_angle  = pi/6

  def set_chain_info(self, chain_info):
    self.chain_info = chain_info

  def set_state(self, chain_state):
    self.chain_state = chain_state

  def get_markers(self):
    if self.chain_info == None or self.chain_state == None:
      rospy.logwarn('no state yet')
      return []

    markers = []
    self.next_id = 0

    nJoints = len([1 for s in self.chain_info.segments if s.type != Segment.FIXED])
    nSegments = len(self.chain_info.segments)

    if (len(self.chain_state.position_measured) < nJoints or
        len(self.chain_state.position_desired) < nJoints):
      rospy.logwarn('state invalid')
      return []

    f = kdl.Frame.Identity()
    j = 0
    for i in range(nSegments):
      seg_markers = []

      segment = self.chain_info.segments[i]

      a = segment.axis
      axis = kdl.Vector(a.x, a.y, a.z)
      type = segment.type

      q = self.chain_state.position_measured[j]
      q_des = self.chain_state.position_desired[j]
      if type != Segment.FIXED:
        j += 1  # used one joint angle, proceed with next one

      # this starts a new chain piece
      if segment.reset_transform:
        f = kdl.Frame()

      # create the axis marker
      if type == Segment.ROTATIONAL:
        joint_pose = kdl.Frame(kdl.Rotation.Rot2(axis, q))
        seg_markers.append(self._rot_marker(f, axis, q, q_des))
      if type == Segment.TRANSLATIONAL:
        joint_pose = kdl.Frame(axis * q)
        seg_markers.append(self._trans_marker(f, axis, q, q_des))
      if type == Segment.FIXED:
        joint_pose = kdl.Frame()
        seg_markers.append(self._fixed_marker(f))

      # draw skeleton
      f_next = f * joint_pose * pm.fromMsg(segment.link)
      if f.p != f_next.p:
        seg_markers.extend(self._link_markers(f.p, f_next.p))
      f = f_next

      # configure 'the rest'
      for m in seg_markers:
        m.ns = self.chain_info.name
        m.header.frame_id = segment.frame_id

      markers.extend(seg_markers)

    return markers


  def _rot_marker(self, f, axis, q, q_des):
    m = marker.create(type=Marker.CYLINDER, id=self.next_id)
    self.next_id += 1

    p1 = f*(-axis) * self.axisLength
    p2 = f*  axis  * self.axisLength

    marker.align(m, p1, p2, self.axisWidth)
    m.color = self._color_code(abs(q - q_des) / self.error_angle)
    return m


  def _trans_marker(self, f, axis, q, q_des):
    m = marker.create(type=Marker.CUBE, id=self.next_id)
    self.next_id += 1

    p1 = f.p
    p2 = f*(axis*q)

    marker.align(m, p1, p2, self.axisWidth)
    m.color = self._color_code(abs(q - q_des) / self.error_length)
    return m

  def _fixed_marker(self, f):
    m = marker.create(type=Marker.SPHERE, id=self.next_id)
    m.scale = Vector3(*([self.axisWidth]*3))
    m.color = self.linkColor
    self.next_id += 1
    return m

  def _link_markers(self, frm, to):
    m = marker.create(type=Marker.CYLINDER, id=self.next_id)
    marker.align(m, frm, to, self.linkWidth)
    m.color = self.linkColor

    mj = marker.create(type=Marker.SPHERE, id=self.next_id+1)
    mj.scale = Vector3(*([self.linkWidth]*3))
    mj.pose.position = Point(to[0], to[1], to[2])
    mj.color = self.linkColor
    self.next_id += 2
    return [m, mj]


  def _color_code(self, index):
    # TODO: stress test this!
    try:
      intervals = [0.0, 0.5, 1.0]
      colors = [[0.0, 1.0, 0.0, 1.0],
                [1.0, 1.0, 0.0, 1.0],
                [1.0, 0.0, 0.0, 1.0]]
 
      # handle corner cases
      if index <= intervals[0]:
        return ColorRGBA(*colors[0])
 
      if index >= intervals[-1]:
        return ColorRGBA(*colors[-1])
 
      # pick interval
      (i0, i1) = ((i-1,i+1) for i,t in enumerate(intervals) if t >= index).next()
      (t0, t1) = intervals[i0:i1]
      (c0, c1) = colors[i0:i1]
 
      t = (index-t0)/(t1-t0)
      return ColorRGBA(*[(1-t)*x0 + t*x1 for (x0,x1) in zip(c0,c1)])
    except e:
      print 'index: '+index
      print e


# main #

rospy.init_node('chain_viz')

global redraw_flag
redraw_flag = True

drawer = ChainDrawer()


def callback_structure(msg):
  # change the structure of the chain drawer
  print 'got chain info'
  drawer.set_chain_info(msg)

def callback_state(msg):
  # set new angles
  global redraw_flag
  print 'got chain state'
  drawer.set_state(msg)
  if redraw_flag:
    redraw_flag = False
    redraw()


def redraw():
  mrks = drawer.get_markers()
  for m in mrks:
    pub.publish(m)


pub = rospy.Publisher('/visualization_marker', Marker)

rospy.Subscriber("/chain_info", ChainInfo, callback_structure)
rospy.Subscriber("/chain_state", ChainState, callback_state)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
  redraw_flag = True
  rate.sleep()

