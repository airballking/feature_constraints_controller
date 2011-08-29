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


# create the spatula chain message
info = ChainInfo()
info.name = "virtual linkage"

# cylinder coords
s = Segment()
s.type = Segment.ROTATIONAL
s.name = 'angle'
s.pose_index = -1
s.axis = Vector3(0.0, 0.0, 1.0)
info.segments.append(s)

s = Segment()
s.type = Segment.TRANSLATIONAL
s.name = 'distance'
s.pose_index = -1
s.axis = Vector3(1.0, 0.0, 0.0)
info.segments.append(s)

s = Segment()
s.type = Segment.TRANSLATIONAL
s.name = 'height'
s.pose_index = -1
s.axis = Vector3(0.0, 0.0, 1.0)
info.segments.append(s)

#reverse RPY

s = Segment()
s.type = Segment.ROTATIONAL
s.name = 'roll'
s.pose_index = 0
s.axis = Vector3(1.0, 0.0, 0.0)
info.segments.append(s)

s = Segment()
s.type = Segment.ROTATIONAL
s.name = 'pitch'
s.pose_index = 0
s.axis = Vector3(0.0, 1.0, 0.0)
info.segments.append(s)

s = Segment()
s.type = Segment.ROTATIONAL
s.name = 'yaw'
s.pose_index = 0
s.axis = Vector3(0.0, 0.0, 1.0)
info.segments.append(s)


def _mid_pose(chi):
  mid_pos = kdl.Vector(cos(chi[0])*chi[1], sin(chi[0])*chi[1], chi[2])
  mid_pose = kdl.Frame(kdl.Rotation.Rot(kdl.Vector(1,0,0), chi[0]), mid_pos)
  return mid_pose


def fk(chi):
  mid_pose = _mid_pose(chi)
  rpy = kdl.Frame(kdl.Rotation.RPY(chi[3], chi[4], chi[5]))
  return [mid_pose * rpy.Inverse()]

def ik(pose):
  angle = atan2(pose.p.y(), pose.p.x())
  distance = sqrt(pose.p.x()**2, pose.p.y()**2)
  height = pose.p.z()

  mid_pose = _mid_pose([angle, distance, height])
  rot = pose * (mid_pose.Inverse()*pose).M.Inverse()

  (roll, pitch, yaw) = rot.GetRPY()

  return [angle, distance, height, roll, pitch, yaw]



def color_code(index):
  intervals = [0.0, 0.5, 1.0]
  colors = [[0.0, 1.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 1.0]]

  # handle corner cases
  if index <= intervals[0] or isnan(index):
    return ColorRGBA(*colors[0])

  if index >= intervals[-1]:
    return ColorRGBA(*colors[-1])

  # pick interval
  (i0, i1) = ((i-1,i+1) for i,t in enumerate(intervals) if t >= index).next()
  (t0, t1) = intervals[i0:i1]
  (c0, c1) = colors[i0:i1]

  t = (index-t0)/(t1-t0)
  return ColorRGBA(*[(1-t)*x0 + t*x1 for (x0,x1) in zip(c0,c1)])



#factory function for creating Segment objects
def createSegment(segment_info, seg_id, jnt_id):
  if segment_info.type == Segment.ROTATIONAL:
    return SegmentRotational(segment_info, seg_id, jnt_id)
  if segment_info.type == Segment.TRANSLATIONAL:
    return SegmentTranslational(segment_info, seg_id, jnt_id)


class SegmentBase:
  def __init__(self, segment_info, seg_id, jnt_id):
    self.chi = 0
    self.desired_chi = 0
    self.seg_id = seg_id
    self.jnt_id = jnt_id
    self.set_info(segment_info)
    # TODO: deal with the constant link offset. For virtual linkages we dont have that now...


  def set_info(self, segment_info):
    '''set type and axis of the segment'''
    a = segment_info.axis
    self.name = segment_info.name
    self.axis = kdl.Vector(a.x, a.y, a.z)
    self.pose_base_index = segment_info.pose_index

  def set_pose_base(self, poses, chain_pose):
    '''set the current pose for the segment'''
    if self.pose_base_index < 0:
      self.pose_base = chain_pose
      return chain_pose * self.pose_segment()
    else:
      self.pose_base = poses[self.pose_base_index]
      return self.pose_segment()

  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame()

  def set_chi(self, chi):
    '''set axis angle. We want to override later at this point.'''
    self.chi = chi[self.jnt_id]

  def set_desired_chi(self, chi):
    self.desired_chi = chi[self.jnt_id]

  def update(self):
    '''update the marker according to the current state.'''
   


# TODO: some config data, not shure where to put them right now...
error_length = 0.2
error_angle  = pi/6
axisWidth = 0.02
axisLength = 2.5 * axisWidth



class SegmentRotational(SegmentBase):
  def __init__(self, segment_info, seg_id, jnt_id):
    SegmentBase.__init__(self, segment_info, seg_id, jnt_id)
    self.marker = marker.create(type=Marker.CYLINDER, id=self.seg_id)

  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame(kdl.Rotation.Rot2(self.axis, self.chi))

  def update(self):
    self.update_marker()

  def update_marker(self):
    '''update the marker to reflect the current joint angle'''

    f = self.pose_base
    p1 = f*(-self.axis * axisLength)
    p2 = f*(self.axis  * axisLength)

    marker.align(self.marker, p1, p2, axisWidth)
    self.marker.color = color_code(abs(self.chi - self.desired_chi) / error_angle)


class SegmentTranslational(SegmentBase):
  def __init__(self, segment_info, seg_id, jnt_id):
    SegmentBase.__init__(self, segment_info, seg_id, jnt_id)
    self.marker = marker.create(type=Marker.CUBE, id=seg_id)

  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame(self.axis * self.chi)

  def update(self):
    self.update_marker()

  def update_marker(self):
    '''update the marker to reflect the current joint angle'''
    f = self.pose_base
    p1 = f.p
    p2 = f*(self.axis*self.chi)

    marker.align(self.marker, p1, p2, axisWidth)
    self.marker.color = color_code(abs(self.chi - self.desired_chi) / error_length)




##############################################

class ChainDrawer:

  def __init__(self):
    self.chain_info = None
    self.angles = []
    self.desired_angles = []
    self.angle_errors = []
    self.chain_state = None
    # configuration

    self.segments = []

  def set_info(self, chain_info):
    if self.segments == []:
      self.create_markers(chain_info)

  def set_state(self, chain_state):
    poses = chain_state.poses
    chi = chain_state.position_measured
    chain_pose = kdl.Frame()
    for seg in self.segments:
      seg.set_chi(chi)
      chain_pose = seg.set_pose_base(poses, chain_pose)
      seg.update()

  def create_markers(self, chain_info):
    self.frame_id = chain_info.header.frame_id
    segment_index = 0
    joint_index = 0
    for seg in chain_info.segments:
      self.segments.append(createSegment(seg, segment_index, joint_index))
      if seg.type != Segment.FIXED:
        joint_index += 1
      segment_index += 1

  def get_markers(self):
    return [s.marker for s in self.segments]


# main #

rospy.init_node('chain_viz')

global redraw_flag
redraw_flag = True

drawer = ChainDrawer()
drawer.set_info(info)

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


def callback_chi(msg):
  global redraw_flag
  state = ChainState()
  state.position_measured = msg.data
  state.poses = fk(msg.data)
  drawer.set_state(state)
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
rospy.Subscriber("/chi", Float64MultiArray, callback_chi)

rate = rospy.Rate(2)




while not rospy.is_shutdown():
  redraw_flag = True
  rate.sleep()

