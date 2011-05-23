#!/usr/bin/python

# send marker messages to visualize a KDL chain

# (specialized for pancake baking for now)

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy

import PyKDL as kdl
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench

colors = [ColorRGBA(1, 0, 0, 0.3),
          ColorRGBA(1, 1, 0, 0.3),
          ColorRGBA(0, 1, 0, 0.3),
          ColorRGBA(0, 1, 1, 0.3),
          ColorRGBA(0, 0, 1, 0.3),
          ColorRGBA(1, 0, 1, 0.3),
          ColorRGBA(1, 1, 1, 0.3),
          ColorRGBA(0.5, 0.5, 0.5, 0.3),
          ColorRGBA(1, 0, 0, 0.3),
          ColorRGBA(1, 1, 0, 0.3),
          ColorRGBA(0, 1, 0, 0.3),
          ColorRGBA(0, 1, 1, 0.3),
          ColorRGBA(0, 0, 1, 0.3),
          ColorRGBA(1, 0, 1, 0.3),
          ColorRGBA(1, 1, 1, 0.3),
          ColorRGBA(0.5, 0.5, 0.5, 0.3)]



import marker

chain_baker = kdl.Chain()
chain_baker.addSegment( kdl.Segment(kdl.Joint(kdl.Joint.RotZ))   )
chain_baker.addSegment( kdl.Segment(kdl.Joint(kdl.Joint.TransX)) )
chain_baker.addSegment( kdl.Segment(kdl.Joint(kdl.Joint.TransZ)) )


segments_spatula = [
 kdl.Segment(kdl.Joint(kdl.Joint.RotZ)),
 kdl.Segment(kdl.Joint(kdl.Joint.RotY)),
 kdl.Segment(kdl.Joint(kdl.Joint.RotX))
]

chain_spatula = kdl.Chain()
for segment in segments_spatula:
  chain_spatula.addSegment(segment)



class ChainDrawer:
  FIXED = 0
  ROT = 1
  TRANS = 2

  types = {kdl.Joint.None: FIXED,
           kdl.Joint.RotX:   ROT,   kdl.Joint.RotY:   ROT,   kdl.Joint.RotZ:   ROT,
           kdl.Joint.TransX: TRANS, kdl.Joint.TransY: TRANS, kdl.Joint.TransZ: TRANS}

  axes  = {kdl.Joint.None:   kdl.Vector(0,0,0),
           kdl.Joint.RotX:   kdl.Vector(1,0,0),
           kdl.Joint.RotY:   kdl.Vector(0,1,0),
           kdl.Joint.RotZ:   kdl.Vector(0,0,1),
           kdl.Joint.TransX: kdl.Vector(1,0,0),
           kdl.Joint.TransY: kdl.Vector(0,1,0),
           kdl.Joint.TransZ: kdl.Vector(0,0,1)}

  def __init__(self, chain, name, frame_id):
    self.chain = chain
    self.name = name
    self.frame_id = frame_id
    self.next_id = 0
    self.angles = [0.0]*chain.getNrOfJoints()
    self.desired_angles = [0.0]*chain.getNrOfJoints()
    self.angle_errors = [0.0]*chain.getNrOfJoints()
    # configuration
    self.axisWidth = 0.02
    self.axisLength = 2.5 * self.axisWidth
    self.linkWidth = self.axisWidth / 2.0
    self.linkColor = ColorRGBA(0.5, 0.5, 0.5, 1.0)
    self.error_length = 0.2
    self.error_angle  = pi/6

  def set_angles(self, angles):
    self.angles = angles

  def set_desired_angles(self, desired_angles):
    self.desired_angles = desired_angles
    self.angle_errors = [abs(act - des) for (act,des) in zip(self.angles, self.desired_angles)]

  def get_markers(self):
    markers = []
    self.next_id = 0

    nJoints = self.chain.getNrOfJoints()
    if len(self.angles) < nJoints or len(self.desired_angles) < nJoints:
      print "state invalid"
      return []

    f = kdl.Frame.Identity()
    joint = 0
    for i in range(self.chain.getNrOfSegments()):

      segment = self.chain.getSegment(i)

      t = segment.getJoint().getType()
      type = ChainDrawer.types[t]
      axis = ChainDrawer.axes[t]

      if type == ChainDrawer.ROT:
        markers.append(self._rot_marker(f, axis, joint))
      if type == ChainDrawer.TRANS:
        markers.append(self._trans_marker(f, axis, joint))
      if type == ChainDrawer.FIXED:
        markers.append(self._fixed_marker(f))

      # draw
      f_next = f * segment.pose(self.angles[joint])
      if f.p != f_next.p:
        markers.extend(self._link_markers(f.p, f_next.p))

      if type != ChainDrawer.FIXED:
        joint += 1  # used one joint angle, proceed with next one

      f = f_next

    # set name
    for m in markers:
      m.ns = self.name
      m.header.frame_id = self.frame_id
    return markers


  def _rot_marker(self, f, axis, joint):
    m = marker.create(type=Marker.CYLINDER, id=self.next_id)
    self.next_id += 1

    p1 = f*(-axis) * self.axisLength
    p2 = f*  axis  * self.axisLength

    marker.align(m, p1, p2, self.axisWidth)

    m.color = self._color_code(self.angle_errors[joint]/ self.error_angle)

    return m

  def _trans_marker(self, f, axis, joint):
    m = marker.create(type=Marker.CUBE, id=self.next_id)
    self.next_id += 1

    p1 = f.p
    p2 = f*(axis*self.angles[joint])

    marker.align(m, p1, p2, self.axisWidth)

    m.color = self._color_code(self.angle_errors[joint]/ self.error_length)

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

drawer_baker = ChainDrawer(chain_baker, 'baker_chain', '/pancake')
drawer_spatula = ChainDrawer(chain_spatula, 'spatula_chain', '/spatula')

spatula_angles = [0.0]*3
spatula_desired_angles = [0.0]*3

def callback_desired(msg):
  global spatula_desired_angles
  drawer_baker.set_desired_angles(msg.data[0:3])
  # RPY needs special handling
  spatula_desired_angles = msg.data[3:6]

def callback(msg):
  global redraw_flag, spatula_angles
  drawer_baker.set_angles(msg.data[0:3])
  spatula_angles = [msg.data[3], msg.data[4], msg.data[5]]
  if redraw_flag:
    drawer_spatula.set_angles([0,0,0])
    drawer_spatula.set_desired_angles([x-d for (x,d) in zip(spatula_desired_angles, spatula_angles)])
    redraw_flag = False
    redraw()
    
    

def redraw():
  mrk = MarkerArray()
  mrk.markers.extend(drawer_baker.get_markers())
  mrk.markers.extend(drawer_spatula.get_markers())
  pub.publish(mrk)


pub = rospy.Publisher('/visualization_marker_array', MarkerArray)
rospy.Subscriber("/chi_f", Float64MultiArray, callback)
rospy.Subscriber("/chi_f_desired", Float64MultiArray, callback_desired)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
  redraw_flag = True
  rate.sleep()

