#!/usr/bin/python

# visualize a task jacobian in rviz.
# requires:
# * the jacobian
# * the transform between the two objects

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as posemath
from std_msgs.msg import ColorRGBA, Float64MultiArray, Int8
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose,PoseStamped, Point, Quaternion, Vector3, Twist
from motion_viz.msg import Jacobian
from constraint_msgs.msg import ConstraintCommand


import marker

#TODO: move to marker.py
def color_code(index, intervals=[0.0, 0.5, 1.0],
                      colors=[[0.0, 1.0, 0.0, 1.0],
                              [1.0, 1.0, 0.0, 1.0],
                              [1.0, 0.0, 0.0, 1.0]]):

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





class VirtualChain:
  def __init__(self, name):
    self.name = name

    # retrieve frame names from parameters
    self.target_frame = rospy.get_param('~target_frame', '/base_link')
    self.ref_frame = rospy.get_param('~ref_frame', '/base_link')
    self.ref_point = rospy.get_param('~ref_point', self.ref_frame)

    rospy.loginfo('ref_frame=%s, ref_point=%s, target_frame=%s'
                   % (self.ref_frame, self.ref_point, self.target_frame))

    self.eps = 0.01
    self.rot_length = 0.06
    self.width = 0.02

    self.error_scales = (0.2, 0.2) # lin, rot
    self.errors = [0.0]*6
    self.weights = [0.0]*6

    self.pose = kdl.Frame()


  def set_pose(self, pose):
    ''' takes a Pose message '''
    self.pose = posemath.fromMsg(pose)


  def set_errors(self, errors):
    ''' set the current control errors for each jacobian direction. '''
    self.errors = errors


  def set_weights(self, weights):
    ''' set the weights for each jacobian direction. '''
    self.weights = weights


  def set_jac(self, jac):
    ''' takes a Jacobian message '''
    self.jac = []
    for t in jac.columns:
      twist = kdl.Twist(kdl.Vector(t.linear.x,  t.linear.y,  t.linear.z),
                        kdl.Vector(t.angular.x, t.angular.y, t.angular.z))
      self.jac.append(twist)


  def _compute_lengths(self):
    ''' compute the lenghts for the translational twists to make
        a closed chain.
    '''

    directions = []
    self.joint_types = []


    for i,twist in enumerate(self.jac):
      lr = twist.rot.Norm()
      lv = twist.vel.Norm()

      if lr < self.eps and lv >= self.eps:
        # got a linear twist, get its direction
        directions.append(twist.vel / lv)
        self.joint_types.append(1)
      else:
        self.joint_types.append(0)


    import numpy
    from numpy.linalg import lstsq
    # convert to matrix
    A = numpy.mat(map(lambda d: [d[0], d[1], d[2]], directions)).T
    b = numpy.mat([self.pose.p[0], self.pose.p[1], self.pose.p[2]]).T
    x,_,_,_ = lstsq(A,b) # now solve Ax = b
    self.lengths = x.T.tolist()[0]


  def markers(self):
    ''' get the markers for visualization '''
    self._compute_lengths()

    frame = self._transform()

    mrks = []
    pos = kdl.Vector(0,0,0)
    l_index = 0

    for i,t in enumerate(self.jac):
      twist = frame*t

      lr = twist.rot.Norm()
      lv = twist.vel.Norm()

      if lr < self.eps and lv < self.eps:
        # null twist, make a delete marker
        mrks.append(marker.create(id=i, ns=self.name, action=Marker.DELETE))
        continue

      if lr < self.eps:
        # pure translational twist
        location = pos
        direction = twist.vel / lv * self.lengths[l_index]

        m = marker.create(id=i, ns=self.name, type=Marker.CUBE)
        m = marker.align(m, location, location + direction, self.width)

        l_index += 1
        frame.p = frame.p - direction  # move target point to end of this 'axis'
        pos += direction               # remember current end point
      else:
        # rotational twist
        direction = twist.rot * self.rot_length / lr
        location = twist.rot * twist.vel / kdl.dot(twist.rot, twist.rot)  +  pos
        m = marker.create(id=i, ns=self.name, type=Marker.CYLINDER)
        m = marker.align(m, location - direction, location + direction, self.width)

      m.color = self._color(i)
      m.header.frame_id = self.target_frame
      mrks.append(m)
    return mrks


  def _color(self, index):
    err_scale = self.error_scales[self.joint_types[index]]
    c = color_code(abs(self.errors[index]) / err_scale)
    return color_code(self.weights[index], [0.0, 1.0], [[0.0, 0.0, 1.0, 1.0], [c.r, c.g, c.b, c.a]])
    

  def _transform(self):
    ''' uses tf to compute the transform for the twists '''
    try:
      (x,     rot)  = listener.lookupTransform(self.target_frame, self.ref_frame, rospy.Time(0))
      (trans, rotp) = listener.lookupTransform(self.target_frame, self.ref_point, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
      print 'tf exception!' + str(e)
      return kdl.Frame()
 
    # put the twist in the right frame
    return posemath.fromTf( (trans, rot) )


chi = None
active = True


def pose_callback(msg):
  ''' receive Pose of the chain end '''
  global chain
  chain.set_pose(msg.pose)


def jac_callback(msg):
  ''' receive jacobian '''
  global chain
  global active
  chain.set_jac(msg)
  for m in chain.markers():
    if not active:
      m.action = Marker.DELETE
    pub.publish(m)


def active_callback(msg):
  global active
  active = (msg.data != 0)


def chi_callback(msg):
  global chi
  chi = msg.data


def chi_des_callback(msg):
  global chain
  global chi

  if chi != None:
    errors = [c_des - c for c,c_des in zip(chi, msg.data)]
    chain.set_errors(errors)


def weights_callback(msg):
  global chain
  chain.set_weights(msg.data)


if __name__ == "__main__":
  global pub

  rospy.init_node('jac_viz')

  chain = VirtualChain(rospy.get_name())

  # set up connections
  listener = tf.TransformListener()
  pub = rospy.Publisher('/visualization_marker', Marker)
  sub_jac  = rospy.Subscriber('/jacobian', Jacobian, jac_callback)
  sub_pose = rospy.Subscriber('/pose', PoseStamped, pose_callback)

  # messy interface, to be moved into a a structured message
  sub_chi     = rospy.Subscriber('/chi_f', Float64MultiArray, chi_callback)
  sub_chi_des = rospy.Subscriber('/chi_f_desired', Float64MultiArray, chi_des_callback)
  sub_weights = rospy.Subscriber('/weights', Float64MultiArray, weights_callback)

  sub_active  = rospy.Subscriber('/itasc_active', Int8, active_callback)

  # run
  rospy.spin()