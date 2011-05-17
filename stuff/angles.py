#!/usr/bin/python

# develop useful angle representation

from math import *

import roslib
roslib.load_manifest('force_viz')
import rospy
import tf
import PyKDL as kdl
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Twist
from kdl_chain_viz.msg import Jacobian

def get_frame():
  try:
    (trans, rot)  = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException):
    print 'tf exception'
    return kdl.Frame()
  return kdl.Frame(kdl.Rotation.Quaternion(*rot), kdl.Vector(*trans))


def compute_angles(frame):
  """ compute task angles for the given frame """
  rx = frame.M.UnitX()
  ry = frame.M.UnitY()
  rz = frame.M.UnitZ()

  a0 = kdl.dot(ry, kdl.Vector(1,0,0)) # front edge aliged <=> a0 == 0
  a1 = kdl.dot(rz, kdl.Vector(1,0,0)) # side edge aligned <=> a1 == 0

  # tool direction:
  # * look from above -> project rz onto y-z-plane , yields rzp
  # * take angle with z
  # (tool direction towards center <=> a3 == 1)

  rzp = kdl.Vector(0, rz.y(), rz.z())
  a3 = kdl.dot(rzp, kdl.Vector(0,0,1)) / rzp.Norm()

  return (a0, a1, a3)


def derive_angles(frame, dd=0.01):
  """ numerically derive the task angle function around the given frame.
      Also computes the inverse: this yields the instantaneous rotation axes
      for each angle.
  """
  a0 = kdl.Vector(*compute_angles(frame))
  ax = kdl.Vector(*compute_angles(frame*kdl.Frame(kdl.Rotation.RotX(dd))))
  ay = kdl.Vector(*compute_angles(frame*kdl.Frame(kdl.Rotation.RotY(dd))))
  az = kdl.Vector(*compute_angles(frame*kdl.Frame(kdl.Rotation.RotZ(dd))))

  dx = (ax - a0) / dd
  dy = (ay - a0) / dd
  dz = (az - a0) / dd

  import numpy
  import numpy.linalg

  jac_inv = numpy.matrix([[dx.x(), dy.x(), dz.x()],
                          [dx.y(), dy.y(), dz.y()],
                          [dx.z(), dy.z(), dz.z()]])

  # the columns of jac are three twists to be displayed
  jac = numpy.linalg.inv(jac_inv)

  return (jac_inv, jac)

def publish_jacobian(jac):
  """ publishes the given rotational jacobion """
  msg = Jacobian()
  msg.columns.append(Twist())
  msg.columns.append(Twist())
  msg.columns.append(Twist())
  for col in jac.T:
    t = Twist()
    t.angular.x = col[0,0]
    t.angular.y = col[0,1]
    t.angular.z = col[0,2]
    msg.columns.append(t)
  jac_pub.publish(msg)


rospy.init_node('angles_dev')
listener = tf.TransformListener()
jac_pub = rospy.Publisher('rot_jac', Jacobian)

source_frame = rospy.get_param('~source_frame', '/base_link')
target_frame = rospy.get_param('~target_frame', '/frame')

rate = rospy.Rate(1)

while not rospy.is_shutdown():
  frame = get_frame()
  print frame
  angles = compute_angles(frame)
  (jac_inv, jac) = derive_angles(frame)
  print jac
  print jac_inv
  print 'a0,a1,a2 = '+str(angles)
  publish_jacobian(jac)
  

  rate.sleep()
