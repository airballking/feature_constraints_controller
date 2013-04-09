#!/usr/bin/python

# Display a manipulability ellipsoid in rviz.
# this time, use the urdf and joint_state for getting the kinematic chain
# and the current joint angles.

import roslib
roslib.load_manifest('stuff')

import rospy
import marker

import PyKDL as kdl

from geometry_msgs.msg import Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA

import robot
import numpy
from numpy.linalg import svd,det
from math import *

def get_ellipsoid(F, J, rotational=True):
  JJt = J*J.transpose()
  if not rotational:
    JJt3 = JJt[0:3, 0:3]
  else:
    JJt3 = JJt[3:6, 3:6]
  u,ss,vh = svd(JJt3)

  # ss: scales
  # u: rotation matrix (make sure it is right-handed)

  if det(u) < 0:
    u[2,:] = -u[2,:]  # turn around last column

  # now make it a quaternion
  q = kdl.Rotation(*[x for row in u.tolist() for x in row]).GetQuaternion()
  # compute ellipsoid extensions
  s = [sqrt(x) for x in ss.tolist()]
  # compute condition number k
  k = sqrt(s[2]/s[0])
  # robot arm's position
  p = [F.p[0], F.p[1], F.p[2]]

  return (p,q,s,k)


class ManipulabilityEllipsoidDisplay:
  def __init__(self, base_frame_id, tip_frame_id, scale=1.0):
    self.base_frame_id = base_frame_id
    self.tip_frame_id = tip_frame_id
    self.scale = scale
    self.robot = robot.Robot(base_frame_id, tip_frame_id)

  def show(self):
    jac = self.robot.jac()
    J = numpy.matrix([[jac[i,j] for j in range(jac.columns())]
                                  for i in range(jac.rows())])
    F = self.robot.pose()
    (p,q,s,k) = get_ellipsoid(F,J)

    c = marker.color_code(k)

    m = marker.create(type=marker.Marker.SPHERE, ns='ellipsoid', id=42)
    m.header.frame_id = self.base_frame_id
    m.header.stamp = rospy.Time.now()
    m.pose.position = Point(x=p[0], y=p[1], z=p[2])
    m.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
    m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=k)
    m.scale = Vector3(x=s[0]*self.scale, y=s[1]*self.scale, z=s[2]*self.scale)

    marker.publish(m)


def main():
  rospy.init_node('manipulability_ellipsoid')

  base_link = rospy.get_param('~base_link', '')
  tip_link  = rospy.get_param('~tip_link', '')
  scale     = rospy.get_param('~scale', 1.0)

  if base_link == '' or tip_link == '':
    rospy.logerr('must specify base_link and tip_link!')
    return

  disp = ManipulabilityEllipsoidDisplay(base_link, tip_link, scale)

  rate = rospy.Rate(25)

  while not rospy.is_shutdown():
    disp.show()
    rate.sleep()

if __name__ == "__main__":
  main()
