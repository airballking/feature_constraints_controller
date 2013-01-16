#!/usr/bin/python

from math import *
import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import tf_conversions.posemath as pm
import PyKDL as kdl

# for the legacy cylinder+rpy angles chain:
# publish the pose where the rpy-angle start to turn
# (this pose is given by the cylinder coordinates).
# This pose is used to display the discontinuity plot.


### initialization

chain_baker = kdl.Chain()

chain_baker.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ)));
chain_baker.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransX)));
chain_baker.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ),
                kdl.Frame(kdl.Rotation.RotX(pi/2)*kdl.Rotation.RotY(-pi/2))));

fksolver_baker = kdl.ChainFkSolverPos_recursive(chain_baker)

rospy.init_node('cylinder_pose')
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
  try:
    transform = tf_listener.lookupTransform('/object', '/tool', rospy.Time(0))
  except:
    rospy.sleep(0.1)
    continue

  frame = pm.fromTf(transform)

  p = frame.p;

  chi_f = kdl.JntArray(3);
  chi_f[0] = atan2(p.y(),p.x())
  chi_f[1] = sqrt(p.x()*p.x()+p.y()*p.y())
  chi_f[2] = p.z()

  baker_pose = kdl.Frame()
  fksolver_baker.JntToCart(chi_f, baker_pose)

  tf_broadcaster.sendTransform(baker_pose.p, baker_pose.M.GetQuaternion(), rospy.Time.now() + rospy.Duration(0.1), '/cylinder_pose', '/object')
  rate.sleep()
