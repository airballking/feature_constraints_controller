#!/usr/bin/python

# moves a tf around according to a twist message

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as pm

from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion


rospy.init_node('tf_tree')

#tf_sender = tf.TransformBroadcaster()
tf_pub = rospy.Publisher('/tf', tfMessage)


def send_tree(tf_sender, frame, depth, step, frame_name):
  branch_factor = 4
  for i in range(branch_factor):
    R = kdl.Rotation.RotZ(i/float(branch_factor))
    p = R*kdl.Vector(step, 0, 0.1) # + frame.p
    tf_sender.sendTransform(p, R.GetQuaternion(),
                            rospy.Time.now(), frame_name, frame_name+str(i))
    if depth > 0:
      send_tree(tf_sender, F, depth-1, step/2.0, frame_name+str(i))



def get_tree(frame, depth, step, frame_name):
  branch_factor = 4

  tfs = []
  F = kdl.Frame()

  for i in range(branch_factor):
    F.M = kdl.Rotation.RotZ(2.0*pi*i/branch_factor)
    F.p = F.M*kdl.Vector(step, 0, 0.1) # + frame.p

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.transform.translation = Vector3(*F.p)
    msg.transform.rotation = Quaternion(*(F.M.GetQuaternion()))

    msg.header.frame_id = frame_name
    msg.child_frame_id = frame_name+str(i)

    tfs.append(msg)
    if depth > 0:
      tfs.extend(get_tree(F, depth-1, step/2.0, frame_name+str(i)))

  return tfs

rate = rospy.Rate(10)

while not rospy.is_shutdown():
  #send_tree(tf_sender, kdl.Frame(), 3, 0.4, 'fr')
  msg = tfMessage()
  msg.transforms = get_tree(kdl.Frame(), 2, 0.4, 'fr') 
  tf_pub.publish(msg)
  rate.sleep()
