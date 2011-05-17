#!/usr/bin/python

# This program reads a transform from tf and republishes it
# as a PoseStamped message.

import sys

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
from geometry_msgs.msg import PoseStamped,Point,Quaternion

rospy.init_node('tf2pose')
l = tf.TransformListener()

if len(sys.argv) >= 4:
  target_frame = sys.argv[1]
  base_frame = sys.argv[2]
  topic = sys.argv[3]
else:
  print 'usage: %s target_frame base_frame topic_name' % sys.argv[0]
  sys.exit()

pub = rospy.Publisher(topic, PoseStamped)

rate = rospy.Rate(10)

l.waitForTransform(base_frame, target_frame, rospy.Time(), rospy.Duration(10))

while not rospy.is_shutdown():
  transform = l.lookupTransform(base_frame, target_frame, rospy.Time(0))
  p = PoseStamped()
  p.header.stamp = rospy.Time.now()
  p.header.frame_id = base_frame
  p.pose.position = Point(*transform[0])
  p.pose.orientation = Quaternion(*transform[1])
  pub.publish(p)
  rate.sleep()

