#!/usr/bin/python

# calls an Empty service whenever it receives a message.

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('std_srvs')

import rospy
from std_srvs.srv import Empty

rospy.init_node('msg2service')

delay = rospy.get_param('~delay', 0.1)

def callback(msg):
  print 'trigger'
  rospy.sleep(delay)
  rpc() # call service

rpc = rospy.ServiceProxy('~/service', Empty)
sub = rospy.Subscriber('~/topic', rospy.msg.AnyMsg, callback)

rospy.spin()
