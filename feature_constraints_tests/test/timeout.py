#!/usr/bin/python

# Test if another node elapses a timeout.
#
# The start and stop signals are messages on a String topic.
# Multiple test periods are supported.
# This test is configured through rosparams.


import roslib
roslib.load_manifest('feature_constraints')

import sys

import unittest
import rospy
import rostest

from std_msgs.msg import String

class TestTimeout(unittest.TestCase):

  def state_cb(self, msg):
    if msg.data == self.start_marker:
      self.start_time = rospy.Time.now()
    elif msg.data == self.finish_marker:
      self.start_time = rospy.Time(0)
      self.periods += 1

  def test_timeout(self):
    rospy.init_node('test_timeout')

    self.time_limit = rospy.Duration(rospy.get_param('~duration_limit', 30.0))
    self.start_marker = rospy.get_param('~start_marker', 'start')
    self.finish_marker = rospy.get_param('~finish_marker', 'finish')
    self.num_periods = rospy.get_param('~num_periods', 1)

    self.start_time = rospy.Time(0)
    self.periods = 0

    self.subscriber = rospy.Subscriber('/state', String, self.state_cb)

    while True:
      if ((not self.start_time.is_zero())
          and rospy.Time.now() > self.start_time + self.time_limit):
        self.fail('time limit elapsed')
      if self.periods == self.num_periods:
        break  # success
      rospy.sleep(0.1)

if __name__ == '__main__':
  rostest.run('feature_constraints_test', 'time_limit_test',
              TestTimeout, sys.argv)

