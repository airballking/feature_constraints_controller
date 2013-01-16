#!/usr/bin/python
# the the constraints rank service of the feature_constraints package

import roslib
roslib.load_manifest('feature_constraints')

import sys

import unittest
import rospy
import rostest

from constraint_msgs.srv import ConstraintsRank
from constraint_msgs.msg import Constraint, Feature
from geometry_msgs.msg import Vector3


rospy.init_node('feature_rank_test')

class TestConstraintRanks(unittest.TestCase):

  def setUp(self):
    self.rank_srv = rospy.ServiceProxy('/constraints_rank', ConstraintsRank)
    self.request = ConstraintsRank._request_class()

    self.tool_center    = Feature('spatula', 'center', Feature.POINT,
                           Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0))
    self.tool_front     = Feature('spatula', 'front_edge', Feature.LINE,
                           Vector3(0,0,0.075), Vector3(0,0.1,0), Vector3(0,0,0))
    self.tool_center_right = Feature('spatula', 'center_right', Feature.LINE,
                              Vector3(0,0,0), Vector3(0,0.1,0), Vector3(0,0,0))
    self.tool_front_rev = Feature('spatula', 'front_edge_rev', Feature.LINE,
                           Vector3(0,0,0.075), Vector3(0,-0.1,0), Vector3(0,0,0))
    self.tool_side_left = Feature('spatula', 'side_edge', Feature.LINE,
                           Vector3(0,-0.05,0), Vector3(0,0,0.15), Vector3(0,0,0))
    self.tool_side_right = Feature('spatula', 'side_edge', Feature.LINE,
                            Vector3(0,0.05,0), Vector3(0,0,0.15), Vector3(0,0,0))
    self.tool_forward   = Feature('spatula', 'forward', Feature.LINE,
                           Vector3(0,0,0), Vector3(0,0,0.15), Vector3(0,0,0))
    self.tool_blade     = Feature('hand', 'blade', Feature.PLANE,
                           Vector3(0,0,0), Vector3(0.12,0,0), Vector3(0,0,0))
    self.up             = Feature('pancake', 'plane', Feature.PLANE,
                           Vector3(0,0,0), Vector3(0,0,0.3), Vector3(1,0,0))


  def test_6dof(self):
    c = []

    c.append(Constraint('angle', 'angle',
                          self.tool_center_right, self.up))
    c.append(Constraint('dist', 'distance',
                          self.tool_center, self.up))
    c.append(Constraint('height', 'height',
                          self.tool_center, self.up))

    c.append(Constraint('align_front', 'perpendicular',
                          self.tool_front_rev, self.up))
    c.append(Constraint('align_side',  'perpendicular',
                          self.tool_side_left, self.up))
    c.append(Constraint('pointing_at', 'pointing_at',
                          self.tool_forward,   self.up))

    self.request.constraints = c
    result = self.rank_srv(self.request)
    self.assertEqual(result.rank, 6)


  def test_2dependent(self):
    c = []

    c.append(Constraint('align_side_left',   'perpendicular', self.tool_side_left,  self.up))
    c.append(Constraint('align_side_right',  'perpendicular', self.tool_side_right, self.up))

    self.request.constraints = c
    result = self.rank_srv(self.request)
    self.assertEqual(result.rank, 1)

  def test_3dependent(self):
    c = []

    c.append(Constraint('align_front', 'perpendicular', self.tool_front,  self.up))
    c.append(Constraint('align_side',  'perpendicular', self.tool_side_left, self.up))
    c.append(Constraint('align_blade', 'perpendicular', self.tool_blade, self.up))

    self.request.constraints = c
    result = self.rank_srv(self.request)
    self.assertEqual(result.rank, 2)



if __name__ == '__main__':
  rostest.run('feature_constraints_test', 'rank_test',
              TestConstraintRanks, sys.argv)

