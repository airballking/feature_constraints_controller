#!/usr/bin/python
# the the constraints rank service of the feature_constraints package

import roslib
roslib.load_manifest('feature_constraints')

import sys

import unittest
import rospy
import rostest

import PyKDL as kdl

rospy.init_node('robot_test')

import robot

class TestRobot(unittest.TestCase):
  def setUp(self):
    self.robot = robot.Robot("calib_arm_arm_base_link", "arm_arm_7_link")

  def test_forward_kinematics(self):
    # random joints
    joints  = [( 0.689,  0.479,  0.049,  0.666,  0.816,  0.012,  0.895),
               (-0.706,  1.888, -0.893,  1.857,  2.425,  2.993,  1.308),
               ( 0.441, -0.984,  0.729,  1.060,  0.028, -0.511,  1.072),
               (-0.898, -0.354, -0.282,  2.245, -0.099,  1.719, -0.505),
               ( 1.718, -0.794, -0.921, -0.469,  0.725, -0.907,  0.447),
               ( 1.117,  0.922, -0.468,  1.483,  0.170,  0.181, -0.199)]

    # created with kdl_parser against kuka_lwr_arm.urdf.xacro
    results = [(( 0.035981,  0.082242,  0.935418,  0.341958),
                (-0.094037, -0.062164,  1.048050)),
               ((-0.289470,  0.878095,  0.173468,  0.339213),
                (-0.454364,  0.004407,  0.442485)),
               ((-0.187758, -0.878333, -0.437030,  0.047787),
                ( 0.475119,  0.474862,  0.425745)),
               (( 0.055407,  0.425327, -0.664491,  0.611946),
                ( 0.138564, -0.309946,  0.355384)),
               ((-0.502542,  0.403851,  0.619655,  0.447642),
                (-0.206100,  0.433057,  0.910329)),
               (( 0.087219,  0.274837,  0.401357,  0.869350),
                ( 0.097688, -0.199488,  0.848627))]

    for joint, result in zip(joints, results):
      ja = kdl.JntArray(7)
      for i in range(7):
        ja[i] = joint[i]


      expected_result = kdl.Frame(kdl.Rotation.Quaternion(*result[0]),
                                  kdl.Vector(*result[1]))

      actual_result = kdl.Frame()
      err = self.robot.kin_fwd.JntToCart(ja, actual_result)

      self.assertEqual(err, 0)
      self.assertTrue(kdl.Equal(expected_result, actual_result, 0.00001))


if __name__ == '__main__':
  rostest.run('robot_test', 'robot_test',
              TestRobot, sys.argv)
 
