#!/usr/bin/python

# Three dependent constraints (left side, front side, blade).
# This configuration is meant for testing the singularity tester.

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Vector3

tool_center    = Feature('spatula', 'center', Feature.POINT,
                   Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0))
tool_front     = Feature('spatula', 'front_edge', Feature.LINE,
                   Vector3(0,0,0.075), Vector3(0,0.1,0), Vector3(0,0,0))
tool_center_right = Feature('spatula', 'center_right', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0.1,0), Vector3(0,0,0))
tool_front_rev = Feature('spatula', 'front_edge_rev', Feature.LINE,
                   Vector3(0,0,0.075), Vector3(0,-0.1,0), Vector3(0,0,0))
tool_side_left = Feature('spatula', 'side_edge', Feature.LINE,
                   Vector3(0,-0.05,0), Vector3(0,0,0.15), Vector3(0,0,0))
tool_side_right = Feature('spatula', 'side_edge', Feature.LINE,
                   Vector3(0,0.05,0), Vector3(0,0,0.15), Vector3(0,0,0))
tool_forward   = Feature('spatula', 'forward', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0,0.15), Vector3(0,0,0))
tool_blade     = Feature('spatula', 'blade', Feature.PLANE,
                   Vector3(0,0,0), Vector3(0.12,0,0), Vector3(0,0,0))
up             = Feature('pancake', 'plane', Feature.PLANE,
                   Vector3(0,0,0), Vector3(0,0,0.3), Vector3(1,0,0))

c = []

c.append(Constraint('align_front',   'perpendicular', tool_front,  up))
c.append(Constraint('align_side',  'perpendicular', tool_side_left, up))
c.append(Constraint('align_blade',  'perpendicular', tool_blade, up))

rospy.init_node('constraint_config')
pub = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
pub.publish(ConstraintConfig(c))

rospy.spin()

