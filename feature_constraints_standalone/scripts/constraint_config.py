#!/usr/bin/python

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Vector3

tool_center    = Feature('hand', 'center', Feature.POINT,
                   Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0))
tool_front     = Feature('hand', 'front_edge', Feature.LINE,
                   Vector3(0,0,0.075), Vector3(0,0.1,0), Vector3(0,0,0))
tool_center_right = Feature('hand', 'center_right', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0.1,0), Vector3(0,0,0))
tool_front_rev = Feature('hand', 'front_edge_rev', Feature.LINE,
                   Vector3(0,0,0.075), Vector3(0,-0.1,0), Vector3(0,0,0))
tool_side      = Feature('hand', 'side_edge', Feature.LINE,
                   Vector3(0,-0.05,0), Vector3(0,0,0.15), Vector3(0,0,0))
tool_forward   = Feature('hand', 'forward', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0,0.15), Vector3(0,0,0))
up             = Feature('baker', 'side_edge', Feature.PLANE,
                   Vector3(0,0,0), Vector3(0,0,0.3), Vector3(1,0,0))

c = []

c.append(Constraint('angle', 'angle',   tool_center_right, up))
c.append(Constraint('dist', 'distance', tool_center, up))
c.append(Constraint('height', 'height', tool_center, up))

c.append(Constraint('align_front', 'perpendicular', tool_front_rev, up))
c.append(Constraint('align_side',  'perpendicular', tool_side,      up))
c.append(Constraint('pointing_at', 'pointing_at',   tool_forward,   up))

rospy.init_node('constraint_config')
pub = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
pub.publish(ConstraintConfig(c))

rospy.spin()

