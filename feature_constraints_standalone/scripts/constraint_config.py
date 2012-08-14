#!/usr/bin/python

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Vector3

tool_front     = Feature('hand', 'front_edge', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,1,0), Vector3(0,0,0))
tool_front_rev = Feature('hand', 'front_edge_rev', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,-1,0), Vector3(0,0,0))
tool_side      = Feature('hand', 'side_edge', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0,1), Vector3(0,0,0))
up             = Feature('hand', 'side_edge', Feature.LINE,
                   Vector3(0,0,0), Vector3(0,0,1), Vector3(1,0,0))

c = []

c.append(Constraint('angle', 'angle',   tool_front, up))
c.append(Constraint('dist', 'distance', tool_front, up))
c.append(Constraint('height', 'height', tool_front, up))

c.append(Constraint('align_front', 'perpendicular', tool_front_rev, up))
c.append(Constraint('align_side', 'perpendicular',  tool_side,      up))
c.append(Constraint('pointing_at', 'pointing_at',   tool_front,     up))

rospy.init_node('constraint_config')
pub = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
pub.publish(ConstraintConfig(c))

rospy.spin()

