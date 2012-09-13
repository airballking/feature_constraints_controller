#!/usr/bin/python

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Vector3

feature = Feature('', '', Feature.POINT, Vector3(0,0,0),
                                         Vector3(0,0,0), Vector3(0,0,0))
c = []

c.append(Constraint('angle',  'chain0', feature, feature))
c.append(Constraint('dist',   'chain1', feature, feature))
c.append(Constraint('height', 'chain2', feature, feature))

c.append(Constraint('align_front', 'chain3', feature, feature))
c.append(Constraint('align_side',  'chain4', feature, feature))
c.append(Constraint('pointing_at', 'chain5', feature, feature))

rospy.init_node('constraint_config')
pub = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
pub.publish(ConstraintConfig(c))

rospy.spin()

