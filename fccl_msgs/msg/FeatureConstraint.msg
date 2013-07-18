# Task space constraints are the main means of expressing desired motion goals
# in the feature constraints controller. The setup the space of the constraint
# by relating the geometric features using a feature function which is evaluated
# w.r.t. to an external frame. Actual constraint ranges are then defined over
# output of those feature functions.
 
# name of id assigned by the knowledge base
string name

# feature function used to relate the two features
uint8 above=1
uint8 below=2
uint8 left=3
uint8 right=4
uint8 infront=5
uint8 behind=6
uint8 distance=7
uint8 pointing_at=8
uint8 perpendicular=9
uint8 projected_pointing_at=10
uint8 projected_distance=11
uint8 projected_height=12

uint8 feature_function 

# two features related to another...
# ...feature on the tool controlled
Feature tool_feature
# ...feature on the external object
Feature world_feature

# tf-frame w.r.t. which the feature function is evaluated:
# This allows expression of view-point-dependent constraints such as
# 'have the tip of your tool right of the object, seen from your perspective'.
string view_frame

# actual constraint range
Constraint constraint
