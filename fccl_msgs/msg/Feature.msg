# Geometric features are the basic building block in the feature constraints
# controller library. They can be regarded as point features with varying
# orientation information.

# tf-frame in which position and orientation are expressed
string frame_id

# name or id assigned by the knowledge base
string name

# type of the control feature
uint8 LINE=1
uint8 PLANE=2
uint8 POINT=3
uint8 type

# geometric properties of the feature...
# ...origin of the feature
geometry_msgs/Vector3 position  
# ... 3D direction of the feature:
# (a) ignored for point features
# (b) direction along line for line features
# (c) normal for plane features
geometry_msgs/Vector3 direction
