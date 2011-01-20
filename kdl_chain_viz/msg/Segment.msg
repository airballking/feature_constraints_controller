# one segment of a kinematic chain

byte FIXED=0
byte ROTATIONAL=1
byte TRANSLATIONAL=2

byte type  # type of the joint (FIXED, ROTATIONAL or TRANSLATIONAL)
geometry_msgs/Transform fixed_transform  # transform to the next joint
geometry_msgs/Vector3 axis_direction  # direction of the actuated axis
