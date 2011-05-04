# one segment of a kinematic chain

byte FIXED=0
byte TRANSLATIONAL=1
byte ROTATIONAL=2
byte RPY=3
byte BALL=4

byte type  # type of the joint (FIXED, ROTATIONAL or TRANSLATIONAL)
string name
geometry_msgs/Transform link  # transform to the next joint
geometry_msgs/Vector3 axis  # direction of the actuated axis
