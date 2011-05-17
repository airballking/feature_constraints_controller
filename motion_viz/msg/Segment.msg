# one segment of a kinematic chain

byte FIXED=0
byte TRANSLATIONAL=1
byte ROTATIONAL=2
#byte RPY=3
#byte BALL=4

string name
byte type  # type of the joint (FIXED, ROTATIONAL or TRANSLATIONAL)
string frame_id # used in combination witch reset_transform to start a second chain piece
bool reset_transform  # used to show special rotation axes
geometry_msgs/Transform link  # transform to the next joint
geometry_msgs/Vector3 axis  # direction of the actuated axis
