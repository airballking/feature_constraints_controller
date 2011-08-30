# one segment of a kinematic chain

byte FIXED=0
byte TRANSLATIONAL=1
byte ROTATIONAL=2
#byte RPY=3
#byte BALL=4

string name
byte type  # type of the joint (FIXED, ROTATIONAL or TRANSLATIONAL)
int32 pose_index # at which pose (transmitted in the status message does this frame start? -1 means proceed in the chain)
geometry_msgs/Vector3 axis  # direction of the actuated axis
