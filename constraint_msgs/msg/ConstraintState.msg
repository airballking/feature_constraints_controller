# The current state of a kinematic chain (joint angles)
Header header
string name

geometry_msgs/Pose base_pose
geometry_msgs/Pose pose

string[] joint_names

float64[] chi
float64[] chi_desired
float64[] weights

geometry_msgs/Twist[] jacobian
float64[] singular_values
