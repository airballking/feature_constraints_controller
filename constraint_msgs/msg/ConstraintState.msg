# The current state of a kinematic chain (joint angles)
Header header
string name

geometry_msgs/Pose base_pose # pose of object w.r.t. header.frame_id
geometry_msgs/Pose pose      # pose of tool w.r.t. base_pose

string[] joint_names

float64[] chi                # current constraint values
float64[] chi_desired        # desired constraint values
float64[] weights            # actual weights of constraints

geometry_msgs/Twist[] interaction_matrix  # H
geometry_msgs/Twist[] jacobian            # H^+
float64[] singular_values                 # from the svd while computing H^+

