# Task space feature constraints to be executed with the given kinematic chain.
FeatureConstraint[] constraints

# Sanity checks for the controller when accepting the command...
# ...names of all joints in the kinematic chain that shall be moved
string[] joint_names
# ...tf-frame of the base of the kinematic chain
string base_frame_id
# ...tf-frame of the tip of the kinematic chain
string tip_frame_id
