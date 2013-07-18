# The feature constraints controllers supports the definition of joint constraints
# for further redundancy resolution. This is the message communicating the 
# desired joint-space motion of the arm in the Nullspace of the task space
# task.

# Joint Constraints to be executed on the given joints.
JointConstraint[] constraints

# Sanity check for the controller when accepting the command...
string[] joint_names
