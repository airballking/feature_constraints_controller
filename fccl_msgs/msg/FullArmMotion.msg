# Complete specififcation of a typical robotic arm, e.g. PR2 arm, KUKA LWR,
# Barrett WAM, etc., specifying main feature constraints task for the end-
# effector and then two means of redundancy resolution: Another feature constraints
# task for the 'elbow' of the arm and a joint-space motion.

# Redundancy resolution is done by Nullspace projection. By omitting tasks
# which are higher up in the hierarchy one can make the lower tasks main task.

# Primary kinematic chain to be controlled:
FeatureConstraintMotion end_effector_motion

# Usually, a sub-chain of the above kinematic chain used for
# redundancy resolution
FeatureConstraintMotion elbow_motion

# Final redundancy resolution schema
JointConstraintMotion joint_motion
