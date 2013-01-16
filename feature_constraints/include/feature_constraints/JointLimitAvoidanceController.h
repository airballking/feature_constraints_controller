#ifndef FEATURE_CONSTRAINTS_JOINT_LIMIT_AVOIDANCE_CONTROLLER_H
#define FEATURE_CONSTRAINTS_JOINT_LIMIT_AVOIDANCE_CONTROLLER_H

#include <feature_constraints/Controller.h>

#include <kdl/jntarray.hpp>

#include <vector>
#include <string>

class JointLimitAvoidanceController
{
public:
  // NOT REALTIME-SAFE
  void prepare(const std::vector<std::string>& joint_names, 
    const std::vector<double>& lower_limits, const std::vector<double>& upper_limits);

  // REALTIME-SAFE
  void update(const KDL::JntArray& q);

  //! Names of the joints this controller is working on
  std::vector<std::string> joint_names_;

  // INPUT OF THE CONTROLLER...
  // ...Internal representation of robot state
  KDL::JntArray q_;

  //  ...Internal representation of the joint limit avoidance command
  //  Note: A joint that is limitless or for joint limit avoidance should
  //        deactivated needs to have a weight of 0.0 in the command. Please
  //        also note that these are the weights of the constraints on the
  //        joints not the joint weights themselves.
  Ranges command_;

  // ...Gains of the controller
  KDL::JntArray gains_;

  // OUTPUT OF THE CONTROLLER...
  // ... goal within the specified ranges
  KDL::JntArray q_desired_;

  // ... desired instantaneous joint velocity to avoid the joint limits
  KDL::JntArray q_dot_desired_;

  // weights for these constraints to be given to the WDLS solver as Wy
  KDL::JntArray weights_;
};
#endif //FEATURE_CONSTRAINTS_JOINT_LIMIT_AVOIDANCE_CONTROLLER_H

