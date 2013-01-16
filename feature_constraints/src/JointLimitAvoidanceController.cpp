#include <feature_constraints/JointLimitAvoidanceController.h>

#include <ros/ros.h>

void JointLimitAvoidanceController::prepare(const std::vector<std::string>& joint_names,
  const std::vector<double>& lower_limits, const std::vector<double>& upper_limits)
{
  // sanity check of input
  assert(joint_names.size() == lower_limits.size());
  assert(lower_limits.size() == upper_limits.size());
  assert(joint_names.size() > 0);

  // resize internal data structures
  unsigned int dof = joint_names.size();
  joint_names_.resize(dof);
  q_.resize(dof);
  command_.resize(dof);
  gains_.resize(dof);
  q_desired_.resize(dof);
  q_dot_desired_.resize(dof);
  weights_.resize(dof);

  // copy input information
  joint_names_ = joint_names;

  // set gains to standard values
  for(unsigned int i=0; i<dof; i++)
  {
    gains_(i) = 1.0;
  } 

  // adjust command according to read joint limits
  for(unsigned int i=0; i<dof; i++)
  {
    if((lower_limits[i]==upper_limits[i]) && (lower_limits[i]==0.0))
    {
      // the provided values indicate: no joint limit
      ROS_WARN("[JointLimitAvoidanceController] Joint '%s' is interpreted to be without limits!",
        joint_names[i].c_str());
      command_.weight(i) = 0.0;
    }
    else
    {
      command_.weight(i) = 1.0;
      command_.pos_lo(i) = lower_limits[i];
      command_.pos_hi(i) = upper_limits[i];
    }
  } 
}

void JointLimitAvoidanceController::update(const KDL::JntArray& q)
{
  assert(q.rows() == q_.rows());
  
  // get the new joint values inside
  q_.data = q.data;

  // do the range-based control which will update the values of
  // q_dot_desired_, weights_ and q_desired_
  control(q_dot_desired_, weights_, q_desired_, q, command_, gains_);
}
