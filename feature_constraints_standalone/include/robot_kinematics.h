#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class RobotKinematics
{
public:
  //non-realtime
  bool init(ros::NodeHandle& n);
  std::vector<std::string> getJointNames();
  
  // real-time
  void getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac);
  void getForwardKinematics(const KDL::JntArray& q, KDL::Frame& frame);
  unsigned int getNumberOfJoints();

private:
  std::string tool_frame_id_;
  std::string base_frame_id_;
  KDL::Chain chain_;
  KDL::ChainFkSolverPos_recursive* jnt_to_pose_solver_;
  KDL::ChainJntToJacSolver* jnt_to_jac_solver_; 
};


// Assumption: The joint names must appear in the same order
// in one message. ('holes' are allowed)
class JointStateInterpreter
{
public:
  JointStateInterpreter(std::vector<std::string> joint_names);
  bool parseJointState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q);
private:
  std::vector<std::string> joint_names_;

};
