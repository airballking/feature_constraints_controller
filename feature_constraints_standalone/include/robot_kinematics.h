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


/*! \file robot_kinematics.h
 *
 *  This file contains two helper classes to compute the (position-
 *  and velocity-) forward kinematics. The classes do make use use ROS
 *  libraries, but are passive, i.e. they have no threads or callbacks.
 *  This should make it easier to reuse them.
 */

//! Robot kinematics helper using ROS
/*! - gets the kinematic chain from the URDF
 *  - configured by ROS parameters 'tool_frame' and 'base_frame'.
      (TODO: is this flexible enough?)
 *  - computes the forward kinematics
 *  - computes the Jacobian
 */
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


//! A realtime-safe joint state reader.
/*! Assumptions:
 *    - The joint names must appear in one JointState message
 *    - The order of the names must not change
 *    - ('holes' are allowed)
 *
 *  If the joint states were distributed over several messages,
 *  then a collection of JointStateInterpreters is required. 
 */
class JointStateInterpreter
{
public:
  JointStateInterpreter(std::vector<std::string> joint_names);
  bool parseJointState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q);
private:
  std::vector<std::string> joint_names_;
};

