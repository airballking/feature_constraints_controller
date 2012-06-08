#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <constraint_msgs/ConstraintConfig.h>
#include <constraint_msgs/ConstraintCommand.h>

#include <robot_kinematics.h>

#include <SolverWeighted.hpp>
#include <feature_constraints/Controller.h>
#include <feature_constraints/Conversions.h>

#include <Eigen/Core>

class FeatureConstraintsController
{
public:
  // non-realtime
  bool init(ros::NodeHandle& n);

  //realtime
  void update();  

private:
  // callback functions
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void feature_constraints_callback(const constraint_msgs::ConstraintConfig::ConstPtr& msg);
  void constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg);

  // these transforms will be looked up from tf
  KDL::Frame T_tool_in_ee_, T_object_in_world_, T_base_in_world_;

  // internal representation of robot state
  KDL::JntArray q_, qdot_;

  // internal representation of robot jacobian
  KDL::Jacobian jacobian_robot_;

  // used to calc kinematics of robot
  RobotKinematics *robot_kinematics_;

  // used to parse JointState msgs that come from the robot
  JointStateInterpreter *joint_state_interpreter_;

  // matrices used for talking to the solver
  Eigen::MatrixXd A_, Wq_, Wy_;

  // solver from KUL to inverse matrics
  SolverWeighted solver_;

  // object from Ingo  to translate feature constraints into interaction matrix
  Controller feature_controller_;

  // subscribers
  ros::Subscriber joint_state_subscriber_, feature_constraints_subscriber_, constraint_command_subscriber_;

  // publisher
  ros::Publisher qdot_publisher_;

  // corresponding message for publishing
  std_msgs::Float64MultiArray qdot_msg_;

  // flag to remember if feature_controller and solver have been resized meaningfully
  bool ready_;
};
