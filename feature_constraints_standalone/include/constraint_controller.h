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


//! A feature-grounded constraint controller
/*! This class communicates to ROS. It receives constraint configurations and
 *  -commands, tf frames (not yet!) and joint states. It feeds them to the
 *  task-function based \ref feature_controller_, a WDLS solver and sends
 *  joint velocities back to the robot.
 */
class FeatureConstraintsController
{
public:
  // non-realtime
  bool init(ros::NodeHandle& n);  //!< initialize controller, solver and ROS communication

  // realtime
  void update();

private:
  // callback functions

  //! Receives new joint angles.
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

  //! Receives a new constraint configuration.
  void feature_constraints_callback(const constraint_msgs::ConstraintConfig::ConstPtr& msg);

  //! Receives the desired ranges for the constraints.
  void constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg);

  //! Receives the offset of the tool from the robots flange
  void tool_offset_callback(const geometry_msgs::Pose::ConstPtr& msg);

  //! Receives the offset of the object from the world's origin
  void object_offset_callback(const geometry_msgs::Pose::ConstPtr& msg);

  //! These transforms will be looked up from tf
  KDL::Frame T_tool_in_ee_, T_object_in_world_, T_base_in_world_;

  //! Internal representation of robot state
  KDL::JntArray q_, qdot_;

  //! Internal representation of robot jacobian
  KDL::Jacobian jacobian_robot_;

  //! Calculate kinematics of robot
  RobotKinematics *robot_kinematics_;

  //! Parses JointState messages that come from the robot
  JointStateInterpreter *joint_state_interpreter_;

  //! Matrices used for talking to the solver
  Eigen::MatrixXd A_, Wq_, Wy_, H_;

  //! WDLS solver from KUL to inverse matrix
  SolverWeighted solver_;

  //! Controller component to translate feature constraints into interaction matrix and output velocities
  Controller feature_controller_;

  //! Subscribers
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber feature_constraints_subscriber_;
  ros::Subscriber constraint_command_subscriber_;
  ros::Subscriber tool_offset_subscriber_;
  ros::Subscriber object_offset_subscriber_;


  //! Publishers
  ros::Publisher qdot_publisher_;
  ros::Publisher state_publisher_;
  //! Corresponding message for publishing
  std_msgs::Float64MultiArray qdot_msg_;
  constraint_msgs::ConstraintState state_msg_;


  //! Flag to remember if feature_controller_ and solver_ have been resized meaningfully
  // flags to remember the state of the feature controller
  bool configured_, started_;
};

