#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <constraint_msgs/ConstraintConfig.h>
#include <constraint_msgs/ConstraintCommand.h>
#include <constraint_msgs/JointAvoidanceState.h>

#include <robot_kinematics.h>

#include <SolverWeighted.hpp>
#include <feature_constraints/Controller.h>
#include <feature_constraints/Conversions.h>
#include <feature_constraints/JointLimitAvoidanceController.h>

#include <Eigen/Core>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//! A feature-grounded constraint controller
/*! This class communicates to ROS. It receives constraint configurations and
 *  -commands, tf frames (not yet!) and joint states. It feeds them to the
 *  task-function based \ref feature_controller_, a WDLS solver and sends
 *  joint velocities back to the robot.
 */
class FeatureConstraintsController
{
public:
  FeatureConstraintsController();
  ~FeatureConstraintsController();
  // non-realtime
  bool init(ros::NodeHandle& n);  //!< initialize controller, solver and ROS communication

  // realtime
  void update(double dt);

private:
  // callback functions

  //! Receives new joint angles.
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

  //! Receives a new constraint configuration.
  void feature_constraints_callback(const constraint_msgs::ConstraintConfig::ConstPtr& msg);

  //! Receives the desired ranges for the constraints.
  void constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg);

  // Retrieves the names of the relevant tf-frames from the parameter server during init
  // returns false if some parameter could not be loaded
  bool load_frame_names(ros::NodeHandle& n);

  // Receives all four of the below transforms from tf
  void tf_poses_lookup();
  // looks up the relevant transforms from tf
  bool lookup_tf_frames();

  // our NodeHandle received in init()
  ros::NodeHandle n_;

  //! These transforms will be looked up from tf
  KDL::Frame T_tool_in_ee_, T_object_in_world_, T_base_in_world_, T_arm_in_base_;
  std::string robot_base_frame_, world_frame_, arm_base_frame_, arm_ee_frame_, tool_frame_, object_frame_;
  
  // tf interfacing
  tf::TransformListener tf_listener_;
  tf::StampedTransform aux_transform_;

  // mutex to protect look ups from tf
  boost::mutex tf_lookup_mutex_;
  // thread that does the tf lookups
  boost::thread *tf_lookup_thread_;
  // flag to tell the tf lookup thread to pause and stop
  bool stop_tf_lookup_, pause_tf_lookup_;

  //! Internal representation of robot state
  KDL::JntArray q_, qdot_;

  //! Internal representation of robot jacobian
  KDL::Jacobian jacobian_robot_;

  //! Calculate kinematics of robot
  RobotKinematics *robot_kinematics_;

  //! Parses JointState messages that come from the robot
  JointStateInterpreter *joint_state_interpreter_;

  //! Matrices used for talking with the feature controller
  Eigen::MatrixXd H_feature_;

  //! Matrices used for talking to the solver and Nullspace projection
  Eigen::MatrixXd A_, A_inv_, ydot_, Wy_, Wq_, Identity_joints_;

  //! WDLS solver from KUL to inverse matrix
  SolverWeighted solver_;

  //! Controller component to translate feature constraints into interaction matrix and output velocities
  Controller feature_controller_;

  //! The feature controller uses filters to smooth the estimated constraint velocities.
  //  The ROS filter_chain that is used for this expects the filter parameters on the parameter server.
  //  This is the namespace under which the filter parameters can be found.
  std::string filter_namespace_;

  //! Controller component that avoids joint limits
  JointLimitAvoidanceController joint_limit_avoidance_controller_;

  //! Subscribers
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber feature_constraints_subscriber_;
  ros::Subscriber constraint_command_subscriber_;

  //! Publishers
  ros::Publisher qdot_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher limit_avoidance_state_publisher_;
  //! Corresponding message for publishing
  std_msgs::Float64MultiArray qdot_msg_;
  constraint_msgs::ConstraintState state_msg_;
  constraint_msgs::JointAvoidanceState joint_avoidance_state_msg_;

  // flags to remember the state of the feature controller
  bool configured_, started_;

  // flag to turn joint limit avoidance on and off
  // will be set once during init
  bool joint_limit_avoidance_on_;

  // slot to remember last time we were called to calc time between control cycles
  ros::Time last_time_;
};

