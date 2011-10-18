#ifndef OROCOS_PANCAKE_BAKING_COMPONENT_HPP
#define OROCOS_PANCAKE_BAKING_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <motion_viz/ConstraintCommand.h>
#include <motion_viz/ConstraintState.h>
#include <motion_viz/Jacobian.h>

#include "SubTask.hpp"


// ROS stuff
#include <ros/ros.h>

class FeatureTask
  : public iTaSC::SubTask
{
 public:
  FeatureTask(string const& name);
  ~FeatureTask();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook(){};
  void cleanupHook(){};


private:
  int start_count;
  int guard_time;

  KDL::Jacobian Jf_total;
  KDL::JntArray chi_f;
  KDL::JntArray ydot;
  std::vector<double> feedback_gain, weights, desired_values;

  Eigen::MatrixXd Wy;
  KDL::Frame pose;

  // custom named RTT ports
  std::vector< RTT::OutputPort<double>* > measured_ports;
  std::vector< RTT::InputPort<double>* > desired_ports;
  
  // ROS communication
  RTT::OutputPort<std_msgs::Float64MultiArray> ros_chi_f_port;
  RTT::OutputPort<std_msgs::Float64MultiArray> ros_chi_f_desired_port;
  RTT::InputPort<motion_viz::ConstraintCommand> ros_constraint_command_port;
  RTT::InputPort<std_msgs::Int8> ros_constraint_mode_port;
  RTT::InputPort<std_msgs::Int8> ros_constraint_select_port;
  RTT::OutputPort<motion_viz::ConstraintState> ros_constraint_state_port;

  RTT::OutputPort<geometry_msgs::PoseStamped> ros_o1o2_pose_port;

  // debugging and visualization
  RTT::OutputPort<geometry_msgs::Twist> ros_task_twist_port;
  RTT::OutputPort<motion_viz::Jacobian> ros_task_jacobian_port;
  RTT::OutputPort<std_msgs::Float64MultiArray> ros_weights_port;

  RTT::InputPort<std_msgs::Float64MultiArray> ros_chi_f_command_port;
  RTT::InputPort<std_msgs::Float64MultiArray> ros_weight_command_port;

  std_msgs::Float64MultiArray ros_chi_f;
  std_msgs::Float64MultiArray ros_chi_f_desired;
  std_msgs::Float64MultiArray ros_weights;
  std_msgs::Int8 ros_mode;    // 0: position mode 1: range mode
  std_msgs::Int8 ros_select;  // which constraint set to use: 0: cylinder+alignment

  geometry_msgs::Twist ros_task_twist;
  motion_viz::Jacobian ros_task_jacobian;

  motion_viz::ConstraintCommand ros_constraint_command;
  motion_viz::ConstraintState ros_constraint_state;
  std_msgs::Float64MultiArray ros_chi_f_command;
  std_msgs::Float64MultiArray ros_weight_command;

  void doControl();
  void doControl_ranges();

  // communication functions
  void RTT_init();
  void RTT_receive();
  void RTT_publish();

  void ROS_init();
  void ROS_receive();
  void ROS_publish();

  template <class P> void ROS_add_port(const std::string &rtt_name,
				       const std::string &ros_name,
				       P &port);

  // configuration
  std::string ros_prefix;
  std::vector<std::string> axis_names;


  //! requires at least 'nc' doubles to be in the array feature_values
  void compute_features(double *feature_values, KDL::Frame frame);
  void derive_features(KDL::Frame frame, double dd=0.001);
  KDL::Jacobian J_inv_t;
};

#endif

