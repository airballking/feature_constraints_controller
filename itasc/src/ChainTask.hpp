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

#include "SubTask.hpp"


// ROS stuff
#include <ros/ros.h>

class ChainTask
  : public iTaSC::SubTask
{
 public:
  ChainTask(string const& name);
  ~ChainTask();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook(){};
  void cleanupHook(){};


private:
  bool pose_valid;
  int start_count;

  KDL::Chain chain_baker;
  KDL::Chain chain_spatula;
  KDL::ChainFkSolverPos_recursive* fksolver_baker;
  KDL::ChainJntToJacSolver* jacsolver_baker;
  KDL::ChainFkSolverPos_recursive* fksolver_spatula;
  KDL::ChainJntToJacSolver* jacsolver_spatula;

  KDL::Jacobian Jf_total,Jf_baker,Jf_spatula;
  KDL::JntArray chi_f_baker;
  KDL::JntArray chi_f_spatula, chi_f_spatula_init;
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

  RTT::OutputPort<geometry_msgs::PoseStamped> ros_chain_pose_port;
  RTT::OutputPort<geometry_msgs::PoseStamped> ros_o1o2_pose_port;
  RTT::OutputPort<geometry_msgs::PoseStamped> ros_desired_pose_port;
  RTT::OutputPort<geometry_msgs::Twist> ros_task_twist_port;
  RTT::OutputPort<std_msgs::Float64MultiArray> ros_weights_port;

  RTT::InputPort<std_msgs::Float64MultiArray> ros_chi_f_command_port;
  RTT::InputPort<std_msgs::Float64MultiArray> ros_weight_command_port;

  std_msgs::Float64MultiArray ros_chi_f;
  std_msgs::Float64MultiArray ros_chi_f_desired;
  std_msgs::Float64MultiArray ros_weights;
  std_msgs::Int8 ros_mode;

  geometry_msgs::Twist ros_task_twist;

  motion_viz::ConstraintCommand ros_constraint_command;
  std_msgs::Float64MultiArray ros_chi_f_command;
  std_msgs::Float64MultiArray ros_weight_command;

  void model_update();
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
  std::string chain_name;
  std::string ros_prefix;
  std::vector< vector<std::string> > axis_names;
  std::vector< std::string > object_names;


  void RPY_angles(KDL::Rotation);

  void compute_angles(KDL::Frame frame, double *a0, double *a1, double *a2);
  void derive_angles(KDL::Frame frame, double dd=0.001);

  // helper matrices for derive_angles
  Eigen::MatrixXd U, V, jac, jacinv;
  Eigen::VectorXd S, Sp, tmp;
  bool new_rotation;
};

#endif

