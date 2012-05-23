#ifndef OROCOS_PANCAKE_BAKING_COMPONENT_HPP
#define OROCOS_PANCAKE_BAKING_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <iostream>

#include "SubTask.hpp"

#include <feature_constraints/Controller.h>
#include <feature_constraints/Conversions.h>
#include <feature_constraints/FeatureConstraints.h>

#include <ros/ros.h>

class FeatureTask2
  : public iTaSC::SubTask
{
 public:
  FeatureTask2(std::string const& name);
  ~FeatureTask2();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook(){};
  void cleanupHook(){};


private:
  int start_count;
  int guard_time;

  KDL::Frame pose;
  Eigen::MatrixXd Wy;

  // ROS communication
  RTT::InputPort<constraint_msgs::ConstraintConfig> config_port;
  RTT::InputPort<constraint_msgs::ConstraintCommand> command_port;
  RTT::OutputPort<constraint_msgs::ConstraintState> state_port;

  Controller controller;

  constraint_msgs::ConstraintCommand command;
  constraint_msgs::ConstraintState state;
  constraint_msgs::ConstraintConfig config;

  // communication functions

  template <class P> void ROS_add_port(const std::string &rtt_name,
				       const std::string &ros_name,
				       P &port);

  // configuration
  std::string ros_prefix;
};

#endif

