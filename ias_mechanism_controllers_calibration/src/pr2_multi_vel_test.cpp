// basic ros stuff
#include <ros/ros.h>
// messages and goals used
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>
// RobotArm class to talk to standard arm controllers
#include <ias_mechanism_controllers_calibration/robot_arm.h>

// some convenience functions...
std::string construct_full_name(const std::string& ns, const std::string& param_name)
{
  // construct full parameter name
  std::string full_name;
  if(!ns.compare(""))
  {
    full_name = param_name;
  }
  else
  {
    full_name = ns + std::string("/") + param_name;
  } 
  return full_name;
}

bool retrieveParameter(const ros::NodeHandle& n, const std::string& ns, const std::string& param_name, std::string& param)
{
  std::string full_name = construct_full_name(ns, param_name);
  // look up parameter from server
  if(!n.getParam(full_name, param))
  {
    ROS_ERROR("Parameter '%s' not given in namespace: '%s'", full_name.c_str(), n.getNamespace().c_str());
    return false;
  }
  return true;
}

bool retrieveParameter(const ros::NodeHandle& n, const std::string& ns, const std::string& param_name, int& param)
{
  std::string full_name = construct_full_name(ns, param_name);
  // look up parameter from server
  if(!n.getParam(full_name, param))
  {
    ROS_ERROR("Parameter '%s' not given in namespace: '%s'", full_name.c_str(), n.getNamespace().c_str());
    return false;
  }
  return true;
}

bool retrieveParameter(const ros::NodeHandle& n, const std::string& ns, const std::string& param_name, double& param)
{
  std::string full_name = construct_full_name(ns, param_name);
  // look up parameter from server
  if(!n.getParam(full_name, param))
  {
    ROS_ERROR("Parameter '%s' not given in namespace: '%s'", full_name.c_str(), n.getNamespace().c_str());
    return false;
  }
  return true;
}

void printAndSleep(const double sleep_time)
{
  ROS_INFO("Sleeping for %fs...", sleep_time);
  ros::Duration(sleep_time).sleep();
}

void performTest(const ros::Publisher& qdot_des_pub, const std::string& joint_to_test, const int joint_index, const double joint_velocity, const double test_time)
{
  // query user for start of test
  ROS_INFO("About to test joint '%s' with index [%d]. Velocity=%f, time=%f", joint_to_test.c_str(), joint_index, joint_velocity, test_time);
  ROS_INFO("Starting test...");
  std_msgs::Float64MultiArray qdot_msg;
  qdot_msg.data.resize(7);
  for(int i=0; i<7; i++)
  {
    if(i==joint_index)
      qdot_msg.data[i] = joint_velocity;
    else
      qdot_msg.data[i] = 0.0;
  }
  ROS_INFO("Sending command...");
  qdot_des_pub.publish(qdot_msg);
  ROS_INFO("Done.");
  printAndSleep(test_time);
  ROS_INFO("Stopping joint.");
  qdot_msg.data[joint_index] = 0.0;
  qdot_des_pub.publish(qdot_msg);
  ROS_INFO("Test finished.");    
}

// actual test application
int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "pr2_multi_vel_test");
  ros::NodeHandle n_("~");

  // advertise joint velocity command
  ros::Publisher qdot_des_pub_ = n_.advertise<std_msgs::Float64MultiArray>("multi_vel_command", 1);

  // construct RobotArm object...
  RobotArm arm(n_);
  // ...init it
  ROS_INFO("Init of arm action interface...");
  if(!arm.init())
    return 0;

  // activate standard controller to go to desired configuration
  ROS_INFO("Starting standard arm controllers...");
  ros::Duration(0.5).sleep();
  if(!arm.startControllerPlugin())
    return 0;

  // ... use it to go the initial joint configuration
  ROS_INFO("Going to desired initial configuration...");
  if(!arm.gotoDesiredConfiguration())
    return 0;

  // ... and deactivate the standard controller
  ROS_INFO("Stopping standard arm controllers...");
  if(!arm.stopControllerPlugin())
    return 0;
  
  // get controller namespace and desired test parameters from parameter server
  std::string joint_to_test_;
  double joint_velocity_, test_time_;
  int joint_index_;
  std::string vel_controller_ns_;

  if(!retrieveParameter(n_, "", "vel_controller_namespace", vel_controller_ns_))
    return 0;
  if(!retrieveParameter(n_, vel_controller_ns_, "joint_to_test", joint_to_test_))
    return 0;
  if(!retrieveParameter(n_, vel_controller_ns_, "joint_index", joint_index_))
    return 0;
  if(!retrieveParameter(n_, vel_controller_ns_, "joint_velocity", joint_velocity_))
    return 0;
  if(!retrieveParameter(n_, vel_controller_ns_, "test_time", test_time_))
    return 0;

  // start test with user questions
  performTest(qdot_des_pub_, joint_to_test_, joint_index_, joint_velocity_, test_time_);
  printAndSleep(5.0);

  // restart standard controller to be nice
  ROS_INFO("Re-starting standard arm controllers...");
  if(!arm.startControllerPlugin())
    return 0;

  // exit
  return 0;
}
