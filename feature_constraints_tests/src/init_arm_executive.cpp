// ros for NodeHandles and output
#include "ros/ros.h"
// RobotArm class to talk to standard arm controllers
#include <ias_mechanism_controllers_calibration/robot_arm.h>

bool goToInitConfiguration(ros::NodeHandle& n)
{
  // construct RobotArm object...
  RobotArm arm(n);
  // ...init it
  ROS_INFO("Init of arm action interface...");
  if(!arm.init())
    return false;

  // ... use it to go the initial joint configuration
  ROS_INFO("Going to desired initial configuration...");
  if(!arm.gotoDesiredConfiguration())
    return false;

  // no erros occured
  return true;
}


int main(int argc, char **argv)
{
  // init ROS and get node-handle
  ros::init(argc, argv, "init_arm_executive");
  ros::NodeHandle n("~");

  // use standard arm-controllers to go to desired initial configuration
  // report error if something went wrong
  if(!goToInitConfiguration(n))
    ROS_ERROR("Init of arm failed.");

  return 0;
}
