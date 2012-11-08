#include "ros/ros.h"
#include "constraint_msgs/ConstraintConfig.h"

void configCallback(const constraint_msgs::ConstraintConfig::ConstPtr& msg)
{
  ROS_INFO("Constraint configuration received.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("constraint_config", 5, configCallback);

  ros::spin();
  
  return 0;
}
