#include <ros/ros.h>

#include <analyse_human_observations/HumanObservationAnalyser.h>

int main(int argc, char **argv)
{
  // start up ros
  ros::init(argc, argv, "analyse_human_observations");
  ros::NodeHandle n("~");
  
  // create analysis object which does the ROS-stuff
  HumanObservationAnalyser my_analyser(n);
  
  // spin until we're dead
  ros::spin();
  return 0;
}

