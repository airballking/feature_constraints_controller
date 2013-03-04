#ifndef ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H
#define ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H

#include <ros/ros.h>

#include <human_observation_msgs/HumanObservationInput.h>
#include <human_observation_msgs/HumanObservationOutput.h>
#include <human_observation_msgs/SingleHumanObservationInput.h>

class HumanObservationAnalyser
{
public:
  HumanObservationAnalyser(ros::NodeHandle& nh);

  void observation_callback(const human_observation_msgs::HumanObservationInput::ConstPtr& msg);  
  void analyse_observation(const human_observation_msgs::SingleHumanObservationInput& observation,
    human_observation_msgs::HumanObservationOutput& analysis_result);
 
  ros::Subscriber human_observation_subscriber_;
  ros::Publisher analysis_publisher_;
};

#endif //ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H
