#ifndef ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H
#define ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H

#include <ros/ros.h>

#include <kdl/frames.hpp>
#include <feature_constraints/FeatureConstraints.h>
#include <string>
#include <vector>

class HumanObservationAnalyser
{
public:
  // input data slots
  KDL::Frame tool_pose_in_object_frame_;
  std::vector<Feature> tool_features_;
  std::vector<Feature> world_features_;

  HumanObservationAnalyser()
  {
    // set correspondes between constraint function names and constraint functions
    Constraint::init();  // needs to be called once

    tool_features_.clear();
    world_features_.clear();
  }
  
  /*
    Calculates values of defined feature functions for all possible combinations of 
    tool- and world-features. 

    All 3 member variables of this class are used as input. All parameters of function
    are used as output values.

    Vector entries with equal index make up one single observation, i.e. value of feature
    function i between tool_feature i and world_feature i can be found in constraint_values[i].
   */
  void calculate_observations(std::vector<std::string>& functions, std::vector<std::string>& tool_feature_names,
    std::vector<std::string>& world_feature_names, std::vector<double>& constraint_values); 

};

/*
   Calculates single observation. All parameters are input, and corresponding 
   feature function value is returned.
 */
double calculate_observation(const std::string& feature_function, const KDL::Frame& frame,
  const Feature& tool_feature, const Feature& world_feature);

#endif //ANALYSE_HUMAN_OBSERVATIONS_HUMAN_OBSERVATION_ANALYSER_H
