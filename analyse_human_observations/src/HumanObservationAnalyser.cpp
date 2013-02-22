#include <analyse_human_observations/HumanObservationAnalyser.h>
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Conversions.h>
#include <tf_conversions/tf_kdl.h>

HumanObservationAnalyser::HumanObservationAnalyser(ros::NodeHandle& nh)
{
  // advertise the topic under which we publish the analysis results
  analysis_publisher_ = nh.advertise<human_observation_msgs::HumanObservationOutput>("analysis_output", 1);

  // subscribe to the topic which communicates the observations
  human_observation_subscriber_ = nh.subscribe<human_observation_msgs::HumanObservationInput>
    ("human_observation_input", 1, &HumanObservationAnalyser::observation_callback, this);

  // set correspondences between constraint function names and constraint functions
  Constraint::init();
}

void HumanObservationAnalyser::observation_callback(const human_observation_msgs::HumanObservationInput::ConstPtr& msg)
{
  ROS_INFO("[HumanObservationAnalyser] Received %d observations...", ((int) msg->observations.size()));

  for(unsigned int i=0; i<msg->observations.size(); i++)
  {
    ROS_INFO("[HumanObservationAnalyser] Calculating constraints for timestamp %d...", i);
    human_observation_msgs::HumanObservationOutput analysis_result;
    analysis_result.index = i;
    analyse_observation(msg->observations[i], analysis_result);
    analysis_publisher_.publish(analysis_result);
  }
  ROS_INFO("[HumanObservationAnalyser] Finished.");
}

void HumanObservationAnalyser::analyse_observation(const human_observation_msgs::SingleHumanObservationInput& observation,
    human_observation_msgs::HumanObservationOutput& analysis_result)
{
  assert(observation.tool_features.size() == observation.object_features.size());

  // extract transform 
  KDL::Frame T_tool_in_object;
  tf::PoseMsgToKDL(observation.tool_pose_in_object_frame, T_tool_in_object);
  
  // reset result-message
  analysis_result.constraint_values.clear();

  // create results for cross-product of tool_features, object_features and 
  // feature_functions, and push them into result-message
  for(unsigned int i=0; i<observation.tool_features.size(); i++)
  {
    for(unsigned int j=0; j<observation.object_features.size(); j++)
    {
      for(std::map<std::string, ConstraintFunc>::iterator it=Constraint::constraint_functions_.begin(); it!=Constraint::constraint_functions_.end(); ++it)
      {
        // make single analysis result
        human_observation_msgs::SingleHumanObservationOutput single_result;
        single_result.tool_feature_name = observation.tool_features[i].name;
        single_result.object_feature_name = observation.object_features[j].name;
        single_result.function_type = it->first;

        // assemble auxiliary constraint to evaluate constraint
        Constraint constraint;
        constraint.setFunction(it->first);
        fromMsg(observation.tool_features[i], constraint.tool_feature);
        fromMsg(observation.object_features[j], constraint.object_feature);

        // finally, make the call
        single_result.constraint_value = constraint(T_tool_in_object);

        // push it into the combined result message
        analysis_result.constraint_values.push_back(single_result);
      }      
    }
  }
}
