#include <ros/ros.h>
#include <analyse_human_observations/HumanObservationAnalyser.h>
#include <feature_constraints/Conversions.h>
#include <tf_conversions/tf_kdl.h>
#include <human_observation_msgs/AnalyseHumanObservation.h>

// Class to hold the actual analysis object and to handle the
// ROS-related stuff.
class AnalysisServer
{
public:
  HumanObservationAnalyser analyser_;

  bool analyse(human_observation_msgs::AnalyseHumanObservation::Request& req,
               human_observation_msgs::AnalyseHumanObservation::Response& res)
  {
    // first, get request into analyser object
    tf::PoseMsgToKDL(req.tool_pose_in_object_frame, analyser_.tool_pose_in_object_frame_);
    analyser_.tool_features_.resize(req.tool_features.size());
    analyser_.world_features_.resize(req.object_features.size());
    fromMsg(req.tool_features, analyser_.tool_features_);
    fromMsg(req.object_features, analyser_.world_features_);   
 
    // second, prepare vectors for return values from analyse
    std::vector<std::string> function_names, tool_feature_names, 
      world_feature_names;
    std::vector<double> constraint_values;

    // then, make the call
    analyser_.calculate_observations(function_names, tool_feature_names,
      world_feature_names, constraint_values);

    // finally, copy return values into response
    assert(function_names.size() == tool_feature_names.size());
    assert(function_names.size() == world_feature_names.size());
    assert(function_names.size() == constraint_values.size());
    res.observations.resize(function_names.size());
    for(unsigned int i=0; i<function_names.size(); i++)
    {
      res.observations[i].function_type = function_names[i];
      res.observations[i].tool_feature_name = tool_feature_names[i];
      res.observations[i].object_feature_name = world_feature_names[i];
      res.observations[i].constraint_value = constraint_values[i];
    }
  
    // all went well. so, let's get outta here. 
    return true;
  }
};

int main(int argc, char **argv)
{
  // start up ros
  ros::init(argc, argv, "human_observations_analysis_server");
  ros::NodeHandle n("~");

  // create analysis server which handles the ROS-stuff
  AnalysisServer my_server;
  ros::ServiceServer service = n.advertiseService("analyse_observations", &AnalysisServer::analyse, &my_server);

  // spin until we're dead
  ros::spin();
  return 0;
}
