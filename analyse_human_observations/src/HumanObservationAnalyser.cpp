#include <analyse_human_observations/HumanObservationAnalyser.h>
#include <map>

double calculate_observation(const std::string& feature_function, const KDL::Frame& frame,
  const Feature& tool_feature, const Feature& world_feature)
{
  // assemble auxiliary constraint to evaluate constraint
  Constraint constraint;
  constraint.setFunction(feature_function);
  constraint.tool_feature = tool_feature;
  constraint.object_feature = world_feature;

  // finally, make the call
  return constraint(frame);
}

void HumanObservationAnalyser::calculate_observations(std::vector<std::string>& functions,
  std::vector<std::string>& tool_feature_names, std::vector<std::string>& world_feature_names,
  std::vector<double>& constraint_values)
{
  // first, empty all return vectors
  functions.clear();
  tool_feature_names.clear();
  world_feature_names.clear();
  constraint_values.clear();

  // create results for cross-product of tool_features_, object_features_ and 
  // all available feature_functions
  // push the results into the return vectors
  for(unsigned int i=0; i<tool_features_.size(); i++)
  {
    for(unsigned int j=0; j<world_features_.size(); j++)
    {
      for(std::map<std::string, ConstraintFunc>::iterator it=Constraint::constraint_functions_.begin(); it!=Constraint::constraint_functions_.end(); ++it)
      {
        // make single analysis result
        double constraint_value = calculate_observation(it->first,
          tool_pose_in_object_frame_, tool_features_[i], world_features_[i]);

        // save the result in the vectors
        functions.push_back(it->first);
        tool_feature_names.push_back(tool_features_[i].name);
        world_feature_names.push_back(world_features_[i].name);
        constraint_values.push_back(constraint_value);
      }      
    }
  }
}
