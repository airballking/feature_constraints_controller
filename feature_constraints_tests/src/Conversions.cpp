#include "feature_constraints_tests/Conversions.h"

void fillFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id)
{
  target.name = name;
  target.frame_id = frame_id;
}

void fillLineFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id)
{
  fillFeature(target, name, frame_id);
  target.type = constraint_msgs::Feature::LINE;
}

void fillPlaneFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id)
{
  fillFeature(target, name, frame_id);
  target.type = constraint_msgs::Feature::PLANE;
}

void fillConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name)
{
  target.tool_feature = tool;
  target.world_feature = world;
  target.name = name;
}

void fillPointingAtConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name)
{
  fillConstraint(target, tool, world, name);
  target.function = "pointing_at";
}

void fillPerpendicularConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name)
{
  fillConstraint(target, tool, world, name);
  target.function = "perpendicular";
}

void fillDistanceConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name)
{
  fillConstraint(target, tool, world, name);
  target.function = "distance";
}

void fillHeightConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name)
{
  fillConstraint(target, tool, world, name);
  target.function = "height";
}

void addConstraintCommand(constraint_msgs::ConstraintCommand& target, double weight, double lower_limit, double upper_limit)
{
  target.weight.push_back(weight);
  target.pos_lo.push_back(lower_limit);
  target.pos_hi.push_back(upper_limit);
}

