#ifndef FEATURE_CONSTRAINTS_TESTS_CONVERSIONS_H
#define FEATURE_CONSTRAINTS_TESTS_CONVERSIONS_H

#include "constraint_msgs/ConstraintConfig.h"
#include "constraint_msgs/Constraint.h"
#include "constraint_msgs/ConstraintCommand.h"

// Convenience Feature Fillers
void fillFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id);
void fillLineFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id);
void fillPlaneFeature(constraint_msgs::Feature& target, std::string name, std::string frame_id);

// Convenience Constraint Fillers
void fillConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name);
void fillPointingAtConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name);
void fillPerpendicularConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name);
void fillDistanceConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name);
void fillHeightConstraint(constraint_msgs::Constraint& target, constraint_msgs::Feature& tool, constraint_msgs::Feature& world, std::string name);

// Convenience Command Filler
void addConstraintCommand(constraint_msgs::ConstraintCommand& target, double weight, double lower_limit, double upper_limit);

#endif //FEATURE_CONSTRAINTS_TESTS_CONVERSIONS_H
