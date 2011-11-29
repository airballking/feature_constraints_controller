// ROS message conversions

#ifndef FEATURE_CONSTRAINTS_CONVERSIONS_H
#define FEATURE_CONSTRAINTS_CONVERSIONS_H

#include <geometry_msgs/Vector3.h>
#include <constraint_msgs/Feature.h>
#include <constraint_msgs/Constraint.h>
#include <constraint_msgs/ConstraintConfig.h>
#include <constraint_msgs/ConstraintCommand.h>
#include <constraint_msgs/ConstraintState.h>

#include <feature_constraints/Controller.h>
#include <feature_constraints/FeatureConstraints.h>


// (hopefully) realtime-safe conversion functions

void fromMsg(const geometry_msgs::Vector3& msg, KDL::Vector vec);
void fromMsg(const constraint_msgs::Feature& msg, Feature& feat);
void fromMsg(const constraint_msgs::Constraint& msg, Constraint& c);
void fromMsg(const constraint_msgs::ConstraintConfig& msg, std::vector<Constraint>& cc);
void fromMsg(std::vector<double>& msg, KDL::JntArray& jnts);
void fromMsg(constraint_msgs::ConstraintCommand& msg, Ranges& ranges);

void toMsg(KDL::Frame& frame, geometry_msgs::Pose& pose);
void toMsg(KDL::JntArray& joints, std::vector<double>& msg);
void toMsg(const KDL::Twist& t, geometry_msgs::Twist& t_msg);
void toMsg(KDL::Jacobian& jac, std::vector<geometry_msgs::Twist>& jac_msg);
void toMsg(Controller& c, constraint_msgs::ConstraintState& c_msg);


// convenience functions

geometry_msgs::Pose toMsg(KDL::Frame& frame);
std::vector<double> toMsg(KDL::JntArray& joints);
geometry_msgs::Twist toMsg(KDL::Twist& t);
std::vector<geometry_msgs::Twist> toMsg(KDL::Jacobian jac);
constraint_msgs::ConstraintState toMsg(Controller& c);

KDL::Vector fromMsg(const geometry_msgs::Vector3& msg);
Feature fromMsg(const constraint_msgs::Feature& msg);
Constraint fromMsg(const constraint_msgs::Constraint& msg);
KDL::JntArray fromMsg(std::vector<double>& msg);
Ranges fromMsg(constraint_msgs::ConstraintCommand& msg);


#endif //FEATURE_CONSTRAINTS_CONVERSIONS_H
