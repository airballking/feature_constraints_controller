// some ros stuff
#include "ros/ros.h"

// the messages
#include "constraint_msgs/ConstraintConfig.h"
#include "constraint_msgs/Constraint.h"
#include "constraint_msgs/ConstraintCommand.h"

// some convenience functions
#include "feature_constraints_tests/Conversions.h"

int main(int argc, char **argv)
{
  // init ROS and get node-handle
  ros::init(argc, argv, "feature_constraints_executive");
  ros::NodeHandle n;

  // advertise topics
  ros::Publisher constraint_config_publisher = n.advertise<constraint_msgs::ConstraintConfig>("constraint_config", 1, true);
  ros::Publisher constraint_command_publisher = n.advertise<constraint_msgs::ConstraintCommand>("constraint_command", 1, true);

  // construct messages...
  // ... features
  constraint_msgs::Feature spatula_axis, spatula_front, pancake_plane;
  // ... constraints
  constraint_msgs::Constraint pointing_constraint, distance_constraint, align_constraint, align_front_constraint, height_constraint;
  // ... entire configuration
  constraint_msgs::ConstraintConfig constraint_config_msg;
    // ... command
  constraint_msgs::ConstraintCommand constraint_command_msg;
  
  // fill messages...
  // ... features
  fillLineFeature(spatula_axis, "main axis", "spatula");
  spatula_axis.direction.z = 0.125;

  fillLineFeature(spatula_front, "front spatula", "spatula");
  spatula_front.position.z = 0.075;
  spatula_front.direction.y = 0.125;

  fillPlaneFeature(pancake_plane, "pancake plane", "pancake");
  pancake_plane.direction.z = 0.25;
  pancake_plane.contact_direction.x = 0.25;

  // ...constraints
  fillPointingAtConstraint(pointing_constraint, spatula_axis, pancake_plane, "spatula at pancake");
  fillPerpendicularConstraint(align_constraint, spatula_axis, pancake_plane, "spatula pointing downwards");
  fillPerpendicularConstraint(align_front_constraint, spatula_front, pancake_plane, "align spatula front");
  fillDistanceConstraint(distance_constraint, spatula_axis, pancake_plane, "distance from pancake");
  fillHeightConstraint(height_constraint, spatula_axis, pancake_plane, "keep over");

  // ... entire configuration
  constraint_config_msg.constraints.push_back(pointing_constraint);
  constraint_config_msg.constraints.push_back(distance_constraint);
  constraint_config_msg.constraints.push_back(align_constraint);
  constraint_config_msg.constraints.push_back(height_constraint);
  constraint_config_msg.constraints.push_back(align_front_constraint);

  // ...commands...
  // ...pointing command
  addConstraintCommand(constraint_command_msg, 1.0, 0.0, 0.0);
  // ...distance command
  addConstraintCommand(constraint_command_msg, 1.0, 0.2, 2.0);
  // ...align command
  addConstraintCommand(constraint_command_msg, 1.0, -0.3, -0.1);
  // ...height command
  addConstraintCommand(constraint_command_msg, 1.0, 0.05, 0.2);
  // ...align front command
  addConstraintCommand(constraint_command_msg, 1.0, -0.05, 0.05);

  // publish messages
  constraint_config_publisher.publish(constraint_config_msg);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    constraint_command_publisher.publish(constraint_command_msg);
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  // bye bye
  return 0;
}
