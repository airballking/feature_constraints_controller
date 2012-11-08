#include "ros/ros.h"
#include "constraint_msgs/ConstraintConfig.h"
#include "constraint_msgs/Constraint.h"
#include "constraint_msgs/ConstraintCommand.h"

int main(int argc, char **argv)
{
  // init ROS and get node-handle
  ros::init(argc, argv, "feature_constraints_executive");
  ros::NodeHandle n;

  // advertise topics
  ros::Publisher constraint_config_publisher = n.advertise<constraint_msgs::ConstraintConfig>("constraint_config", 1, true);
  ros::Publisher constraint_command_publisher = n.advertise<constraint_msgs::ConstraintCommand>("constraint_command", 1, true);

  // construct messages
  constraint_msgs::ConstraintConfig constraint_config_msg;
  constraint_msgs::Constraint pointing_constraint, distance_constraint, align_constraint, height_constraint;
  constraint_msgs::ConstraintCommand constraint_command_msg;

  constraint_msgs::Feature spatula_axis;
  constraint_msgs::Feature pancake_plane;

  // fill messages...
  // ... features
  spatula_axis.name = "main axis";
  spatula_axis.frame_id = "spatula";
  spatula_axis.type = constraint_msgs::Feature::LINE;
  spatula_axis.direction.z = 0.125;

  pancake_plane.name = "pancake plane";
  pancake_plane.frame_id = "pancake";
  pancake_plane.type = constraint_msgs::Feature::PLANE;
  pancake_plane.direction.z = 0.25;
  pancake_plane.contact_direction.x = 0.25;

  // constraints ...
  pointing_constraint.name = "spatula at pancake";
  pointing_constraint.function = "pointing_at";
  distance_constraint.name = "distance from pancake";
  distance_constraint.function = "distance";
  align_constraint.name = "spatula pointing downwards";
  align_constraint.function = "perpendicular";
  height_constraint.name = "keep over";
  height_constraint.function = "height";
  pointing_constraint.tool_feature = spatula_axis;
  pointing_constraint.world_feature = pancake_plane;
  distance_constraint.tool_feature = spatula_axis;
  distance_constraint.world_feature = pancake_plane;
  align_constraint.tool_feature = spatula_axis;
  align_constraint.world_feature = pancake_plane;
  height_constraint.tool_feature = spatula_axis;
  height_constraint.world_feature = pancake_plane;

  constraint_config_msg.constraints.push_back(pointing_constraint);
  constraint_config_msg.constraints.push_back(distance_constraint);
  constraint_config_msg.constraints.push_back(align_constraint);
  constraint_config_msg.constraints.push_back(height_constraint);

  // pointing command
  constraint_command_msg.pos_lo.push_back(0.0);
  constraint_command_msg.pos_hi.push_back(0.0);
  constraint_command_msg.weight.push_back(1.0);  
  // distance command
  constraint_command_msg.pos_lo.push_back(0.3);
  constraint_command_msg.pos_hi.push_back(2.0);
  constraint_command_msg.weight.push_back(1.0);  
  // align command
  constraint_command_msg.pos_lo.push_back(-0.5);
  constraint_command_msg.pos_hi.push_back(-0.3);
  constraint_command_msg.weight.push_back(1.0);  
  // height command
  constraint_command_msg.pos_lo.push_back(0.05);
  constraint_command_msg.pos_hi.push_back(0.2);
  constraint_command_msg.weight.push_back(1.0);  

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
