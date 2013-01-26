// some ros stuff
#include "ros/ros.h"
// the messages
#include "constraint_msgs/ConstraintConfig.h"
#include "constraint_msgs/Constraint.h"
#include "constraint_msgs/ConstraintCommand.h"
#include "constraint_msgs/ConstraintState.h"
// for test output
#include "std_msgs/String.h"
// some convenience functions to build config message
#include "feature_constraints_tests/Conversions.h"
// the state machine
#include "feature_constraints_tests/ApplicationStateMachine.h"

// constraints_fulfilled_ triggers state transitions, i.e. it's an event
bool left_constraints_fulfilled_ = false;
bool right_constraints_fulfilled_ = false;

// callback functions which listen to controller states and figure out when all constraints are met
void leftControllerStateCallback(const constraint_msgs::ConstraintState::ConstPtr& msg)
{
  if(left_constraints_fulfilled_ && right_constraints_fulfilled_)
  {
    left_constraints_fulfilled_ = false;
  }
  else
  {
    left_constraints_fulfilled_ = true;
  }
  
  for(unsigned int i=0; i<msg->weights.size(); i++)
  {
    left_constraints_fulfilled_ &= (msg->weights[i] < 1.0);
  }  

  if(left_constraints_fulfilled_){
    ROS_INFO("[LEFT ARM] All constraints are fulfilled.");
  }
}

void rightControllerStateCallback(const constraint_msgs::ConstraintState::ConstPtr& msg)
{
  if(left_constraints_fulfilled_ && right_constraints_fulfilled_)
  {
    right_constraints_fulfilled_ = false;
  }
  else
  {
    right_constraints_fulfilled_ = true;
  }

  for(unsigned int i=0; i<msg->weights.size(); i++)
  {
    right_constraints_fulfilled_ &= (msg->weights[i] < 1.0);
  }  

  if(right_constraints_fulfilled_){
    ROS_INFO("[RIGHT ARM] All constraints are fulfilled.");
  }
}

int main(int argc, char **argv)
{
  // init ROS and get node-handle
  ros::init(argc, argv, "feature_constraints_executive");
  ros::NodeHandle n;

  // advertise topics
  ros::Publisher l_constraint_config_publisher = n.advertise<constraint_msgs::ConstraintConfig>("l_constraint_config", 1, true);
  ros::Publisher l_constraint_command_publisher = n.advertise<constraint_msgs::ConstraintCommand>("l_constraint_command", 1, true);
  ros::Publisher r_constraint_config_publisher = n.advertise<constraint_msgs::ConstraintConfig>("r_constraint_config", 1, true);
  ros::Publisher r_constraint_command_publisher = n.advertise<constraint_msgs::ConstraintCommand>("r_constraint_command", 1, true);

  ros::Publisher executive_state_publisher = n.advertise<std_msgs::String>("executive_state", 1, true);

  // subscribe to state topic
  ros::Subscriber l_constraint_state_subscriber = n.subscribe("l_constraint_state", 1, leftControllerStateCallback);
  ros::Subscriber r_constraint_state_subscriber = n.subscribe("r_constraint_state", 1, rightControllerStateCallback);

  // construct messages...
  // ... features
  // LEFT
  constraint_msgs::Feature l_spatula_axis, l_spatula_front, l_spatula_plane;
  // PANCAKE
  constraint_msgs::Feature pancake_plane, pancake_right_rim, pancake_right_rim_plane;
  // RIGHT 
  constraint_msgs::Feature r_spatula_front, r_spatula_plane;

  // ... constraints
  // LEFT
  constraint_msgs::Constraint l_pointing_constraint, l_distance_constraint, l_align_constraint, l_align_front_constraint, l_height_constraint, l_facing_constraint;
  // RIGHT
  constraint_msgs::Constraint r_distance_constraint, r_height_constraint, r_align_front_constraint, r_align_side_constraint;

  // ... entire configurations
  constraint_msgs::ConstraintConfig l_constraint_config_msg;
  constraint_msgs::ConstraintConfig r_constraint_config_msg;
  // ... commands
  constraint_msgs::ConstraintCommand l_constraint_command_msg;
  constraint_msgs::ConstraintCommand r_constraint_command_msg;

  // fill messages...
  // ... features
  // LEFT TOOL FEATURES
  fillLineFeature(l_spatula_axis, "left spatula: main axis", "l_spatula");
  l_spatula_axis.direction.z = 0.125;

  fillLineFeature(l_spatula_front, "left spatula: front axis", "l_spatula");
  l_spatula_front.position.z = 0.0475;
  l_spatula_front.direction.y = 0.125;

  fillPlaneFeature(l_spatula_plane, "left spatula: plane", "l_spatula");
  l_spatula_plane.direction.x = 0.1;
  l_spatula_plane.contact_direction.z = 0.1;
  
  // RIGHT TOOL FEATURES
  fillLineFeature(r_spatula_front, "right spatula: front axis", "r_spatula");
  r_spatula_front.position.z = 0.0475;
  r_spatula_front.direction.y = 0.125;

  fillPlaneFeature(r_spatula_plane, "right spatula: plane", "r_spatula");
  r_spatula_plane.direction.x = 0.1;
  r_spatula_plane.contact_direction.z = 0.1;

  // PANCAKE FEATURES
  fillPlaneFeature(pancake_plane, "pancake plane", "pancake");
  pancake_plane.direction.z = 0.1;
  pancake_plane.contact_direction.x = 0.1;

  fillLineFeature(pancake_right_rim, "pancake right rim", "pancake");
  pancake_right_rim.position.y = -0.06;
  pancake_right_rim.direction.x = 0.1;

  fillPlaneFeature(pancake_right_rim_plane, "pancake right rim plane", "pancake");
  pancake_right_rim_plane.direction.z = 0.1;
  pancake_right_rim_plane.position.y = -0.1;
  pancake_right_rim_plane.contact_direction.x = 0.1;

  // ...constraints
  // LEFT
  fillPointingAtConstraint(l_pointing_constraint, l_spatula_axis, pancake_plane, "left spatula: at pancake");
  fillPerpendicularConstraint(l_align_constraint, l_spatula_axis, pancake_plane, "left spatula: pointing downwards");
  fillPerpendicularConstraint(l_align_front_constraint, l_spatula_front, pancake_plane, "left spatula: align front");
  fillDistanceConstraint(l_distance_constraint, l_spatula_axis, pancake_plane, "left spatula: distance from pancake");
  fillHeightConstraint(l_height_constraint, l_spatula_axis, pancake_plane, "left spatula: keep over");
  fillPerpendicularConstraint(l_facing_constraint, l_spatula_plane, pancake_plane, "left spatula: facing pancake");
  // RIGHT
  fillDistanceConstraint(r_distance_constraint, r_spatula_front, pancake_right_rim_plane, "right spatula: distance front to pancake rim");
  fillHeightConstraint(r_height_constraint, r_spatula_front, pancake_right_rim_plane, "right spatula: height front over pancake rim");
  fillPerpendicularConstraint(r_align_front_constraint, r_spatula_front, pancake_right_rim_plane, "right spatula: align front");
  fillPerpendicularConstraint(r_align_side_constraint, r_spatula_plane, pancake_right_rim_plane, "right spatula: align side");

  // ... entire configurations
  // LEFT
  l_constraint_config_msg.constraints.push_back(l_pointing_constraint);
  l_constraint_config_msg.constraints.push_back(l_distance_constraint);
  l_constraint_config_msg.constraints.push_back(l_align_constraint);
  l_constraint_config_msg.constraints.push_back(l_height_constraint);
  l_constraint_config_msg.constraints.push_back(l_align_front_constraint);
  l_constraint_config_msg.constraints.push_back(l_facing_constraint);
  // RIGHT
  r_constraint_config_msg.constraints.push_back(r_distance_constraint);
  r_constraint_config_msg.constraints.push_back(r_height_constraint);
  r_constraint_config_msg.constraints.push_back(r_align_front_constraint);
  r_constraint_config_msg.constraints.push_back(r_align_side_constraint);

  // publish constraint configuration message
  l_constraint_config_publisher.publish(l_constraint_config_msg);
  r_constraint_config_publisher.publish(r_constraint_config_msg);

  // start application which sends different command messages
  ros::Rate loop_rate(1);
  ROS_INFO("Starting executive in initial state.");

  std_msgs::String executive_state_msg;
  executive_state_msg.data = "start";
  executive_state_publisher.publish(executive_state_msg);

  // another flag which signals that the state machine has finished
  bool state_machine_finished = false;
  while(ros::ok() && !state_machine_finished)
  {
    
    // the mean state machine that michael should not see ;)
    state_machine_finished = updateStateMachineAndWriteCommand(l_constraint_command_msg, r_constraint_command_msg, left_constraints_fulfilled_, right_constraints_fulfilled_);
    
    l_constraint_command_publisher.publish(l_constraint_command_msg);
    r_constraint_command_publisher.publish(r_constraint_command_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  executive_state_msg.data = "finish";
  executive_state_publisher.publish(executive_state_msg);

  // bye bye
  ROS_INFO("Terminating executive.");

  return 0;
}
