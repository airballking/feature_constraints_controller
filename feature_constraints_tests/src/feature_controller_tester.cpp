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
bool constraints_fulfilled_ = false;

// callback function listens to controller state and figures out when all constraints are met
void controllerStateCallback(const constraint_msgs::ConstraintState::ConstPtr& msg)
{
  constraints_fulfilled_ = !constraints_fulfilled_;
  
  for(unsigned int i=0; i<msg->weights.size(); i++)
  {
    constraints_fulfilled_ &= (msg->weights[i] < 1.0);
  }  

  if(constraints_fulfilled_){
    ROS_INFO("All constraints are fulfilled.");
  }
}

int main(int argc, char **argv)
{
  // init ROS and get node-handle
  ros::init(argc, argv, "feature_constraints_executive");
  ros::NodeHandle n;

  // advertise topics
  ros::Publisher constraint_config_publisher = n.advertise<constraint_msgs::ConstraintConfig>("constraint_config", 1, true);
  ros::Publisher constraint_command_publisher = n.advertise<constraint_msgs::ConstraintCommand>("constraint_command", 1, true);

  ros::Publisher executive_state_publisher = n.advertise<std_msgs::String>("executive_state", 1, true);

  // subscribe to state topic
  ros::Subscriber constraint_state_subscriber = n.subscribe("constraint_state", 1, controllerStateCallback);

  // construct messages...
  // ... features
  constraint_msgs::Feature spatula_axis, spatula_front, pancake_plane, spatula_plane;
  // ... constraints
  constraint_msgs::Constraint pointing_constraint, distance_constraint, align_constraint, align_front_constraint, height_constraint, facing_constraint;
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

  fillPlaneFeature(spatula_plane, "spatula plane", "spatula");
  spatula_plane.direction.x = 0.1;
  spatula_plane.contact_direction.z = 0.1;

  // ...constraints
  fillPointingAtConstraint(pointing_constraint, spatula_axis, pancake_plane, "spatula at pancake");
  fillPerpendicularConstraint(align_constraint, spatula_axis, pancake_plane, "spatula pointing downwards");
  fillPerpendicularConstraint(align_front_constraint, spatula_front, pancake_plane, "align spatula front");
  fillDistanceConstraint(distance_constraint, spatula_axis, pancake_plane, "distance from pancake");
  fillHeightConstraint(height_constraint, spatula_axis, pancake_plane, "keep over");
  fillPerpendicularConstraint(facing_constraint, spatula_plane, pancake_plane, "spatula facing pancake");

  // ... entire configuration
  constraint_config_msg.constraints.push_back(pointing_constraint);
  constraint_config_msg.constraints.push_back(distance_constraint);
  constraint_config_msg.constraints.push_back(align_constraint);
  constraint_config_msg.constraints.push_back(height_constraint);
  constraint_config_msg.constraints.push_back(align_front_constraint);
  constraint_config_msg.constraints.push_back(facing_constraint);

  // publish constraint configuration message
  constraint_config_publisher.publish(constraint_config_msg);

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
    state_machine_finished = updateStateMachineAndWriteCommand(constraint_command_msg, constraints_fulfilled_);
    
    constraint_command_publisher.publish(constraint_command_msg);
  
    ros::spinOnce();
    loop_rate.sleep();
  }

  executive_state_msg.data = "finish";
  executive_state_publisher.publish(executive_state_msg);

  // bye bye
  ROS_INFO("Terminating executive.");

  return 0;
}
