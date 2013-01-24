#include "feature_constraints_tests/ApplicationStateMachine.h"

#include "feature_constraints_tests/Conversions.h"

#include "ros/ros.h"

// the state machine that michael shouldn't see ;)

// state flags
bool initial_state_ = true;
bool approach_state_ = false;
bool touch_oven_state_ = false;
bool push_under_state_ = false;
bool lift_state_ = false;
bool flip_state_ = false;
bool final_state_ = false;

// convenience functions that fill the command message for the motion phases
void emptyCommand(constraint_msgs::ConstraintCommand& target)
{
  target.weight.clear();
  target.pos_lo.clear();
  target.pos_hi.clear();
  target.min_vel.clear();
  target.max_vel.clear();
}

void fillCommandApproach(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // ...pointing command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(target, 1.0, 0.15, 0.17, -0.1, 0.1);
  // ...align command
  addConstraintCommand(target, 1.0, -0.3, -0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(target, 1.0, 0.05, 0.1, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandTouchOven(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // ...pointing command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(target, 1.0, 0.15, 0.17, -0.1, 0.1);
  // ...align command
  addConstraintCommand(target, 1.0, -0.3, -0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(target, 1.0, 0.0, 0.01, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandPushUnder(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // ...pointing command
  addConstraintCommand(target, 0.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(target, 1.0, -0.02, 0.02, -0.1, 0.1);
  // ...align command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...height command
  addConstraintCommand(target, 1.0, -0.01, 0.01, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandLift(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // ...pointing command
  addConstraintCommand(target, 0.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(target, 1.0, 0.00, 0.07, -0.1, 0.1);
  // ...align command
  addConstraintCommand(target, 1.0, -0.08, 0.08, -0.1, 0.1);
  // ...height command
  addConstraintCommand(target, 1.0, 0.15, 0.3, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(target, 1.0, -0.1, -0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandFlip(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // ...pointing command
  addConstraintCommand(target, 0.0, -0.05, 0.05, -0.2, 0.2);
  // ...distance command
  addConstraintCommand(target, 1.0, 0.00, 0.07, -0.2, 0.2);
  // ...align command
  addConstraintCommand(target, 1.0, -0.2, 0.2, -0.2, 0.2);
  // ...height command
  addConstraintCommand(target, 1.0, 0.1, 0.3, -0.2, 0.2);
  // ...align front command
  // SINGULARITY !!
  // maybe implement and try new command 'facing'
  addConstraintCommand(target, 0.0, -0.9, -0.8, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(target, 1.0, -0.5, -0.4, -0.1, 0.1);
}

void fillCommandStop(constraint_msgs::ConstraintCommand& target)
{
  emptyCommand(target);

  // give all constraints a well-behaved command with zero weight, i.e. do not change
  // NOTE: this might still causes motion because of the joint limit avoidance
  // ...pointing command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...align command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...spatula plane facing pancake plan command
  addConstraintCommand(target, 0.0, -0.1, 0.1, -0.1, 0.1);
}

// the actual state machine code
// returns a flag to signal whether it has finished, i.e. TRUE means the state machine is in 'final state'
bool updateStateMachineAndWriteCommand(constraint_msgs::ConstraintCommand& constraint_command_msg, bool constraints_fulfilled)
{
    if(initial_state_ )
    {
      ROS_INFO("Switching to approach-state.");
      initial_state_ = false;
      approach_state_ = true;
      fillCommandApproach(constraint_command_msg);
      return false;
    }

    if(approach_state_ && constraints_fulfilled)
    {
      ROS_INFO("Switching to touc-oven-state.");
      approach_state_ = false;
      touch_oven_state_ = true;
      fillCommandTouchOven(constraint_command_msg);
      return false;
    }
    
    if(touch_oven_state_ && constraints_fulfilled)
    {
      ROS_INFO("Switching to push-under-state.");
      touch_oven_state_ = false;
      push_under_state_ = true;
      fillCommandPushUnder(constraint_command_msg);
      return false;
    }

    if(push_under_state_ && constraints_fulfilled)
    {
      ROS_INFO("Switching to lift-state.");
      push_under_state_ = false;
      lift_state_ = true;
      fillCommandLift(constraint_command_msg);
      return false;
    }

    if(lift_state_ && constraints_fulfilled)
    {
      ROS_INFO("Switching to flip-state.");
      lift_state_ = false;
      flip_state_ = true;
      fillCommandFlip(constraint_command_msg);
      return false;
    }
    if(flip_state_ && constraints_fulfilled)
    {
      ROS_INFO("Switching to final-state.");
      flip_state_ = false;
      final_state_ = true;
      fillCommandStop(constraint_command_msg);
      return true;
    }
    
     if(final_state_)
      return true;
 
     // some state but no new event 
     return false;
}
