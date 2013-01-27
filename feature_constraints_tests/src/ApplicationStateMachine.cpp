#include "feature_constraints_tests/ApplicationStateMachine.h"

#include "feature_constraints_tests/Conversions.h"

#include "ros/ros.h"

// the state machine that michael shouldn't see ;)

// state flags
bool initial_state_ = true;
bool both_approach_state_ = false;
bool both_touch_oven_state_ = false;
bool left_push_under_state_ = false;
bool right_remove_state_ = false;
bool left_lift_state_ = false;
bool left_flip_state_ = false;
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

void fillCommandBothApproach(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target)
{
  emptyCommand(left_target);
  emptyCommand(right_target);

  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(left_target, 1.0, 0.15, 0.17, -0.1, 0.1);
  // ...align command
  addConstraintCommand(left_target, 1.0, -0.3, -0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(left_target, 1.0, 0.05, 0.1, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(left_target, 0.0, 0.8, 1.0, -0.1, 0.1);

  // RIGHT
  // ... distance command
  addConstraintCommand(right_target, 1.0, 0.00, 0.02, -0.1, 0.1);
  // ... height command
  addConstraintCommand(right_target, 1.0, 0.05, 0.1, -0.1, 0.1);
  // ... align front command
  addConstraintCommand(right_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... align side command
  addConstraintCommand(right_target, 1.0, -0.6, -0.5, -0.1, 0.1);
}

void fillCommandBothTouchOven(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target)
{
  emptyCommand(left_target);
  emptyCommand(right_target);

  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(left_target, 1.0, 0.15, 0.17, -0.1, 0.1);
  // ...align command
  addConstraintCommand(left_target, 1.0, -0.3, -0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(left_target, 1.0, 0.0, 0.01, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(left_target, 0.0, 0.8, 1.0, -0.1, 0.1);

  // RIGHT
  // ... distance command
  addConstraintCommand(right_target, 1.0, 0.0, 0.02, -0.1, 0.1);
  // ... height command
  addConstraintCommand(right_target, 1.0, -0.005, 0.005, -0.1, 0.1);
  // ... align front command
  addConstraintCommand(right_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... align side command
  addConstraintCommand(right_target, 1.0, -0.6, -0.5, -0.1, 0.1);
}

void fillCommandLeftPushUnder(constraint_msgs::ConstraintCommand& left_target)
{
  emptyCommand(left_target);

  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 0.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(left_target, 1.0, -0.02, 0.02, -0.1, 0.1);
  // ...align command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ...height command
  addConstraintCommand(left_target, 1.0, -0.01, 0.01, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(left_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(left_target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandRightRemove(constraint_msgs::ConstraintCommand& right_target)
{
  emptyCommand(right_target);

  // RIGHT
  // ... distance command
  addConstraintCommand(right_target, 1.0, 0.2, 0.3, -0.1, 0.1);
  // ... height command
  addConstraintCommand(right_target, 1.0, 0.05, 0.1, -0.1, 0.1);
  // ... align front command
  addConstraintCommand(right_target, 1.0, -0.05, 0.05, -0.1, 0.1);
  // ... align side command
  addConstraintCommand(right_target, 1.0, -0.6, -0.5, -0.1, 0.1);
}

void fillCommandLeftLift(constraint_msgs::ConstraintCommand& left_target)
{
  emptyCommand(left_target);

  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 0.0, -0.05, 0.05, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(left_target, 1.0, 0.00, 0.07, -0.1, 0.1);
  // ...align command
  addConstraintCommand(left_target, 1.0, -0.08, 0.08, -0.1, 0.1);
  // ...height command
//  addConstraintCommand(left_target, 1.0, 0.15, 0.3, -0.1, 0.1); // WAS WORKING BUT LOOKED PRETTY HIGH!!
  addConstraintCommand(left_target, 1.0, 0.1, 0.15, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(left_target, 1.0, -0.1, -0.05, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(left_target, 0.0, 0.8, 1.0, -0.1, 0.1);
}

void fillCommandLeftFlip(constraint_msgs::ConstraintCommand& left_target)
{
  emptyCommand(left_target);

  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 0.0, -0.05, 0.05, -0.2, 0.2);
  // ...distance command
  addConstraintCommand(left_target, 100.0, 0.00, 0.07, -0.4, 0.4);
  // ...align command
  addConstraintCommand(left_target, 1.0, -0.2, 0.2, -0.2, 0.2);
  // ...height command
//  addConstraintCommand(left_target, 1.0, 0.1, 0.3, -0.2, 0.2); // WAS WORKING BUT LOOKED PRETTY HIGH!!
  addConstraintCommand(left_target, 1.0, 0.1, 0.15, -0.2, 0.2);
  // ...align front command
  // use facing constraint instead of align front to avoid singularity 
  // note: direction vectors of spatula front and plane are perpencicular but relate
  //       that's why this trick works
  addConstraintCommand(left_target, 0.0, -0.9, -0.8, -0.1, 0.1);
  // ... facing constraint
  addConstraintCommand(left_target, 1.0, -0.5, -0.4, -0.1, 0.1);
}

void fillCommandBothStop(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target)
{
  emptyCommand(left_target);
  emptyCommand(right_target);

  // give all constraints a well-behaved command with zero weight, i.e. do not change
  // NOTE: this might still causes motion because of the joint limit avoidance
  
  // LEFT
  // ...pointing command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...distance command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...align command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...height command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...align front command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ...spatula plane facing pancake plan command
  addConstraintCommand(left_target, 0.0, -0.1, 0.1, -0.1, 0.1);

  // RIGHT: TODO
  // ... ditance command
  addConstraintCommand(right_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ... height command
  addConstraintCommand(right_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ... align front command
  addConstraintCommand(right_target, 0.0, -0.1, 0.1, -0.1, 0.1);
  // ... align side command
  addConstraintCommand(right_target, 0.0, -0.1, 0.1, -0.1, 0.1);
}

// the actual state machine code
// returns a flag to signal whether it has finished, i.e. TRUE means the state machine is in 'final state'
bool updateStateMachineAndWriteCommand(constraint_msgs::ConstraintCommand& left_constraint_command_msg, 
                                       constraint_msgs::ConstraintCommand& right_constraint_command_msg,
                                       bool left_constraints_fulfilled,
                                       bool right_constraints_fulfilled)
{
    if(initial_state_ )
    {
      ROS_INFO("Switching to both-approach-state.");
      initial_state_ = false;
      both_approach_state_ = true;
      fillCommandBothApproach(left_constraint_command_msg, right_constraint_command_msg);
      return false;
    }

    if(both_approach_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to both-touch-oven-state.");
      both_approach_state_ = false;
      both_touch_oven_state_ = true;
      fillCommandBothTouchOven(left_constraint_command_msg, right_constraint_command_msg);
      return false;
    }
    
    if(both_touch_oven_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to left-push-under-state.");
      both_touch_oven_state_ = false;
      left_push_under_state_ = true;
      fillCommandLeftPushUnder(left_constraint_command_msg);
      return false;
    }

    if(left_push_under_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to right-remove-state.");
      left_push_under_state_ = false;
      right_remove_state_ = true;
      fillCommandRightRemove(right_constraint_command_msg);
      return false;
    }

    if(right_remove_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to left-lift-state.");
      right_remove_state_ = false;
      left_lift_state_ = true;
      fillCommandLeftLift(left_constraint_command_msg);
      return false;
    }

    if(left_lift_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to left-flip-state.");
      left_lift_state_ = false;
      left_flip_state_ = true;
      fillCommandLeftFlip(left_constraint_command_msg);
      return false;
    }

    if(left_flip_state_ && left_constraints_fulfilled && right_constraints_fulfilled)
    {
      ROS_INFO("Switching to final-state.");
      left_flip_state_ = false;
      final_state_ = true;
      fillCommandBothStop(left_constraint_command_msg, right_constraint_command_msg);
      return true;
    }
    
     if(final_state_)
      return true;
 
     // some state but no new event 
     return false;
}
