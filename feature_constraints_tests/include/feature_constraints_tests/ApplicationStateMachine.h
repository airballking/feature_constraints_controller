#ifndef FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H
#define FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H

#include "constraint_msgs/ConstraintCommand.h"

// convenience functions that write the desired commands for the motion phases
void emptyCommand(constraint_msgs::ConstraintCommand& target);
void fillCommandBothApproach(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target);
void fillCommandBothTouchOven(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target);
void fillCommandLeftPushUnder(constraint_msgs::ConstraintCommand& left_target);
void fillCommandRightRemove(constraint_msgs::ConstraintCommand& right_target);
void fillCommandLeftLift(constraint_msgs::ConstraintCommand& left_target);
void fillCommandLeftFlip(constraint_msgs::ConstraintCommand& left_target);
void fillCommandBothStop(constraint_msgs::ConstraintCommand& left_target, constraint_msgs::ConstraintCommand& right_target);

// the actual state machine
bool updateStateMachineAndWriteCommand(constraint_msgs::ConstraintCommand& left_constraint_command_msg, constraint_msgs::ConstraintCommand& right_constraint_command_msg, bool left_constraints_fulfilled, bool right_constraints_fulfilled);
#endif //FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H

