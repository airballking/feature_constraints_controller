#ifndef FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H
#define FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H

#include "constraint_msgs/ConstraintCommand.h"

// convenience functions that write the desired commands for the motion phases
void emptyCommand(constraint_msgs::ConstraintCommand& target);
void fillCommandApproach(constraint_msgs::ConstraintCommand& target);
void fillCommandPushUnder(constraint_msgs::ConstraintCommand& target);
void fillCommandLift(constraint_msgs::ConstraintCommand& target);
void fillCommandFlip(constraint_msgs::ConstraintCommand& target);
void fillCommandStop(constraint_msgs::ConstraintCommand& target);

// the actual state machine
bool updateStateMachineAndWriteCommand(constraint_msgs::ConstraintCommand& constraint_command_msg, bool constraints_fulfilled);
#endif //FEATURE_CONSTRAINTS_APPLICATION_STATE_MACHINE_H

