#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <kdl/jntarray.hpp>

#include <ReflexxesAPI.h>

int main(int argc, char** argv)
{
  // ros-stuff
  ros::init(argc, argv, "reflexxes_tutorial");
  ros::NodeHandle n("~");

  // init experimental joint-state representation
  unsigned int dof = 2;
  double cycle_time = 0.01;
  double time = 0.00;
  KDL::JntArray q, qdot, qdot_max, qddot_max, jerk_max;
  q.resize(dof);
  qdot.resize(dof);
  qdot_max.resize(dof);
  qddot_max.resize(dof);
  jerk_max.resize(dof);

  // init experimental targets
  KDL::JntArray q_target, qdot_target;
  q_target.resize(dof);
  qdot_target.resize(dof);

  // init selection vector
  bool selection_vector[dof];

  // fill experimental values
  for(unsigned int i=0; i<q.rows(); i++)
  {
    // filling joint-state and dynamic joint constraints
    double j = i;
    q(i) = j*M_PI;
    qdot(i) = 0.0;
    qdot_max(i) = 0.1;
    qddot_max(i) = qdot_max(i)/2.0;
    jerk_max(i) = 0.5;

    // filling desired target
    q_target(i) = (1.0-j)*M_PI;
    qdot_target(i) = j*qdot_max(i)*0.5;

    // fill seletion matrix
    selection_vector[i] = true;
  }

  // set up relfexxes-trajectory structures
  ReflexxesAPI trajectory_generator(dof, cycle_time);
  RMLPositionInputParameters trajectory_input(dof);
  RMLPositionOutputParameters trajectory_output(dof);
  RMLPositionFlags trajectory_generator_flags;
  
  // perform experimental interpolation
  // set up input of trajectory generator
  trajectory_input.SetMaxVelocityVector(qdot_max.data.data());
  trajectory_input.SetMaxAccelerationVector(qddot_max.data.data());
  trajectory_input.SetMaxJerkVector(jerk_max.data.data());

  trajectory_input.SetTargetPositionVector(q_target.data.data());
  trajectory_input.SetTargetVelocityVector(qdot_target.data.data());

  trajectory_input.SetSelectionVector(selection_vector);

  // write header and initial motion state to file
  std::ofstream myfile;
  myfile.open("trajectory.data");
  myfile << "time \t q[0] \t q[1] \t qdot[0] \t qdot[1]\n";
  myfile << time << "\t" << q(0) << "\t" << q(1) << "\t" << qdot(0) << "\t" << qdot(1) << "\n";

  // the loop
  int ResultValue = 0;
  while(ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
  {
    // set the current state of the robot as input
    trajectory_input.SetCurrentPositionVector(q.data.data());
    trajectory_input.SetCurrentVelocityVector(qdot.data.data());
   
    // check the input
    if(!trajectory_input.CheckForValidity())
    {
      ROS_WARN("Trajectory input was not valid.");
    }

    // query the trajectory generator
    ResultValue = trajectory_generator.RMLPosition(trajectory_input, &trajectory_output, trajectory_generator_flags);

    // check the output of the trajectory generator
    if(ResultValue < 0)
    {
      ROS_ERROR("Trajectory generator returned with an error.");
      break;
    }

    // perform 'control of the robot'
    time += cycle_time;
    trajectory_output.GetNewPositionVector(q.data.data(), dof*sizeof(double));
    trajectory_output.GetNewVelocityVector(qdot.data.data(), dof*sizeof(double));

    // write the current motion state of 'robot'
    myfile << time << "\t" << q(0) << "\t" << q(1) << "\t" << qdot(0) << "\t" << qdot(1) << "\n";
  } 

  myfile.close();
  return 0;
}
