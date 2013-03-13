
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>
#include <feature_constraints/Conversions.h>

#include <Eigen/Core>


using namespace KDL;
using namespace Eigen;


Ranges::Ranges(int size) :
  pos_lo(size), pos_hi(size), weight(size), max_vel(size), min_vel(size)
{

}

void Ranges::resize(int size)
{
  pos_lo.resize(size);
  pos_hi.resize(size);
  weight.resize(size);
  max_vel.resize(size);
  min_vel.resize(size);
}


bool Controller::prepare(std::string& filter_namespace)
{
  int n = constraints.size();

  chi.resize(n);
  chi_desired.resize(n);
  chi_dot_desired.resize(n);

  Ht.resize(n);
  ydot.resize(n);
  weights.resize(n);

  p_gains.resize(n);
  d_gains.resize(n);
  command.resize(n);
  intermediate_command.resize(n);

  J.resize(n);
  singularValues.resize(6);

  tmp.resize(n);
  analysis.resize(n);  // maximum number of constraints

  // stuff for Reflexxes trajectory generation
  if(command_trajectory_generator)
    delete command_trajectory_generator;
  if(command_trajectory_input)
    delete command_trajectory_input;
  if(command_trajectory_output)
    delete command_trajectory_output;

  command_trajectory_generator = new ReflexxesAPI(n, 0.01);
  command_trajectory_input = new RMLPositionInputParameters(n);
  command_trajectory_output = new RMLPositionOutputParameters(n);
  
  // stuff for constraint velocity estimation
  last_chi.resize(n);
  chi_dot.resize(n);
  tmp_vector1.resize(n);
  tmp_vector2.resize(n);
  filter_chain.clear();
  return filter_chain.configure(n, filter_namespace);
}


void Controller::update(KDL::Frame& frame, bool with_control, double dt)
{
  differentiateConstraints(Ht, chi, frame, constraints, 0.001, tmp);
  estimateVelocities(chi, last_chi, dt, filter_chain, chi_dot, tmp,
    tmp_vector1, tmp_vector2);
  if(with_control)
  {
    control(ydot, weights, chi_desired, chi, command, p_gains);

    // TODO(Georg): redo this
    //control(ydot, weights, chi_desired, chi_dot_desired, chi, chi_dot, command,
    //  *command_trajectory_generator, *command_trajectory_input, *command_trajectory_output,
    //  command_trajectory_flags, tmp, p_gains, d_gains);
  }
  // TODO: check if we are still using J
  analysis.analyzeH(Ht, J, singularValues, 1e-7);
  this->frame = frame;
}

void Controller::clampOutput()
{
  clamp(ydot, command.min_vel, command.max_vel);
}

void control(KDL::JntArray& ydot,
             KDL::JntArray& weights,
             KDL::JntArray& chi_desired,
             const KDL::JntArray& chi,
             const Ranges& command,
             const KDL::JntArray gains)
{
  double s = 0.05;
  for(unsigned int i=0; i < chi.rows(); i++)
  {
    if(command.weight(i) == 0.0)
    {
      ydot(i) = 0.0;
      weights(i) = 0.0;
      continue;
    }

    double value = chi(i);

    double lo = command.pos_lo(i);
    double hi = command.pos_hi(i);

    // adjust margin if range is too small
    double ss = (hi - lo < 2*s) ? (hi - lo) / 2 : s;

    if(value > hi - ss)
    {
      ydot(i) = gains(i)*(hi - ss - value);
      chi_desired(i) = hi - ss;
    }
    else if(value < lo + ss)
    {
      ydot(i) = gains(i)*(lo + ss - value);
      chi_desired(i) = lo + ss;
    }
    else
    {
      ydot(i) = 0.0;
      chi_desired(i) = value;
    }


    if(value > hi || value < lo)
    {
      //weights(i) = 1.0;
      weights(i) = command.weight(i);
    }
    else
    {
      double w_lo = (1/s)*(-hi + value)+1;
      double w_hi = (1/s)*( lo - value)+1;

      w_lo = (w_lo > 0.0) ? w_lo : 0.0;
      w_hi = (w_hi > 0.0) ? w_hi : 0.0;

      weights(i) = (w_lo > w_hi) ? w_lo : w_hi;
    }
  }
}


void control(KDL::JntArray& ydot,
             KDL::JntArray& weights,
             KDL::JntArray& chi_desired,
             KDL::JntArray& chi_dot_desired,
             const KDL::JntArray& chi,
             const KDL::JntArray& chi_dot,
             const Ranges& command,
             ReflexxesAPI& trajectory_generator,
             RMLPositionInputParameters& trajectory_input,
             RMLPositionOutputParameters& trajectory_output,
             RMLPositionFlags& trajectory_flags,
             KDL::JntArray& tmp,
             const KDL::JntArray p_gains,
             const KDL::JntArray d_gains)
{
  // reflexxes library: set max_vel, max_acc and max_jerk
  // TODO: redo this
  trajectory_input.SetMaxVelocityVector(command.max_vel.data.data());
  KDL::Multiply(command.max_vel, 100.0, tmp);
  trajectory_input.SetMaxAccelerationVector(tmp.data.data()); // implies we reach max_vel within 0.01sec (with max_jerk-->infinity)
  //trajectory_input.SetMaxAccelerationVector(command.max_vel.data.data()); // implies we reach max_vel within 1sec (with max_jerk-->infinity)
  //KDL::Multiply(command.max_vel, 5.0, tmp);
  trajectory_input.SetMaxJerkVector(tmp.data.data()); // implies we reach max_acc within 0.01sec

  // reflexxes library: explicitedly enable all trajectory generators
  for(unsigned int i=0; i<chi.rows(); i++)
  {
    trajectory_input.SetSelectionVectorElement(true, i);
  }
  
  // reflexxes library: set current constraint velocities and position
  trajectory_input.SetCurrentPositionVector(chi.data.data());
  // TODO: redo this
  KDL::SetToZero(tmp);
  //trajectory_input.SetCurrentVelocityVector(chi_dot.data.data());
  trajectory_input.SetCurrentVelocityVector(tmp.data.data());

  // our algorithm: determine chi_desired and weights based on ranges 
  double s = 0.05;
  for(unsigned int i=0; i < chi.rows(); i++)
  {
    if(command.weight(i) == 0.0)
    {
      ydot(i) = 0.0;
      weights(i) = 0.0;
      continue;
    }

    double value = chi(i);

    double lo = command.pos_lo(i);
    double hi = command.pos_hi(i);

    // adjust margin if range is too small
    double ss = (hi - lo < 2*s) ? (hi - lo) / 2 : s;

    if(value > hi - ss)
    {
      chi_desired(i) = hi - ss;
    }
    else if(value < lo + ss)
    {
      chi_desired(i) = lo + ss;
    }
    else
    {
      chi_desired(i) = value;
    }

    // calc ouput weights based on desired final weights
    if(value > hi || value < lo)
    {
      //weights(i) = 1.0;
      weights(i) = command.weight(i);
    }
    else
    {
      double w_lo = (1/s)*(-hi + value)+1;
      double w_hi = (1/s)*( lo - value)+1;

      w_lo = (w_lo > 0.0) ? w_lo : 0.0;
      w_hi = (w_hi > 0.0) ? w_hi : 0.0;

      weights(i) = (w_lo > w_hi) ? w_lo : w_hi;
    }
  }

  // reflexxes library: set desired positions and desired final velocity (later here hard-coded to zero)
  // set the desired values
  trajectory_input.SetTargetPositionVector(chi_desired.data.data());
  KDL::SetToZero(tmp);
  trajectory_input.SetTargetVelocityVector(tmp.data.data());

  // reflexxes library: ask for interpolation
  int return_value = trajectory_generator.RMLPosition(trajectory_input, &trajectory_output, trajectory_flags);
  if(return_value < 0)
  {
    ROS_ERROR("[FEATURE_CONSTRAINTS_CONTROLLER] Reflexxes interpolator returned with error '%d'", return_value);
    KDL::SetToZero(ydot);
    return;
  }

  // reflexess library copy results into chi_desired and chi_dot_desired
  trajectory_output.GetNewPositionVector(chi_desired.data.data(), chi_desired.rows()*sizeof(double));
  trajectory_output.GetNewVelocityVector(chi_dot_desired.data.data(), chi_dot_desired.rows()*sizeof(double));

  // perform control to calculate ydot
  for(unsigned int i=0; i<chi.rows(); i++)
  {
    // TODO: change this to consider the margins
    if(chi(i) < command.pos_hi(i) && chi(i) > command.pos_lo(i))
    {
      ydot(i) = 0.0;
    }
    else
    {
      ydot(i) = p_gains(i)*(chi_desired(i) - chi(i)); //+ d_gains(i)*(chi_dot_desired(i) - chi_dot(i));
    }
  }
}

double clamp(double input_velocity, double min_velocity, double max_velocity)
{
  assert(min_velocity <= max_velocity);

  if(input_velocity < min_velocity)
    return min_velocity;
  if(input_velocity > max_velocity)
    return max_velocity;

  // nothing to clamp
  return input_velocity;
}

void clamp(KDL::JntArray& joint_velocities, const KDL::JntArray& min_velocities,
           const KDL::JntArray& max_velocities)
{
  assert(joint_velocities.rows() == min_velocities.rows());
  assert(joint_velocities.rows() == max_velocities.rows());

  for(unsigned int i=0; i<joint_velocities.rows(); i++)
  {
    joint_velocities(i) = clamp(joint_velocities(i), min_velocities(i), 
                            max_velocities(i));
  }
}

void estimateVelocities(const KDL::JntArray& chi, KDL::JntArray& chi_old, double dt,
                        filters::MultiChannelFilterChain<double>& filters,
                        KDL::JntArray& chi_dot, KDL::JntArray& tmp,
                        std::vector<double>& tmp_vector1, std::vector<double>& tmp_vector2)
{
  assert(chi.rows() == chi_old.rows());
  assert(chi.rows() == chi_dot.rows());
  assert(chi.rows() == tmp.rows());
  assert(chi.rows() == tmp_vector1.size());
  assert(chi.rows() == tmp_vector2.size());

  // calculate instantaneous velocities through numerical differentiation
  // NOTE: in the next line I'm using chi_dot as a second 'tmp'
  KDL::Subtract(chi, chi_old, chi_dot);
  KDL::Divide(chi_dot, dt, tmp);

  // copy the result into a std::vector to send it to the filters
  // use one of our existing message-conversion functions to do the job
  toMsg(tmp, tmp_vector1);
  
  // perform the filtering
  filters.update(tmp_vector1, tmp_vector2);

  // copy the filtered velocities back into the resulting JntArray
  // again use one of our message-conversion functions
  fromMsg(tmp_vector2, chi_dot);

  // update chi_old
  chi_old = chi;
}
