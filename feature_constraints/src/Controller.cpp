
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

void Ranges::reset(double maximum_vel)
{
  for(unsigned int i=0; i < pos_lo.rows(); i++)
    pos_lo(i) = -HUGE_VAL;
  for(unsigned int i=0; i < pos_hi.rows(); i++)
    pos_hi(i) =  HUGE_VAL;
  for(unsigned int i=0; i < weight.rows(); i++)
    weight(i) =  0.0;
  for(unsigned int i=0; i < max_vel.rows(); i++)
    max_vel(i) =  maximum_vel;
  for(unsigned int i=0; i < min_vel.rows(); i++)
    min_vel(i) = -maximum_vel;
}

unsigned int Ranges::size() const
{
  assert(pos_lo.rows() == pos_hi.rows());
  assert(pos_lo.rows() == weight.rows());
  assert(pos_lo.rows() == max_vel.rows());
  assert(pos_lo.rows() == min_vel.rows());

  return pos_lo.rows();
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
  KDL::SetToZero(chi_dot);
  tmp_vector1.resize(n);
  tmp_vector2.resize(n);
  filter_chain.clear();
  return filter_chain.configure(n, filter_namespace);
}


void Controller::update(KDL::Frame& frame, bool with_control, double dt)
{
  differentiateConstraints(Ht, chi, frame, constraints, 0.001, tmp);
  //estimateVelocities(chi, last_chi, dt, filter_chain, chi_dot, command.max_vel,
  //  tmp, tmp_vector1, tmp_vector2);
  if(with_control)
  {
    //control(ydot, weights, chi_desired, chi, command, p_gains);

    control(ydot, weights, chi_desired, chi_dot_desired, chi, chi_dot, command,
      *command_trajectory_generator, *command_trajectory_input, *command_trajectory_output,
      command_trajectory_flags, tmp, p_gains, d_gains);
  }
  analysis.analyzeH(Ht, J, singularValues, 1e-7);
  this->frame = frame;

  // remember chi_dot_desired as chi_dot for next control cycle
  chi_dot = chi_dot_desired;
}

void Controller::clampOutput()
{
  clamp(ydot, command.max_vel);
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
      chi_desired(i) = chi(i);
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
  trajectory_input.SetMaxVelocityVector(command.max_vel.data.data());
  KDL::Multiply(command.max_vel, 2.0, tmp);
  trajectory_input.SetMaxAccelerationVector(tmp.data.data()); // implies we reach max_vel within 0.5sec (with max_jerk-->infinity)
  KDL::Multiply(command.max_vel, 2.0, tmp);
  trajectory_input.SetMaxJerkVector(tmp.data.data()); // implies we reach max_acc within 0.5sec

  // reflexxes library: explicitedly enable all trajectory generators
  for(unsigned int i=0; i<chi.rows(); i++)
  {
    trajectory_input.SetSelectionVectorElement(true, i);
  }
  
  // reflexxes library: set current constraint velocities and position
  trajectory_input.SetCurrentPositionVector(chi.data.data());
  trajectory_input.SetCurrentVelocityVector(chi_dot.data.data());

  // our algorithm: determine chi_desired and weights based on ranges 
  double s = 0.05;
  for(unsigned int i=0; i < chi.rows(); i++)
  {
    if(command.weight(i) == 0.0)
    {
      ydot(i) = 0.0;
      weights(i) = 0.0;
      chi_desired(i) = chi(i);
      continue;
    }

    double value = chi(i);

    double lo = command.pos_lo(i);
    double hi = command.pos_hi(i);

    // adjust margin if range is too small
    double ss = (hi - lo < 2*s) ? (hi - lo) / 10 : s;

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
    // printing chi (current position)
    std::cout << "\nchi(";
    for(unsigned int i=0; i<chi.rows(); i++)
    {
      std::cout << " " << chi(i);
    }
    std::cout << ")\n";
    // printing chi_dot (current velocity)
    std::cout << "\nchi_dot(";
    for(unsigned int i=0; i<chi_dot.rows(); i++)
    {
      std::cout << " " << chi_dot(i);
    }
    std::cout << ")\n";
    // printing chi_desired (desired position)
    std::cout << "\nchi_desired(";
    for(unsigned int i=0; i<chi_desired.rows(); i++)
    {
      std::cout << " " << chi_desired(i);
    }
    std::cout << ")\n";
    // printing chi_dot (desired velocity)
    std::cout << "\nchi_dot_desired(";
    for(unsigned int i=0; i<chi_dot_desired.rows(); i++)
    {
      std::cout << " " << chi_dot_desired(i);
    }
    std::cout << ")\n";

    KDL::SetToZero(ydot);
    return;
  }

  // reflexess library copy results into chi_desired and chi_dot_desired
  trajectory_output.GetNewPositionVector(chi_desired.data.data(), chi_desired.rows()*sizeof(double));
  trajectory_output.GetNewVelocityVector(chi_dot_desired.data.data(), chi_dot_desired.rows()*sizeof(double));

  // perform control to calculate ydot
  for(unsigned int i=0; i<chi.rows(); i++)
  {
    double hi = command.pos_hi(i);
    double lo = command.pos_lo(i);
    double s = 0.05;
    double ss = (hi - lo < 2*s) ? (hi - lo) / 10 : s;

    // calculate ydot-control signal
    // note: if we are in 'reduced' constraint range the constraint function should not change  
    if(chi(i) < (hi - ss) && chi(i) > (lo + ss))
    {
      ydot(i) = 0.0;
    }
    else
    {
      ydot(i) = p_gains(i)*(chi_desired(i) - chi(i)) + d_gains(i)*(chi_dot_desired(i) - chi_dot(i));
    }
  }
}

double clamp(double input_velocity, double max_velocity)
{
  assert(max_velocity >= 0.0);

  if(std::fabs(input_velocity) > max_velocity)
    return input_velocity*max_velocity/(std::fabs(input_velocity));

  // nothing to clamp
  return input_velocity;
}

void clamp(KDL::JntArray& joint_velocities, const KDL::JntArray& max_velocities)
{
  assert(joint_velocities.rows() == max_velocities.rows());

  for(unsigned int i=0; i<joint_velocities.rows(); i++)
  {
    joint_velocities(i) = clamp(joint_velocities(i), max_velocities(i));
  }
}

void estimateVelocities(const KDL::JntArray& chi, KDL::JntArray& chi_old, double dt,
                        filters::MultiChannelFilterChain<double>& filters,
                        KDL::JntArray& chi_dot, const KDL::JntArray& chi_dot_max, KDL::JntArray& tmp,
                        std::vector<double>& tmp_vector1, std::vector<double>& tmp_vector2)
{
  assert(chi.rows() == chi_old.rows());
  assert(chi.rows() == chi_dot.rows());
  assert(chi.rows() == tmp.rows());
  assert(chi.rows() == tmp_vector1.size());
  assert(chi.rows() == tmp_vector2.size());
  assert(chi.rows() == chi_dot_max.rows());

  // calculate instantaneous velocities through numerical differentiation
  // NOTE: in the next line I'm using chi_dot as a second 'tmp'
  KDL::Subtract(chi, chi_old, chi_dot);
  KDL::Divide(chi_dot, dt, tmp);
  chi_dot = tmp;

  // clamp estimated velocities to something reasonable, i.e. a multiple of max_velocities.
  // note, this is workaround for some sophisticated velocity-estimation but it was
  // necessary because sometimes there were numerial problems, i.e. extremely high estimated
  // instantaneous velocities.
  // note, tmp is holding the higher velocity boundaries.
  KDL::Multiply(chi_dot_max, 22.0, tmp);
  clamp(chi_dot, chi_dot_max);

  // copy the clamped velocity estimates into a std::vector to send it to the filters
  // use one of our existing message-conversion functions to do the job
  toMsg(chi_dot, tmp_vector1);
  
  // perform the filtering
  filters.update(tmp_vector1, tmp_vector2);
  
  // copy the filtered velocities back into the resulting JntArray
  // again use one of our message-conversion functions
  fromMsg(tmp_vector2, chi_dot);

  // update chi_old
  chi_old = chi;
}
