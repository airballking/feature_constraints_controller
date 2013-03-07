
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

  Ht.resize(n);
  ydot.resize(n);
  weights.resize(n);

  gains.resize(n);
  command.resize(n);
  intermediate_command.resize(n);

  J.resize(n);
  singularValues.resize(6);

  tmp.resize(n);
  analysis.resize(n);  // maximum number of constraints

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
    // interpolate command
    interpolateCommand(chi, command, dt, intermediate_command);
    control(ydot, weights, chi_desired, chi, command, intermediate_command, gains);
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
             const KDL::JntArray& chi,
             const Ranges& command,
             const Ranges& intermediate_command,
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

    double lo = intermediate_command.pos_lo(i);
    double hi = intermediate_command.pos_hi(i);

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

    // calc ouput weights based on desired final weights
    lo = command.pos_lo(i);
    hi = command.pos_hi(i);

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

void interpolateCommand(const KDL::JntArray& chi, const Ranges& command,
                        double delta_time, Ranges& intermediate_command)
{
  // do sanity checks on inputs and ouputs
  unsigned int size = chi.rows();
  assert(command.pos_lo.rows() == size);
  assert(command.pos_hi.rows() == size);
  assert(command.weight.rows() == size);
  assert(command.max_vel.rows() == size);
  assert(command.min_vel.rows() == size);
  assert(intermediate_command.pos_lo.rows() == size);
  assert(intermediate_command.pos_hi.rows() == size);
  assert(intermediate_command.weight.rows() == size);
  assert(intermediate_command.max_vel.rows() == size);
  assert(intermediate_command.min_vel.rows() == size);
  assert(delta_time > 0.0);

  for(unsigned int i=0; i<command.pos_lo.rows(); i++)
  {
    // copy the the final command to the intermediate command
    // because in some cases it's just that AND the max/min velocities and
    // weights just stay untouched
    // TODO: write assignment operator for Ranges
    intermediate_command.min_vel(i) = command.min_vel(i);
    intermediate_command.max_vel(i) = command.max_vel(i);
    intermediate_command.weight(i) = command.weight(i);
    intermediate_command.pos_lo(i) = command.pos_lo(i);
    intermediate_command.pos_hi(i) = command.pos_hi(i);

    // do the actual interpolation
    if(chi(i) <= command.pos_lo(i) || chi(i) >= command.pos_hi(i))
    {
      // we're not there yet --> need to interpolate
      assert(command.pos_hi(i) >= command.pos_lo(i));
      // TODO: check for too small ranges, i.e. equalities

      // calculate range of final constraint
      double range = command.pos_hi(i) - command.pos_lo(i);

      // project range around current value to set up 'virtual' range
      intermediate_command.pos_lo(i) = chi(i) - 0.5*range;
      intermediate_command.pos_hi(i) = chi(i) + 0.5*range;

      // calculate the delta between the 'virtual' and the end range
      double delta = command.pos_hi(i) - intermediate_command.pos_hi(i);

      // move the intermediate towards the desired end range BUT
      // not more than the maximum velocities allow AND
      // not further than the desired range...
      // TODO: refactor this using one variable for the min/max_vel
      if(delta < 0.0)
      {
        // we have to go negative, i.e. min_vel applies
        if(delta < command.min_vel(i)*delta_time)
        {
          // the error is too big, we need to calculate an intermediate range
          // using the minimum velocity
          intermediate_command.pos_lo(i)+=command.min_vel(i)*delta_time;
          intermediate_command.pos_hi(i)+=command.min_vel(i)*delta_time;
        }
        else
        {
          // the error is not too big, just add the delta --> set original command
          intermediate_command.pos_lo(i)+=delta;
          intermediate_command.pos_hi(i)+=delta;
        }
      }
      else
      {
        // we have to go positive, i.e. max_vel applies
        if(delta > command.max_vel(i)*delta_time)
        {
          // the error is too big, we need to calculate an intermediate range
          // using the maximum velocity
          intermediate_command.pos_lo(i)+=command.max_vel(i)*delta_time;
          intermediate_command.pos_hi(i)+=command.max_vel(i)*delta_time;

        }
        else
        {
          // the error is not too big, just add the delta --> set original command
          intermediate_command.pos_lo(i)+=delta;
          intermediate_command.pos_hi(i)+=delta;
        }
      }
    }
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
