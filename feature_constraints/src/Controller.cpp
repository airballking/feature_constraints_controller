
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>

#include <Eigen/Core>


using namespace KDL;
using namespace Eigen;


Ranges::Ranges(int size) :
  pos_lo(size), pos_hi(size), weight(size)
{

}

void Ranges::resize(int size)
{
  pos_lo.resize(size);
  pos_hi.resize(size);
  weight.resize(size);
}


void Controller::prepare(int max_constraints)
{
  // TODO(Ingo & Georg): Discuss whether this is still necessary? Or
  //     whether the semantics of these variables have changed because
  //     right now prepare(n) will be called with every new Config-msg.
  int n = (max_constraints == -1) ? constraints.size() : max_constraints;

  chi.resize(n);
  chi_desired.resize(n);

  Ht.resize(n);
  ydot.resize(n);
  weights.resize(n);

  gains.resize(n);
  command.resize(n);

  J.resize(n);
  singularValues.resize(6);

  tmp.resize(n);
  analysis.resize(n);  // maximum number of constraints
}


void Controller::update(KDL::Frame& frame)
{
  differentiateConstraints(Ht, chi, frame, constraints, 0.001, tmp);
  control(ydot, weights, chi_desired, chi, command, gains);
  analysis.analyzeH(Ht, J, singularValues, 1e-7);
  this->frame = frame;
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

