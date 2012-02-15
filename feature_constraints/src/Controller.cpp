
#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Controller.h>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <Eigen/Core>


using namespace KDL;
using namespace Eigen;


Ranges::Ranges(int size) :
  pos_lo(size), pos_hi(size), weight(size)
{

}


void Controller::prepare(int max_constraints)
{
  int n = constraints.size();

  chi.resize(n);
  chi_desired.resize(n);

  Ht.resize(n);
  ydot.resize(n);
  weights.resize(n);

  J.resize(n);
  singularValues.resize(n);

  tmp.resize(n);
  tmp_pinv.resize(max_constraints);  // maximum number of constraints
}


void Controller::update(KDL::Frame frame)
{
  deriveConstraints(Ht, chi, frame, constraints, 0.001, tmp);
  control(ydot, weights, chi_desired, chi, command, gains);
  analyzeH(tmp_pinv, Ht, J, singularValues, 1e-7);
  this->frame = frame;
}


//! new range-controller.
//! this controller accepts ranges of accepted positions and
//! lowers the weight to zero when inside that range. 
void control(KDL::JntArray& ydot, KDL::JntArray& weights,
	     KDL::JntArray& chi_desired, KDL::JntArray& chi,
	     Ranges& command, KDL::JntArray gains)
{
  double s = 0.05;
  for(int i=0; i < 6; i++)
  {
    if(command.weight(i) == 0.0)
    {
      ydot(i) = 0.0;
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
      weights(i) = 1.0;
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


void PinvData::resize(int size)
{
  U.resize(size, 6);
  V.resize(6,6);
  Sp.resize(6);
  tmp.resize(6);
}


void analyzeH(PinvData& tmp, KDL::Jacobian& Ht, KDL::Jacobian& J, KDL::JntArray& singularValues, double eps)
{
  VectorXd& S = singularValues.data;

  svd_eigen_HH(Ht.data.transpose(), tmp.U, S, tmp.V, tmp.tmp);

  for(int i=0; i < 6; ++i)
      tmp.Sp(i) = (fabs((double) S(i)) > eps) ? 1.0 / S(i) : 0.0;

  J.data = tmp.V * tmp.Sp.asDiagonal() * tmp.U.transpose();
}
