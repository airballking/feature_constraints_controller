
#include <stdlib.h>
#include <feature_constraints/Analysis.h>

#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>


using namespace KDL;
using namespace Eigen;
using namespace std;

double random_number(double min, double max)
{
  return (max - min) * rand() / RAND_MAX + min;
}


KDL::Frame random_frame(double trans_range=10.0, double rot_range=6.28)
{
  return KDL::Frame(Rotation::RotX(random_number(-rot_range, rot_range))*
                    Rotation::RotY(random_number(-rot_range, rot_range))*
                    Rotation::RotZ(random_number(-rot_range, rot_range)),
                    KDL::Vector(random_number(-trans_range, trans_range),
                                random_number(-trans_range, trans_range),
                                random_number(-trans_range, trans_range)));
}

PinvData::PinvData(int size)
{
  resize(size);
}

void PinvData::resize(int size)
{
  U.resize(size, 6);
  V.resize(6,6);
  Sp.resize(6);
  tmp.resize(6);
}


void analyzeH(PinvData& tmp, const KDL::Jacobian& Ht, KDL::Jacobian& J, KDL::JntArray& singularValues, double eps)
{
  VectorXd& S = singularValues.data;

  svd_eigen_HH(Ht.data.transpose(), tmp.U, S, tmp.V, tmp.tmp);

  for(int i=0; i < 6; ++i)
      tmp.Sp(i) = (fabs((double) S(i)) > eps) ? 1.0 / S(i) : 0.0;

  J.data = tmp.V * tmp.Sp.asDiagonal() * tmp.U.transpose();
}


int rank(const std::vector<Constraint> &constraints,
         double dd, double eps)
{
  int size = constraints.size();
  JntArray values(size), tmp(size), lambda(size);
  Jacobian Ht(size), J(size);
  PinvData tmp_pinv(size);

  int min_rank=size;
  int max_rank=0;

  for(int iter=0; iter < 1000; iter++)
  {
    Frame f = random_frame();
    differentiateConstraints(Ht, values, f, constraints, dd, tmp);
    analyzeH(tmp_pinv, Ht, J, lambda, eps);

    int rank=0;
    for(int i=0; i < size; i++)
      if(lambda(i) > eps)
        rank++;

    min_rank = (rank < min_rank) ? rank : min_rank;
    max_rank = (rank > max_rank) ? rank : max_rank;
  }
  return max_rank;
}


bool continuous(const Constraint& constraint, KDL::Frame& frame,
                double dd, double threshold)
{
  std::vector<Constraint> constraints;
  constraints.push_back(constraint);

  JntArray value(1), tmp(1);
  Jacobian Ht(1);

  differentiateConstraints(Ht, value, frame, constraints, dd, tmp);

  for(int i=0; i < 6; i++)
    if(Ht(i, 0) > threshold)
      return false;

  return true;
}


/*! \brief Create all 24 axis-aligned rotations.
 *
 *  Create all frames that can be reached by repeatedly
 *  rotating 90deg around any coordinate axis.
 *
 *  Algorithm: Align x-axis with any of (+x, -x, +y, -y, +z, -z) (upper 3 bits)
 *  and then rotate around x 4 times (lower 2 bits). (6*4=24 orientations).
 */
KDL::Frame axis_sampler(int index)
{
  int x_axis = index >> 2;
  int x_rot  = index & 3;

  Frame f;
  if(x_axis < 4)
    f = Frame(Rotation::RotY(M_PI/2 * x_axis));
  else if(x_axis == 4)
    f = Frame(Rotation::RotZ(-M_PI/2));
  else if(x_axis == 5)
    f = Frame(Rotation::RotZ(M_PI/2));
  
  return f * Frame(Rotation::RotX(M_PI/2 * x_rot));
}

