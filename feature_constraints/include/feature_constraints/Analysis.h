#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <feature_constraints/FeatureConstraints.h>


class Quat
{
public:
  Quat() : x(0), y(0), z(0), w(1) {}
  Quat(double x, double y, double z, double w)
    : x(x), y(y), z(z), w(w) {}
  double x,y,z,w;
};


//! pseudo-inverse working matrices
class PinvData
{
public:
  PinvData(int size=6);
  Eigen::MatrixXd U, V;
  Eigen::VectorXd Sp, tmp;
  void resize(int size);
};


//! Analyze interaction matrix.
/*! This function computes the pseudoinverse of the interaction matrix H
    using the singular value decomposition. It returns the inverse
    as well as the singular values.
    The singular values reveal the rank of Ht, showing whether
    constraints are conflicting.

    This feature is currently unused and is not required for control.
 */
void analyzeH(PinvData& tmpdata,
              const KDL::Jacobian& Ht,
              KDL::Jacobian& J,
              KDL::JntArray& singularValues,
              double eps=1e-15);



/*! \brief Return the number of independent DOF for the given constraints.
 *
 *  This function repeatedly computes the interaction matrix for different
 *  frames and determines the rank of the interaction matrix (using the
 *  threshold eps).
 *  The maximum rank found is returned.
 */
int rank(const std::vector<Constraint> &constraints,
         double dd, double eps);



/*  \brief checks if the given constraint is continuous at frame.
 */
double discontinuity(const Constraint& constraint, const KDL::Frame& frame,
                  double dd);



/* \brief Enumerate all 24 rotations where the transformed axes are aligned
   with the original axes.
 */
KDL::Frame axis_sampler(int index);


std::vector< std::pair<Quat, double> >
  continuityPlotRPY(Constraint c, KDL::Frame offset,
                    int numSamples, double dd, double threshold);


#endif // ANALYSIS_H
