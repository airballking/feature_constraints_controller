#ifndef ANALYSIS_H
#define ANALYSIS_H


#include <feature_constraints/FeatureConstraints.h>

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
bool continuous(const Constraint& constraint, KDL::Frame& frame,
                double dd, double threshold);



/* \brief Enumerate all 24 rotations where the transformed axes are aligned
   with the original axes.
 */
KDL::Frame axis_sampler(int index);


#endif // ANALYSIS_H
