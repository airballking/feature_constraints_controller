#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <Eigen/Geometry>
#include <feature_constraints/FeatureConstraints.h>


//! pseudo-inverse working matrices
class PinvData
{
};


//! Analysis of constraint functions
/*! This class holds various temporary variables
    for performance reasons. All 'important' variables
    are passed to the functions.
 */
class Analysis
{
public:
  Analysis(int size=6);
  void resize(int size);

  //! Analyze interaction matrix.
  /*! This function computes the pseudoinverse of the interaction matrix H
      using the singular value decomposition. It returns the inverse
      as well as the singular values.
      The singular values reveal the rank of Ht, showing whether
      constraints are conflicting.

      This feature is currently unused and is not required for control.
  */
  void analyzeH(const KDL::Jacobian& Ht,
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


private:
  std::vector<Constraint> constraints_;
  // for pseudoinverse
  Eigen::MatrixXd U_, V_;
  Eigen::VectorXd Sp_, tmp_, tmp2_;
  // for rank()
  KDL::JntArray values_, tmpa1_, tmpa2_, lambda_;
  KDL::Jacobian Ht_, J_;

};










/* \brief Enumerate all 24 rotations where the transformed axes are aligned
   with the original axes.
 */
KDL::Frame sampler_axis(int index);


KDL::Frame sampler_near(const KDL::Frame& frame, int index, double step);

double discontinuity_near(const Constraint& constraint, const KDL::Frame& frame,
                          double dd, double surrounding, double density);


std::vector< std::pair<Eigen::Quaterniond, double> >
  continuityPlotRPY(Constraint c, KDL::Frame offset,
                    int numSamples, double dd, double threshold);


#endif // ANALYSIS_H
