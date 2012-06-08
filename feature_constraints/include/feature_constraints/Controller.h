
#ifndef FEATURE_CONSTRAINTS_CONTROLLER_H
#define FEATURE_CONSTRAINTS_CONTROLLER_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <vector>

#include <feature_constraints/FeatureConstraints.h>


#define MAX_CONSTRAINTS 64


// describes the instantaneous constraint command
class Ranges
{
public:
  Ranges(int size=0);
  void resize(int size);
  KDL::JntArray pos_lo;  //!< min values for the constraints
  KDL::JntArray pos_hi;  //!< max values for the constraints
  KDL::JntArray weight;  //!< weights for the constraints (0|1)
};


//! pseudo-inverse working matrices
class PinvData
{
public:
  Eigen::MatrixXd U, V;
  Eigen::VectorXd Sp, tmp;
  void resize(int size);
};


//////////////////////////////////////////////////////////////////////////////


//! Convenience class
/*! Collects all variables of interest for a feature-based task function.
 */
class Controller
{
public:
  // input variables
  std::vector<Constraint> constraints;
  Ranges command;
  KDL::JntArray gains;

  // output variables
  KDL::JntArray chi;
  KDL::JntArray chi_desired;

  KDL::Jacobian Ht;
  KDL::Frame frame;  //! just a buffer variable. remembers the last input of update()
  KDL::JntArray ydot;
  KDL::JntArray weights;

  // extra data
  KDL::Jacobian J;
  KDL::JntArray singularValues;
  KDL::JntArray tmp;
  PinvData tmp_pinv;

  // does non-realtime preparation work
  // assumes that either constraints is set
  // or max_constraints is given
  void prepare(int max_constraints=-1);

  // calls deriveConstraints(), control() and analyzeH()
  void update(KDL::Frame& frame);
};


//////////////////////////////////////////////////////////////////////////////


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


//! Do range-based control.
/*! This controller accepts ranges of accepted positions and
    lowers the weight to zero when inside that range.

    \param ydot    [out] desired velocities in constraint space
    \param weights [out] constraint weights for the solver
    \param chi_desired [out] desired values of the constraints
(a border of the range when not fulfilled, equal to chi otherwise)
    \param chi     [in] current value of the constraints
    \param command [in] desired ranges of the commands
    \param gains   [in] K_p for the P-controller that is active when
                        a constraint is not fulfilled.
 */
void control(KDL::JntArray& ydot,
             KDL::JntArray& weights,
	     KDL::JntArray& chi_desired,
             const KDL::JntArray& chi,
	     const Ranges& command,
             const KDL::JntArray gains);


#endif //FEATURE_CONSTRAINTS_CONTROLLER_H

