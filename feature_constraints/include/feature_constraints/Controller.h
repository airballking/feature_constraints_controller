
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


void control(KDL::JntArray& ydot, KDL::JntArray& weights,
	     KDL::JntArray& chi_desired, KDL::JntArray& chi,
	     Ranges& command, KDL::JntArray gains);

///////////////////////////////


// convenience class
// collects all variables of interest for a feature-based task function.
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
  // assumes that constraints is set
  void prepare(int max_constraints=MAX_CONSTRAINTS);

  // calls deriveConstraints(), control() and analyzeH()
  void update(KDL::Frame frame);
};


//! Analyze interaction matrix.
void analyzeH(PinvData& tmpdata, KDL::Jacobian Ht, KDL::Jacobian& J,
		      KDL::JntArray& singularValues, double eps=1e-15);

//! Do range-based control.
void control(KDL::JntArray& ydot, KDL::JntArray& weights,
	     KDL::JntArray& chi_desired, KDL::JntArray& chi,
	     Ranges& command, KDL::JntArray gains);


#endif //FEATURE_CONSTRAINTS_CONTROLLER_H
