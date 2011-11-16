
#ifndef FEATURE_CONSTRAINTS_CONTROLLER_H
#define FEATURE_CONSTRAINTS_CONTROLLER_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <vector>

#include <feature_constraints/FeatureConstraints.h>

// describes the instantaneous constraint command
class Ranges
{
public:
  Ranges(int size);
  KDL::JntArray pos_lo;  //!< min values for the constraints
  KDL::JntArray pos_hi;  //!< max values for the constraints
  KDL::JntArray weight;  //!< weights for the constraints (0|1)
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

  // does non-realtime preparation work
  // assumes that constraints is set
  void prepare();

  // calls deriveConstraints(), control() and analyzeH()
  void update(KDL::Frame frame);
};

//! Analyze interaction matrix.
void analyzeH(KDL::Jacobian Ht, KDL::Jacobian& J, KDL::JntArray& singularValues, double eps=1e-15);

void control(KDL::JntArray& ydot, KDL::JntArray& weights,
	     KDL::JntArray& chi_desired, KDL::JntArray& chi,
	     Ranges& command, KDL::JntArray gains);


#endif //FEATURE_CONSTRAINTS_CONTROLLER_H
