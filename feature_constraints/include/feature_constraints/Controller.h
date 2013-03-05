
/*! \file Controller.h
 *
 *  \brief Range-based controller and Integration
 */

#ifndef FEATURE_CONSTRAINTS_CONTROLLER_H
#define FEATURE_CONSTRAINTS_CONTROLLER_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <vector>

#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Analysis.h>


#define MAX_CONSTRAINTS 64


//! Describes the instantaneous constraint command
class Ranges
{
public:
  Ranges(int size=0);
  void resize(int size);
  KDL::JntArray pos_lo;  //!< min values for the constraints
  KDL::JntArray pos_hi;  //!< max values for the constraints
  KDL::JntArray weight;  //!< weights for the constraints (0|1)
  KDL::JntArray max_vel; //!< max velocities for the constraints
  KDL::JntArray min_vel; //!< min velocities for the constraints
};


//////////////////////////////////////////////////////////////////////////////


//! Convenience class
/*! Collects all variables of interest for a feature-based task function.
 *
 *  This controller is configured by the feature-based constraints
 *  that are stored in \ref constraints. At realtime, \ref command ranges and \ref frame
 *  are updated and the interaction matrix \ref Ht, the output \ref ydot
 *  (expressed in constraint coordinates) and \ref weights are computed.
 */
class Controller
{
public:
  // input variables
  std::vector<Constraint> constraints;
  Ranges command, intermediate_command;
  KDL::JntArray gains;

  // output variables
  KDL::JntArray chi;         //!< actual values of the constraints
  KDL::JntArray chi_desired; //!< desired values of the constraints i.e. 'controller setpoints'

  KDL::Jacobian Ht;  //!< Interaction matrix
  KDL::Frame frame;  //!< just a buffer variable. remembers the last input of update()
  KDL::JntArray ydot;  //!< output velocities (in constraint coordinates)
  KDL::JntArray weights;

  // extra data
  KDL::Jacobian J;
  KDL::JntArray singularValues;
  KDL::JntArray tmp;
  Analysis analysis;

  // does non-realtime preparation work
  // assumes that either constraints is set
  // or max_constraints is given
  void prepare(int max_constraints=-1);

  // calls deriveConstraints(), control() and analyzeH()
  void update(KDL::Frame& frame, bool with_control);

  // calls clamp(KDL::JntArray, const KDL::JntArray, const KDL::JntArray)
  void clampOutput();
};


//////////////////////////////////////////////////////////////////////////////


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

//! Do range-based control with intermediate goal.
/*! This controller accepts ranges of accepted positions and
    lowers the weight to zero when inside that range.
    It also takes an intermediate (interpolated) range as
    input which is used to calculate to control error.

    \param ydot    [out] desired velocities in constraint space
    \param weights [out] constraint weights for the solver
    \param chi_desired [out] desired values of the constraints
(a border of the range when not fulfilled, equal to chi otherwise)
    \param chi     [in] current value of the constraints
    \param command [in] desired ranges of the commands, used to calculate the weights
    \param intermediate_command [in] interpolated ranges of the commands, used to
                                     calculate the control error and ydot
    \param gains   [in] K_p for the P-controller that is active when
                        a constraint is not fulfilled.
 */
void control(KDL::JntArray& ydot,
             KDL::JntArray& weights,
	     KDL::JntArray& chi_desired,
             const KDL::JntArray& chi,
	     const Ranges& command,
             const Ranges& intermediate_command,
             const KDL::JntArray gains);


/* Auxiliary convenience function to clamp --for example-- output
   velocities that come from a controller.
   \param joint_velocities [in AND out] the original velocities to clamp
   \param min_velocities [in] desired lower velocity boundaries
   \param max_velocities [in] desired upper velocity boundariess
*/
void clamp(KDL::JntArray& joint_velocities,
           const KDL::JntArray& min_velocities,
           const KDL::JntArray& max_velocities);

double clamp(double input_velocity,
             double min_velocity,
             double max_velocity);
            
/* Auxiliary function to interpolate constraints between feature function output
   and desired final feature constraints.
   \param chi [in] the current output value of the feature functions
   \param command [in] desired final constraints for the movement
   \param delta_time [in] time between two control cycles
   \param intermediate_command [out] interpolation constraint to get us to final
*/
void interpolateCommand(const KDL::JntArray& chi, const Ranges& command, 
                        double delta_time, Ranges& intermediate_command);
 
#endif //FEATURE_CONSTRAINTS_CONTROLLER_H

