
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

#include <filters/filter_chain.h>

#include <ReflexxesAPI.h>

#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Analysis.h>

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
  KDL::JntArray p_gains;
  KDL::JntArray d_gains;

  // output variables
  KDL::JntArray chi;         //!< actual values of the constraints
  KDL::JntArray chi_desired; //!< desired values of the constraints i.e. 'controller setpoints'
  KDL::JntArray chi_dot_desired; //< desired velocities of the constraints

  KDL::Jacobian Ht;  //!< Interaction matrix
  KDL::Frame frame;  //!< just a buffer variable. remembers the last input of update()
  KDL::JntArray ydot;  //!< output velocities (in constraint coordinates)
  KDL::JntArray weights;

  // state variables used for estimation of constraint velocities
  KDL::JntArray last_chi; //< the constraint values from the last control cycle
  KDL::JntArray chi_dot; //< estimated constraint velocities

  // filter chain to smooth estimated constraint velocities
  filters::MultiChannelFilterChain<double> filter_chain; //< low-pass filters for estimated constraint velocities

  // trajectory generators from REFLEXXES motion library and their input/output containers
  ReflexxesAPI* command_trajectory_generator;
  RMLPositionInputParameters* command_trajectory_input;
  RMLPositionOutputParameters* command_trajectory_output;
  RMLPositionFlags command_trajectory_flags;

  // extra data
  KDL::Jacobian J;
  KDL::JntArray singularValues;
  KDL::JntArray tmp;
  Analysis analysis;
  std::vector<double> tmp_vector1, tmp_vector2; //!< input and output containers for filter_chain

  Controller(): filter_chain("double"), command_trajectory_generator(NULL),
    command_trajectory_input(NULL), command_trajectory_output(NULL)
  {}

  ~Controller()
  {
    delete command_trajectory_generator;
    delete command_trajectory_input;
    delete command_trajectory_output;
  }

  // does non-realtime preparation work
  // assumes that constraints have been set
  // also assumes that the parameter server holds suitable
  // filter parameters at given filter_namespace
  // returns true if no error occurred
  bool prepare(std::string& filter_namespace);

  // calls deriveConstraints(), control() and analyzeH()
  void update(KDL::Frame& frame, bool with_control, double dt);

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

    \param ydot            [out] desired velocities in constraint space (DOF)
    \param weights         [out] constraint weights for the solver (DOF)
    \param chi_desired     [out] desired values of the constraints (DOF)
    \param chi_dot_desired [out] desired velocities of the constraints (DOF)
    \param chi     [in] current value of the constraints (DOF)
    \param chi_dot [in] current velocities of the constraints (DOF)
    \param command [in] desired ranges of the commands, used to calculate the weights (DOF)
    \param trajectory_generator [in] ReflexxesAPI object for interpolation (DOF)
    \param trajectory_input     [in] InputObject for ReflexxesAPI; just for memory purposes (DOF)
    \param trajectory_output    [in] OutputObject for ReflexxesAPI; just for memory purposes (DOF)
    \param trajectory_flags     [in] FlagObject for ReflexxesAPI; just for memory purposes (no size)
    \param tmp     [in] some memory for calculations (DOF)
    \param p_gains [in] K_p for the PD-controller that is active when
                        a constraint is not fulfilled. (DOF)
    \param d_gains [in] K_d for the PD-controller that is active when
                        a constraint is not fulfilled. (DOF)
*/
void control(KDL::JntArray& ydot,
             KDL::JntArray& weights,
	     KDL::JntArray& chi_desired,
             KDL::JntArray& chi_dot_desired,
             const KDL::JntArray& chi,
	     const KDL::JntArray& chi_dot,
             const Ranges& command,
             ReflexxesAPI& trajectory_generator,
             RMLPositionInputParameters& trajectory_input,
             RMLPositionOutputParameters& trajectory_output,
             RMLPositionFlags& trajectory_flags,
             KDL::JntArray& tmp,
             const KDL::JntArray p_gains,
             const KDL::JntArray d_gains);


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
            

/* Auxiliary function to estimate velocity of JntArray through numerical differentiation.
   NOTE: ALL JntArrays and the filter_chain are assumed to be of equal size.
   NOTE: chi_old will be update to the values in chi
   \param chi [in] current values of constraints
   \param chi_old [in and out] last values of constraints; will be set to chi
   \param dt [in] time passed between chi and chi_old
   \param filters [in] filter chain used to low-pass filter estimate velocities
   \param chi_dot [out] estimated constraint velocities
   \param tmp [in] some memory for intermediate results
   \param tmp_vector1 [in] some memory to talk to the filter
   \param tmp_vector2 [in] some more memory to talk to the filter
*/
void estimateVelocities(const KDL::JntArray& chi, KDL::JntArray& chi_old, double dt,
                        filters::MultiChannelFilterChain<double>& filters,
                        KDL::JntArray& chi_dot, KDL::JntArray& tmp,
                        std::vector<double>& tmp_vector1, std::vector<double>& tmp_vector2);
 
#endif //FEATURE_CONSTRAINTS_CONTROLLER_H

