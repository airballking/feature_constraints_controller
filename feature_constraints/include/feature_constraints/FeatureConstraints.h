
/*!
 *  \file FeatureConstraints.h
 *
 *  \brief Features and Constraints
 *
 *  This file contains the feature and constraint classes, some
 *  constraint functions and the evaluation and differentiation of
 *  constraints.
 */


/*! \mainpage
 *
 *  This package contains code to define geometric \ref Feature "Features"
 *  and combine them using \ref Constraint "Constraints". These constraints
 *  are task functions, which can be numerically \ref differentiateConstraints
 *  "differentiated" and thus used in a \ref Controller. The
 *  \ref control "control law" that is implemented here can keep the
 *  constraints inside desired \ref Ranges "ranges".
 */


#ifndef FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H
#define FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>
#include <vector>
#include <map>

// In theory, having strings inside features, screws its realtime safety for
// creation! (there is _no_ copy-on-write implementation for the GNU std::string)
// For the time being, we just reserve 256 bytes for every string. For longer names,
// realtime may get a hickup. 

#define STRING_SIZE 256


//! A tool- or workspace feature
class Feature
{
public:
  int type;
  std::string name;        //!< name of the feature
  KDL::Vector pos;         //!< position of the feature
  KDL::Vector dir;         //!< direction of the feature
  KDL::Vector contact_dir; //!< direction of 'free space'

  Feature() {name.reserve(STRING_SIZE);}
  Feature(std::string name, KDL::Vector pos,
          KDL::Vector dir, KDL::Vector contact_dir=KDL::Vector())
    : name(name), pos(pos), dir(dir), contact_dir(contact_dir) {name.reserve(STRING_SIZE);}
};

/*! \typedef ConstraintFunc
    \brief Constraint function.

    This task function takes two features and the
    transform between these features and computes a real value.

    \param frame          The transform from tool to object
    \param tool_feature   A geometric feature which is attached to the tool
    \param tool_feature   A geometric feature which is attached to the object
*/
typedef double (*ConstraintFunc) (KDL::Frame& frame, Feature tool_feature, Feature object_feature);


//! A constraint between two features
/*! A constraint consists of a feature on the object, a feature on the tool and
    a constraint function.
*/
class Constraint
{
public:
  std::string name;       //!< Name of the constraint
  ConstraintFunc func;    //!< Constraint function
  Feature tool_feature;   //!< A feature that is attached to the tool
  Feature object_feature; //!< A feature that is attached to the object

  Constraint() {name.reserve(STRING_SIZE);}

  Constraint(std::string name, ConstraintFunc func,
             Feature tool_feature, Feature object_feature)
   : name(name), func(func), tool_feature(tool_feature), object_feature(object_feature)
  {
	name.reserve(STRING_SIZE);
  }

  Constraint(std::string name, std::string func_name,
             Feature tool_feature, Feature object_feature)
   : name(name), func(constraint_functions_[func_name]), tool_feature(tool_feature), object_feature(object_feature)
  {
	name.reserve(STRING_SIZE);
  }


  //! evaluate this constraint, given the transform between object and tool
  double operator() (KDL::Frame frame) const
  {
    return func(frame, tool_feature, object_feature);
  }


  //! set the constraint function by it's name
  void setFunction(const std::string& functionName)
    { func = Constraint::constraint_functions_[functionName]; }

  //// group: static members

  //! set the possible constraint functions
  static void init();
  //! A mapping from constraint function name to the constraint function itself.
  static std::map<std::string, ConstraintFunc> constraint_functions_;
};


//////////////////////////////////////////////////////////////////////////////

/*! @name Constraint Functions
 */

///@{

/*! zero when perpendicular (cos of the angle)
 */
double perpendicular(KDL::Frame& frame, Feature tool_feature, Feature object_feature);


/*! distance between features, projected onto the
    the direction of the object feature
 */
double height(KDL::Frame& frame, Feature tool_feature, Feature object_feature);


/*! distance between features, perpendicular to
    the direction of the object feature
 */
double distance(KDL::Frame& frame, Feature tool_feature, Feature object_feature);


/*! angle between tool feature direction and
    connecting line between object and tool positions
    (seen perpendicular to the object direction)
 */
double pointing_at(KDL::Frame& frame, Feature tool_feature, Feature object_feature);


/*! cosine of the angle of cylinder coordinates.
 */
double direction(KDL::Frame& frame, Feature tool_feature, Feature object_feature);


/*! always returns 0
 */
double null(KDL::Frame& frame, Feature tool_feature, Feature object_feature);

///@}

//////////////////////////////////////////////////////////////////////////////


//! Compute the values of the constraints, given the transform between tool and object.
/*!
    \param values      [out] The computed values of the constraints
    \param frame       [in]  The transform from tool to object
    \param constraints [in]  The constraints that shall be evaluated
*/
void evaluateConstraints(KDL::JntArray &values,
                         const KDL::Frame& frame,
                         const std::vector<Constraint>& constraints);


//! Differentiate n constraints numerically around 'frame'.
/*! The resulting interaction matrix H is transposed so a
    KDL::Jacobian can be used.

   \param Ht     [out] Transposed interaction matrix
   \param values [out] The values of the constraints at 'frame'. (must have n rows)
   \param frame  [in]  The transform between tool and object where the constraints shall be evaluated
   \param constraints [in] The constraints to be evaluated
   \param dd     [in]  How far frame should be moved in order to obtain Ht.
   \param tmp    [out] Temporary variable (must have n rows)
*/
void differentiateConstraints(KDL::Jacobian& Ht,
                              KDL::JntArray& values,
                              const KDL::Frame& frame,
                              const std::vector<Constraint> &constraints,
                              double dd,
                              KDL::JntArray& tmp);

#endif //FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H
