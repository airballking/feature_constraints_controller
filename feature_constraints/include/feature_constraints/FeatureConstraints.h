
#ifndef FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H
#define FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>
#include <vector>
#include <map>

// TODO: having strings inside features, screws its realtime safety for creation! is that bad?
//       (_no_ copy-on-write implementation for the GNU std::string.  :-( )

#define STRING_SIZE 256


// A tool- or workspace feature
class Feature
{
public:
  int type;
  std::string name;
  KDL::Vector pos;     //!< position of the feature
  KDL::Vector dir;     //!< direction of the feature

  Feature() {name.reserve(STRING_SIZE);}
  Feature(std::string name, KDL::Vector pos,
          KDL::Vector dir, KDL::Vector dir2=KDL::Vector())
    : name(name), pos(pos), dir(dir) {name.reserve(STRING_SIZE);}
};


typedef double (*FeatureFunc) (KDL::Frame& frame, Feature* tool_feature, Feature* object_feature);


//! A constraint can have up to two object- and tool features.
//! For some Feature-functions one single feature is not enough (namely 'angle' requires two object features)
//! With two features, however, a coordinate frame can be fully defined, so more
//! is unnecessary.

class Constraint
{
public:
  std::string name;
  FeatureFunc func;
  Feature tool_features[2], object_features[2];

  Constraint() {name.reserve(STRING_SIZE);}
  Constraint(std::string name, FeatureFunc func,
             Feature tool_feature, Feature object_feature)
   : name(name), func(func)
  {
	name.reserve(STRING_SIZE);
    tool_features[0] = tool_feature;
    object_features[0] = object_feature;
  }

  Constraint(std::string name, std::string func_name,
             Feature tool_feature, Feature object_feature)
   : name(name), func(feature_functions_[func_name])
  {
	name.reserve(STRING_SIZE);
    tool_features[0] = tool_feature;
    object_features[0] = object_feature;
  }

  double operator() (KDL::Frame frame)
  {
    return func(frame, tool_features, object_features);
  }

#if 0  // currently unnecessary since the pseudoinverse of Ht does a pretty good job at finding this.
  // the place (point of interest) that the constraint moves/turns around.
  KDL::Vector location(KDL::Frame frame)
  {
    return frame * tool_feature_location;
  }
#endif

  static void init();
  void setFunction(const std::string& functionName)
    { func = Constraint::feature_functions_[functionName]; }
  static std::map<std::string, FeatureFunc> feature_functions_;
};

// some feature functions

//! zero when perpendicular (cos of the angle)
double perpendicular(KDL::Frame frame, Feature* tool_features, Feature* object_features);

//! distance between features, projected onto the
//! the direction of the object feature
double height(KDL::Frame frame, Feature* tool_features, Feature* object_features);

//! distance between features, perpendicular to
//! the direction of the object feature
double distance(KDL::Frame frame, Feature* tool_features, Feature* object_features);

//! angle between tool feature direction and
//! connecting line between object and tool positions
//! (seen perpendicular to the object direction)
double pointing_at(KDL::Frame frame, Feature* tool_features, Feature* object_features);

/////

//! compute the values of the constraints, given a frame
void evaluateConstraints(KDL::JntArray &values, KDL::Frame frame, const std::vector<Constraint> &constraints);

//! compute the numerical derivative of the constraints around frame
void deriveConstraints(KDL::Jacobian& Ht, KDL::JntArray& values, KDL::Frame& frame,
                       std::vector<Constraint> &constraints, double dd,  KDL::JntArray& tmp);


#endif //FEATURE_CONSTRAINTS_FEATURE_CONSTRAINTS_H
