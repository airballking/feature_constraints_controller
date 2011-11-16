#include <feature_constraints/FeatureConstraints.h>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <tf_conversions/tf_kdl.h>


using namespace std;
using namespace KDL;

std::map<std::string, FeatureFunc> Constraint::feature_functions_;

// some feature functions

//! zero when perpendicular
double perpendicular(KDL::Frame& frame, Feature* tool_features, Feature* object_features)
{
  return dot(object_features[0].dir, Frame(frame.M) * tool_features[0].dir);
}

//! distance between features, projected onto the
//! the direction of the object feature
double height(KDL::Frame& frame, Feature* tool_features, Feature* object_features)
{
  Vector &p_o = object_features[0].pos;
  Vector  p_t = frame * tool_features[0].pos;
  Vector &d_o = object_features[0].dir;

  return dot(d_o, p_t - p_o) / d_o.Norm();
}

double distance(KDL::Frame& frame, Feature* tool_features, Feature* object_features)
{
  Vector  p   = frame*tool_features[0].pos - object_features[0].pos;
  Vector &d_o = object_features[0].dir;

  return (p - (dot(d_o, p) / d_o.Norm())*p).Norm();
}

double pointing_at(KDL::Frame& frame, Feature* tool_features, Feature* object_features)
{
  Vector  p   = frame*tool_features[0].pos - object_features[0].pos;
  Vector &d_o = object_features[0].dir;
  Vector  d_t = Frame(frame.M) * tool_features[0].dir;

  // projection of p perpendicular to d_o
  // Vector d = dot(d_o, p) / d_o.Norm() * p;

  // tangent vector of p  w.r.t. a cylinder around d_o
  Vector t = p * d_o;  // cross product
  
  return dot(t, d_t) / (t.Norm()*d_t.Norm());
}


void evaluateConstraints(KDL::JntArray& values, KDL::Frame& frame, std::vector<Constraint>& constraints)
{
  assert(values.rows() == constraints.size());

  for(unsigned int i=0; i < constraints.size(); i++)
    values(i) = constraints[i](frame);
}

/*!
 * \param Ht transposed interaction matrix
 * \param values values of the constraints at frame (must have n rows)
 * \param frame the frame at which the constraints should be evaluated
 * \param dd how far frame should be moved in order to obtain Ht (must have n columns)
 * \param tmp temporary variable (must have n rows)
 * \param constraints the constraints to be evaluated
 */
void deriveConstraints(KDL::Jacobian& Ht, KDL::JntArray& values, KDL::Frame& frame, std::vector<Constraint> &constraints, double dd,  KDL::JntArray& tmp)
{
  assert(Ht.columns() == constraints.size());
  assert(values.rows() == constraints.size());
  assert(tmp.rows() == constraints.size());

  unsigned int nc = constraints.size();

  evaluateConstraints(values, frame, constraints);

  for(unsigned int i=0; i < 6; i++)
  {
    Twist t;
    t(i) = 1.0;
    Frame f = addDelta(frame, t, dd);
    f.p = (f.M * frame.M.Inverse()) * f.p; // change ref point to object

    evaluateConstraints(tmp, frame, constraints);

    for(unsigned int j=0; j < nc; j++)
      Ht(i,j) = (tmp(j) - values(j)) / dd;
  }
}

//TODO: get this executed automatically
void Constraint::init()
{
  feature_functions_["perpendicular"] = perpendicular;
  feature_functions_["height"] = height;
  feature_functions_["distance"] = distance;
  feature_functions_["pointing_at"] = pointing_at;
}
