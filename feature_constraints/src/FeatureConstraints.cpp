#include <feature_constraints/FeatureConstraints.h>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <tf_conversions/tf_kdl.h>


////FROM DEBUGGING
#include <kdl/frames_io.hpp>
////FROM DEBUGGING
#include <kdl/kinfam_io.hpp>



// The constant EPS is used to set some
// nan results to zero. This is useful e.g.
// for inverting the interaction matrix
// but its use this deep in the code
// is questionable...
#define EPS 1e-10

using namespace std;
using namespace KDL;

std::map<std::string, FeatureFunc> Constraint::feature_functions_;

// some feature functions

//! zero when perpendicular
double perpendicular(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector &d_o = object_feature.dir;
  Vector &t_o = tool_feature.dir;

  return dot(d_o, Frame(frame.M) * t_o) / (d_o.Norm() * t_o.Norm());
}

//! distance between features, projected onto the
//! the direction of the object feature
double height(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector &p_o = object_feature.pos;
  Vector  p_t = frame * tool_feature.pos;
  Vector &d_o = object_feature.dir;

  double d_o_norm = d_o.Norm();

  if(d_o_norm < EPS)
    return 0.0;
  else
    return dot(d_o, p_t - p_o) / d_o_norm;
}

double distance(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector  p   = frame*tool_feature.pos - object_feature.pos;
  Vector &d_o = object_feature.dir;

  double d_o_norm = d_o.Norm();

  if(d_o_norm < EPS)
    return 0.0;
  else
    {
    Vector d_on = d_o / d_o_norm;
////FROM DEBUGGING    cout << "p_align=" << (dot(d_on, p))*d_on << std::endl;
////FROM DEBUGGING    cout << "p_perp= " << (p - (dot(d_on, p))*d_on) << std::endl;
    return (p - (dot(d_on, p))*d_on).Norm();
    }
}

double pointing_at(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector  p   = frame*tool_feature.pos - object_feature.pos;
  Vector &d_o = object_feature.dir;
  Vector  d_t = Frame(frame.M) * tool_feature.dir;

  // projection of p perpendicular to d_o ...
  // ... and taking the perpendicular vector in that plane:
  Vector p_h = d_o * p;  // cross product

  // projection of d_t perpendicular to d_o
  Vector d_on = d_o / d_o.Norm();
  Vector d_h = d_t  -  d_on * dot(d_on, d_t);

////FROM DEBUGGING  std::cout << "ALGO :: p_h = " << p_h/p_h.Norm() << std::endl;
////FROM DEBUGGING  std::cout << "ALGO :: d_h = " << d_h/d_h.Norm() << std::endl;

  double denom = p_h.Norm()*d_h.Norm();
  
  if(denom < EPS)
    return 0.0;
  else
    return dot(p_h, d_h) / denom;
}


double direction(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector  p    = frame*tool_feature.pos - object_feature.pos;
  Vector  d_o2 = object_feature.contact_dir;
  Vector &d_o  = object_feature.dir;

  double d_o_norm = d_o.Norm();
  double d_o2_norm = d_o2.Norm();

  if(d_o_norm < EPS)
    return 0.0;
  else
    return dot(d_o2 / d_o2_norm, (p - (dot(d_o, p) / d_o_norm)*p));
}


double angle(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  Vector  p    = frame*tool_feature.pos - object_feature.pos;
  Vector  d_o2 = object_feature.contact_dir;
  Vector &d_o  = object_feature.dir;

  double d_o_norm = d_o.Norm();
  double d_o2_norm = d_o2.Norm();

  // project p perpendicular to d_o
  Vector d_on = d_o / d_o_norm;
  Vector p_p  = p  -  d_on * dot(d_on, p);

  Vector dir = d_o2 / d_o2_norm;

  Vector x = dir * dot(dir, p_p);
  Vector y = p_p - x;



  if(d_o_norm < EPS)
    return 0.0;
  else
    return atan2(y.Norm(), x.Norm());
}


double null(KDL::Frame& frame, Feature tool_feature, Feature object_feature)
{
  return 0.0;
}

void evaluateConstraints(KDL::JntArray& values, KDL::Frame& frame, std::vector<Constraint>& constraints)
{
  assert(values.rows() >= constraints.size());

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
  assert(Ht.columns() >= constraints.size());
  assert(values.rows() >= constraints.size());
  assert(tmp.rows() >= constraints.size());
  assert(dd != 0);

  unsigned int nc = constraints.size();

  evaluateConstraints(values, frame, constraints);

  for(unsigned int i=0; i < 6; i++)
  {
    Twist t;
    t(i) = 1.0;
    Frame f = addDelta(frame, t, dd);
    f.p = (f.M * frame.M.Inverse()) * f.p; // change ref point to object

    evaluateConstraints(tmp, f, constraints);

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
  feature_functions_["direction"] = direction;
  feature_functions_["angle"] = angle;
  feature_functions_["null"] = null;
}
