#include <feature_constraints/FeatureConstraints.h>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <tf_conversions/tf_kdl.h>


// The constant EPS is used to set some
// nan results to zero. This is useful e.g.
// for inverting the interaction matrix
// but its use this deep in the code
// is questionable...
#define EPS 1e-10

using namespace std;
using namespace KDL;

std::map<std::string, ConstraintFunc> Constraint::constraint_functions_;
std::set<ConstraintFunc> Constraint::angular_constraints_;

// some feature functions

//! zero when perpendicular
double perpendicular(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  const Vector &d_o = object_feature.dir;
  const Vector &d_t = tool_feature.dir;

  return dot(d_o, Frame(frame.M) * d_t) / (d_o.Norm() * d_t.Norm());
}

//! distance between features, projected onto the
//! the direction of the object feature
double height(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  const Vector &p_o = object_feature.pos;
  Vector  p_t = frame * tool_feature.pos;
  const Vector &d_o = object_feature.dir;

  double d_o_norm = d_o.Norm();

  if(d_o_norm < EPS)
    return 0.0;
  else
    return dot(d_o, p_t - p_o) / d_o_norm;
}

double distance(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  Vector  p   = frame*tool_feature.pos - object_feature.pos;
  const Vector &d_o = object_feature.dir;

  double d_o_norm = d_o.Norm();

  if(d_o_norm < EPS)
    return 0.0;
  else
    {
    Vector d_on = d_o / d_o_norm;
    return (p - (dot(d_on, p))*d_on).Norm();
    }
}

double pointing_at(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  Vector  p   = frame*tool_feature.pos - object_feature.pos;
  const Vector &d_o = object_feature.dir;
  Vector  d_t = Frame(frame.M) * tool_feature.dir;

  // projection of p perpendicular to d_o ...
  // ... and taking the perpendicular vector in that plane:
  Vector p_h = d_o * p;  // cross product

  // projection of d_t perpendicular to d_o
  Vector d_on = d_o / d_o.Norm();
  Vector d_h = d_t  -  d_on * dot(d_on, d_t);

  double denom = p_h.Norm()*d_h.Norm();
 
  if(denom < EPS)
    return 0.0;
  else
    return dot(p_h, d_h) / denom;
}


double direction(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  Vector  p    = frame*tool_feature.pos - object_feature.pos;
  Vector  d_o2 = object_feature.contact_dir;
  const Vector &d_o  = object_feature.dir;

  double d_o_norm = d_o.Norm();
  double d_o2_norm = d_o2.Norm();

  Vector d_on = d_o / d_o_norm;
  Vector d_o2n = d_o2 / d_o2_norm;

  if(d_o_norm < EPS || d_o2_norm < EPS)
    return 0.0;
  else
    return dot(d_o2n, (p - dot(d_on, p)*d_on));
}


double angle(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  Vector  p    = frame*tool_feature.pos - object_feature.pos;
  Vector  d_o2 = object_feature.contact_dir;
  const Vector &d_o  = object_feature.dir;

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


double null(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return 0.0;
}


void evaluateConstraints(KDL::JntArray& values, const KDL::Frame& frame, const std::vector<Constraint>& constraints)
{
  assert(values.rows() >= constraints.size());

  for(unsigned int i=0; i < constraints.size(); i++)
    values.data(i) = constraints[i](frame);
}


//! Normalize to [0..y)
double inline fmod_pos(double x, double y)
{
  // NOTE: The double call to fmod is necessary to 
  //       cleanly wrap to [0 .. y) (and _not_ to (-y .. 0] when x is negative)
  return fmod(fmod(x, y) + y, y);
}

/** \brief Normalize angle difference to -180..180 deg.
 */
inline double normalized_angle_diff(double angle, double angle_ref)
{
  return fmod_pos(angle - angle_ref + M_PI, 2*M_PI) - M_PI;
}


/** \brief Normalize angle to -180..180 deg relative to angle_ref.
 */
inline double normalized_angle(double angle, double angle_ref)
{
  return normalized_angle_diff(angle, angle_ref) + angle_ref;
}


void differentiateConstraints(KDL::Jacobian& Ht,
                              KDL::JntArray& values,
                              const KDL::Frame& frame,
                              const std::vector<Constraint> &constraints,
                              double dd,
                              KDL::JntArray& tmp)
{
  assert(Ht.columns() >= constraints.size());
  assert(values.rows() >= constraints.size());
  assert(tmp.rows() >= constraints.size());
  assert(dd != 0);

  double dd_r = 1.0 / dd;
  unsigned int nc = constraints.size();

  evaluateConstraints(values, frame, constraints);

  Frame f[6];
  double cd = cos(dd);
  double sd = sin(dd);
  f[0] = Frame(Vector(dd,0,0));
  f[1] = Frame(Vector(0,dd,0));
  f[2] = Frame(Vector(0,0,dd));
  f[3] = Frame(Rotation(1,0,0,  0,cd,-sd,  0,sd,cd));
  f[4] = Frame(Rotation(cd,0,sd,  0,1,0,  -sd,0,cd));
  f[5] = Frame(Rotation(cd,-sd,0,  sd,cd,0,  0,0,1));

  for(unsigned int i=0; i < 6; i++)
  {
    evaluateConstraints(tmp, f[i]*frame, constraints);

    for(unsigned int j=0; j < nc; j++)
      if(Constraint::angular_constraints_.find(constraints[j].func) ==
         Constraint::angular_constraints_.end())
        Ht.data(i,j) = (tmp.data(j) - values.data(j)) * dd_r;
      else
        Ht.data(i,j) = normalized_angle_diff(tmp.data(j), values.data(j)) * dd_r;
  }
}


void differentiateConstraints_3point(KDL::Jacobian& Ht,
                                     KDL::Jacobian& H2t,
                                     KDL::JntArray& values,
                                     const KDL::Frame& frame,
                                     const std::vector<Constraint> &constraints,
                                     double dd,
                                     KDL::JntArray& tmp,
                                     KDL::JntArray& tmp2)
{
  assert(Ht.columns()  >= constraints.size());
  assert(H2t.columns() >= constraints.size());
  assert(values.rows() >= constraints.size());
  assert(tmp.rows()    >= constraints.size());
  assert(tmp2.rows()   >= constraints.size());
  assert(dd != 0);

  double dd_r = 1.0 / dd;
  unsigned int nc = constraints.size();

  Frame fp[6], fn[6];
  double cd = cos(dd);
  double sd = sin(dd);
  fp[0] = Frame(Vector(dd,0,0));
  fp[1] = Frame(Vector(0,dd,0));
  fp[2] = Frame(Vector(0,0,dd));
  fp[3] = Frame(Rotation(1,0,0,  0,cd,-sd,  0,sd,cd));
  fp[4] = Frame(Rotation(cd,0,sd,  0,1,0,  -sd,0,cd));
  fp[5] = Frame(Rotation(cd,-sd,0,  sd,cd,0,  0,0,1));
  sd = -sd;
  fn[0] = Frame(Vector(-dd,0,0));
  fn[1] = Frame(Vector(0,-dd,0));
  fn[2] = Frame(Vector(0,0,-dd));
  fn[3] = Frame(Rotation(1,0,0,  0,cd,-sd,  0,sd,cd));
  fn[4] = Frame(Rotation(cd,0,sd,  0,1,0,  -sd,0,cd));
  fn[5] = Frame(Rotation(cd,-sd,0,  sd,cd,0,  0,0,1));

  evaluateConstraints(values, frame, constraints);

  for(unsigned int i=0; i < 6; i++)
  {
    evaluateConstraints(tmp,  fp[i]*frame, constraints);
    evaluateConstraints(tmp2, fn[i]*frame, constraints);

    for(unsigned int j=0; j < nc; j++)
    {
      if(Constraint::angular_constraints_.find(constraints[j].func) ==
         Constraint::angular_constraints_.end())
      {
        Ht(i,j)  = (tmp(j) - tmp2(j)) * 0.5*dd;
        H2t(i,j) = (tmp(i) - 2*values(i) + tmp2(i)) * dd_r*dd_r;
      }
      else
      {
        Ht(i,j)  = normalized_angle_diff(tmp(j), tmp2(j)) * 0.5 * dd_r;
        H2t(i,j) = normalized_angle_diff(tmp(i), values(i)) + normalized_angle_diff(tmp2(i), values(i)) * dd_r*dd_r;
      }
    }
  }
}


//TODO: get this executed automatically

#include <feature_constraints/ChainRPY.h>

void Constraint::init()
{
  constraint_functions_["perpendicular"] = perpendicular;
  constraint_functions_["height"] = height;
  constraint_functions_["distance"] = distance;
  constraint_functions_["pointing_at"] = pointing_at;
  constraint_functions_["direction"] = direction;
  constraint_functions_["angle"] = angle;
  constraint_functions_["null"] = null;

  angular_constraints_.insert(angle);

  chain_rpy_init();
}

