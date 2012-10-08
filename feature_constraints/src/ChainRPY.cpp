
#include <kdl/frames.hpp>
#include <kdl/kinfam.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <feature_constraints/ChainRPY.h>


#include <stdio.h>

using namespace KDL;

KDL::JntArray cyl_irpy_chain(const KDL::Frame& frame)
{
  static KDL::ChainFkSolverPos_recursive* fksolver_baker=0;

  if(fksolver_baker == 0)
  {
    KDL::Chain chain_baker;

    chain_baker.addSegment(Segment(Joint(Joint::RotZ)));
    chain_baker.addSegment(Segment(Joint(Joint::TransX)));
    chain_baker.addSegment(Segment(Joint(Joint::TransZ),
                    Frame(Rotation::RotX(M_PI/2)*Rotation::RotY(-M_PI/2))));

    fksolver_baker = new ChainFkSolverPos_recursive(chain_baker);
  }

  JntArray chi_f(6);

  Vector p = frame.p;

  //printf("atan2: %f\n", atan2(p.y(),p.x()));

  chi_f(0)=atan2(p.y(),p.x());
  chi_f(1)=sqrt(p.x()*p.x()+p.y()*p.y());
  chi_f(2)=p.z();

  //printf("chi_f(0): %f\n", chi_f(0));

  Frame baker_pose;
  JntArray chi_f_baker(3);
  chi_f_baker(0) = chi_f(0);
  chi_f_baker(1) = chi_f(1);
  chi_f_baker(2) = chi_f(2);
  fksolver_baker->JntToCart(chi_f_baker, baker_pose);

  // rotations
  Rotation rot = (baker_pose.Inverse()*frame).M.Inverse();
  rot.GetRPY(chi_f(3),chi_f(4),chi_f(5));

  return chi_f;
}


double chain0(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(0);
}


double chain1(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(1);
}


double chain2(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(2);
}

double chain3(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(3);
}

double chain4(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(4);
}

double chain5(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature)
{
  return cyl_irpy_chain(frame)(5);
}

void chain_rpy_init()
{
  Constraint::angular_constraints_.insert(chain0);
  Constraint::angular_constraints_.insert(chain3);
  Constraint::angular_constraints_.insert(chain4);
  Constraint::angular_constraints_.insert(chain5);

  Constraint::constraint_functions_["chain0"] = chain0;
  Constraint::constraint_functions_["chain1"] = chain1;
  Constraint::constraint_functions_["chain2"] = chain2;
  Constraint::constraint_functions_["chain3"] = chain3;
  Constraint::constraint_functions_["chain4"] = chain4;
  Constraint::constraint_functions_["chain5"] = chain5;
}
