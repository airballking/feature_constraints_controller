#include <feature_constraints/FeatureConstraints.h>

#include <gtest/gtest.h>

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <vector>

using namespace std;
using namespace KDL;


#define EPS_CONSTRAINTS 1e-14


void test_setup(vector<Constraint>& constraints)
{
  Constraint::init();

  Feature tool_front, tool_side;

  tool_front.dir = Vector(0,1,0);
  tool_side.dir = Vector(0,0,1);
  Feature up("up", Vector(0,0,0), Vector(0,0,1));

  Constraint c;

  c.func = Constraint::feature_functions_["height"];
  c.tool_features[0] = tool_front;
  c.object_features[0] = up;

  constraints.push_back(c);

  constraints.push_back(Constraint("dist",
    Constraint::feature_functions_["distance"], tool_front, up));
  constraints.push_back(Constraint("align_front", "perpendicular",
                                   tool_front, up));
  constraints.push_back(Constraint("align_side", "perpendicular",
                                   tool_side, up));

}



//TODO: throw exception when constraint function is not found
TEST(Constraints, derive)
{
  vector<Constraint> constraints;
  test_setup(constraints);

  int nc = constraints.size();

  JntArray values(nc), tmp(nc);
  Jacobian Ht(nc);

  Frame f(Vector(1,2.1, 0));

  deriveConstraints(Ht, values, f, constraints, 0.001, tmp);

  std::cout << "Ht:" << std::endl << Ht << endl;
  std::cout << "values:" << std::endl << values << endl;

}



void check_dist(Frame frame, double l_tool, double l_object)
{
  Feature tool_front("front", Vector(0,0,0), Vector(l_tool,0,0));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object));

  Constraint constr("distance", "distance", tool_front, object_up);

  double d     = constr(frame);
  double d_des = Vector(frame.p.x(), frame.p.y(), 0).Norm();

  EXPECT_NEAR(d, d_des, EPS_CONSTRAINTS);
}



void check_height(Frame frame, double l_tool, double l_object)
{
  Feature tool_front("front", Vector(0,0,0), Vector(l_tool,0,0));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object));

  Constraint constr("height", "height", tool_front, object_up);

  double h     = constr(frame);
  double h_des = frame.p.z();

  EXPECT_NEAR(h, h_des, EPS_CONSTRAINTS);
}


void check_perpendicular(Frame frame, double l_tool, double l_object)
{
  Feature tool_front("front", Vector(0,0,0), Vector(l_tool,0,0));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object));

  Constraint constr("perpendicular", "perpendicular", tool_front, object_up);

  double a     = constr(frame);
  double a_des = frame.M(2,0);

  EXPECT_NEAR(a, a_des, EPS_CONSTRAINTS);
}


void check_pointing_at(Frame frame, double l_tool, double l_object)
{
  Feature tool_side("side", Vector(0,0,0), Vector(0,0,l_tool));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object));

  Constraint constr("pointing_at", "pointing_at", tool_side, object_up);


  // NOTE: This code was taken from known-to-be-working iTaSC code.

  // tool direction:
  // * look from above -> project -vz onto x-y-plane , yields vzp
  // * take angle perpendicular to horizontal component of frame.p
  // (tool direction towards center <=> a3 == 0)

  Vector vz  = frame.M.UnitZ();
  Vector vzp = Vector(-vz.x(), -vz.y(), 0);
  Vector pxy = Vector(frame.p.y(), -frame.p.x(), 0);

////FROM DEBUGGING  std::cout << "TEST :: p_h = " << pxy/pxy.Norm() << std::endl;
////FROM DEBUGGING  std::cout << "TEST :: d_h = " << vzp/vzp.Norm() << std::endl;

  double a     = constr(frame);
  double a_des = 0;

  if(vzp.Norm() * pxy.Norm() != 0)
    a_des = dot(vzp, pxy) / (vzp.Norm() * pxy.Norm());

  EXPECT_NEAR(a, a_des, EPS_CONSTRAINTS);
}


void check_direction(Frame frame, double l_tool, double l_object)
{
  Feature tool_front("front", Vector(0,0,0), Vector(l_tool,0,0));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object), Vector(1.0,0,0)); // contact_dir points away from the robot

  Constraint constr("direction", "direction", tool_front, object_up);
  
  double a     = constr(frame);
  double a_des = frame.p.x();
 
  EXPECT_NEAR(a, a_des, EPS_CONSTRAINTS);
}


void check_angle(Frame frame, double l_tool, double l_object)
{
  Feature tool_front("front", Vector(0,0,0), Vector(l_tool,0,0));
  Feature object_up("up", Vector(0,0,0), Vector(0,0,l_object), Vector(1.0,0,0)); // contact_dir points away from the robot

  Constraint constr("angle", "angle", tool_front, object_up);

  double a     = constr(frame);
  double a_des = atan2(frame.p.y(), frame.p.x());

  EXPECT_NEAR(a, a_des, EPS_CONSTRAINTS);
}

TEST(Constraints, Constraint_Functions)
{
  double lengths[]={0.1, 1.0, 2.3};

  std::vector<Frame> frames;

  Rotation r1 = Rotation::RotX(0.7)*Rotation::RotY(0.4)*Rotation::RotZ(0.3);
  Rotation r2 = Rotation::RotX(2.2)*Rotation::RotZ(0.25);

  frames.push_back(Frame(Vector(1,0,0)));
  frames.push_back(Frame(Vector(0,1,0)));
  frames.push_back(Frame(Vector(0,0,1)));
  frames.push_back(Frame(Vector(0,0,0)));
  frames.push_back(Frame(r1, Vector(0,0,0)));
  frames.push_back(Frame(r1, Vector(0.30 ,0.71, 1.41)));
  frames.push_back(Frame(r2, Vector(0.22 ,1.21, 0.82)));

  for(int l_obj=0; l_obj < 3; l_obj++)
  {
    for(int l_tool=0; l_tool < 3; l_tool++)
    {
      for(unsigned int f=0; f < frames.size(); f++)
      {
        SCOPED_TRACE(testing::Message() << "\nf=\n" << frames[f] << "\nl_obj = " << lengths[l_obj] << "\nl_tool = " << lengths[l_tool]);
        check_dist          ( frames[f], lengths[l_tool], lengths[l_obj] );
        check_height        ( frames[f], lengths[l_tool], lengths[l_obj] );
        check_perpendicular ( frames[f], lengths[l_tool], lengths[l_obj] );
        check_pointing_at   ( frames[f], lengths[l_tool], lengths[l_obj] );
        //check_direction     ( frames[f], lengths[l_tool], lengths[l_obj] );
        check_angle         ( frames[f], lengths[l_tool], lengths[l_obj] );
      }
    }
  }
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

