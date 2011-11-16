#include <feature_constraints/FeatureConstraints.h>

using namespace std;
using namespace KDL;

vector<Constraint> test_setup()
{
   Constraint::init();

   Feature tool_front, tool_side;

   tool_front.dir = Vector(0,1,0);
   tool_side.dir = Vector(0,0,1);
   Feature up("up", Vector(0,0,0), Vector(0,0,1));

   vector<Constraint> constraints;

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

   return constraints;
}


//TODO: throw exception when constraint function is not found
void test_derive()
{
  vector<Constraint> constraints = test_setup();

  int nc = constraints.size();

  JntArray values(nc), tmp(nc);
  Jacobian Ht(nc);

  Frame f(Vector(1,2,0));

  deriveConstraints(Ht, values, f, constraints, 0.001, tmp);
}


int main()
{
  test_setup();
  test_derive();

  return 0;
}
