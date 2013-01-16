#include <feature_constraints/Analysis.h>
#include <feature_constraints/ChainRPY.h>
#include "stdio.h"

using namespace KDL;
using namespace std;



std::vector<Constraint> feature_pancake_constraints()
{
  Constraint::init();

  // tool features
  Feature tool_front("front_edge", Vector(0,0,0), Vector(0,1,0));
  Feature tool_front_rev("front_edge_rev", Vector(0,0,0), Vector(0,-1,0));
  Feature tool_side("side_edge", Vector(0,0,0), Vector(0,0,1));

  // object feature
  Feature up("up", Vector(0,0,0), Vector(0,0,1), Vector(1,0,0)); // contact_dir points away from robot

  vector<Constraint> constraints;

  constraints.push_back(
    Constraint("angle", "angle", tool_front, up));
  constraints.push_back(
    Constraint("dist", "distance", tool_front, up));
  constraints.push_back(
    Constraint("height", "height", tool_front, up));

  constraints.push_back(
    Constraint("align_front", "perpendicular", tool_front_rev, up));
  constraints.push_back(
    Constraint("align_side",  "perpendicular", tool_side,  up));
  constraints.push_back(
    Constraint("pointing_at", "pointing_at", tool_side, up));

  return constraints;
}

std::vector<Constraint> legacy_pancake_constraints()
{
  Constraint::init();

  Constraint::angular_constraints_.insert(chain0);
  Constraint::angular_constraints_.insert(chain3);
  Constraint::angular_constraints_.insert(chain4);
  Constraint::angular_constraints_.insert(chain5);

  Feature f;  // dummy feature, not used for the legay constraints
  vector<Constraint> constraints;
  constraints.push_back(Constraint("angle",  chain0, f, f));
  constraints.push_back(Constraint("dist",   chain1, f, f));
  constraints.push_back(Constraint("height", chain2, f, f));
  constraints.push_back(Constraint("roll",   chain3, f, f));
  constraints.push_back(Constraint("pitch",  chain4, f, f));
  constraints.push_back(Constraint("yaw",    chain5, f, f));
  return constraints;
}


void print_discontinuity(bool legacy, int index)
{
  Constraint c;

  if(legacy)
    c = legacy_pancake_constraints()[index];
  else
    c = feature_pancake_constraints()[index];

  Frame frame(Vector(1,0,0));
  vector<pair<Eigen::Quaterniond, double> > dis = continuityPlotRPY(c, frame, 256, 0.01, 0.3);

  for(unsigned int i=0; i < dis.size(); i++)
  {
    Eigen::Quaterniond q = dis[i].first;
    printf("%f %f %f %f   %f\n", q.x(), q.y(), q.z(), q.w(), dis[i].second);
  }
}


void discontinuity_at(bool legacy, KDL::Frame frame)
{
  vector<Constraint> constraints;

  if(legacy)
    constraints = legacy_pancake_constraints();
  else
    constraints = feature_pancake_constraints();

  double dd = 0.001;

  vector<double> result(constraints.size());


  double qx,qy,qz,qw;
  frame.M.GetQuaternion(qx, qy, qz, qw);
  printf("%c %f %f %f   %f %f %f %f     ", (legacy) ? 'l' : 'f',
    frame.p.x(), frame.p.y(), frame.p.z(), qx, qy, qz, qw);

  for(unsigned int i=0; i < constraints.size(); i++)
      result[i] = discontinuity_near(constraints[i], frame, dd, 0.01, 0.001);

  for(unsigned int i=0; i < result.size(); i++)
    printf("%f ", result[i]);
  printf("\n");
}

int main(int argc, char* argv[])
{
  print_discontinuity(false, 5);
  //discontinuity_at(false, Frame(Rotation::RotY(0.4), Vector(0,0,1)));
}

