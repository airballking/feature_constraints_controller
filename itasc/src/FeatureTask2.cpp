#include "FeatureTask2.hpp"
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <ocl/Component.hpp>
#include <rtt/ConnPolicy.hpp>

#include <ros/ros.h>

#include <kdl/utilities/svd_eigen_HH.hpp>

#include <iostream>


// TODO: Make it work for more constraints!


/*
 * Using this macro, only one component may live
 * in one library. If you have put your component
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(FeatureTask2);

#define NC_DEFAULT 6

using namespace RTT;
using namespace KDL;
using namespace Eigen;


// some Jacobian operator

inline Twist operator*(const Jacobian& jac, const JntArray& qdot)
{
  assert(jac.columns() == qdot.rows());
  Twist t;

  int naxes = qdot.rows();
  for(int i=0; i < 6; ++i)
    for(int j=0; j < naxes; ++j)
      t(i) += jac(i,j)*qdot(j);

  return t;
}


FeatureTask2::FeatureTask2(const std::string& name) :
  SubTask(name,PreOperational),
  start_count(0),
  guard_time(200),
  Wy(Matrix<double,NC_DEFAULT,NC_DEFAULT>::Identity()),
  ros_prefix("left")
{
  properties()->addProperty("weights_property", controller.weights).doc("local weights of the constraints");
  properties()->addProperty("gains", controller.gains).doc("feedback gains for the constraints");
  properties()->addProperty("ros_prefix", ros_prefix).doc("prefix for the ROS topic names");
  properties()->addProperty("guard_time", guard_time).doc("number of cycles to wait before initialization");

  controller.prepare();

  nc=NC_DEFAULT;
}

// helper for connecting ports to both ROS and RTT
template <class P> void FeatureTask2::ROS_add_port(const std::string &rtt_name,
						     const std::string &ros_name,
						     P &port)
{
  ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
  c.transport = 3;
  c.name_id = ros_name;

  ports()->addPort(rtt_name, port);
  port.createStream(c);
}


FeatureTask2::~FeatureTask2()
{
}


bool FeatureTask2::configureHook()
{
  Constraint::init();

  // tool features
  Feature tool_front("front_edge", Vector(0,0,0), Vector(0,1,0));
  Feature tool_side("side_edge", Vector(0,0,0), Vector(0,0,1));

  // object feature
  Feature up("up", Vector(0,0,0), Vector(0,0,1));
  Feature away("away", Vector(0,0,0), Vector(1,0,0));

  controller.constraints.push_back(
    Constraint("angle", "angle", tool_front, up));
  controller.constraints[0].object_features[1] = away;

  controller.constraints.push_back(
    Constraint("dist", "distance", tool_front, up));
  controller.constraints.push_back(
    Constraint("height", "height", tool_front, up));

  controller.constraints.push_back(
    Constraint("align_front", "perpendicular", tool_front, up));
  controller.constraints.push_back(
    Constraint("align_side",  "perpendicular", tool_side,  up));
  controller.constraints.push_back(
    Constraint("pointing_at", "pointing_at", tool_side, up));


  int nGains = controller.gains.rows();  // save number of gains before resize...

  controller.prepare(nc);

  for(unsigned int i=nGains; i < nc; i++)  // ... and fill the rest with ones.
    controller.gains(i) = 1.0;


  Wy.resize(nc, nc);
  std::cout << "Wy" << Wy << std::endl;
  std::cout << "weights" << controller.weights.data << std::endl;
  std::cout << "r" << controller.weights.data.rows() << std::endl;
  std::cout << "c" << controller.weights.data.cols() << std::endl;
  Wy.diagonal() = controller.weights.data;
  Wy_port.write(Wy);

  ROS_add_port("config", ros_prefix+"/config", config_port);
  ROS_add_port("command", ros_prefix+"/command", command_port);
  ROS_add_port("state", ros_prefix+"/state", state_port);

  command.pos_lo.resize(nc);
  command.pos_hi.resize(nc);
  command.weight.resize(nc);

  state.interaction_matrix.resize(nc);
  state.jacobian.resize(nc);
  state.chi.resize(nc);
  state.chi_desired.resize(nc);
  state.weights.resize(nc);
  state.joint_names.resize(nc);
  state.singular_values.resize(nc);

  return true;
}

bool FeatureTask2::startHook(){

  Logger::In in(this->getName());
  //Check if task is well connected to the robot
  if (!Jq_qdot_port.connected() || !T_o1_o2_port.connected()) {
    log(Error) << "Read ports are not ready." << endlog();
    return false;
  }

  //Initialize ports

  ydot_port.write(controller.ydot);

  T_o1_o2_port.read(pose);

  controller.update(pose);

  Cf_port.write(Matrix<double, 6, 6>::Identity());

  return true;
}


void FeatureTask2::updateHook()
{
  Logger::In in(this->getName());

  T_o1_o2_port.read(pose);

  if(NewData == command_port.read(command))
    fromMsg(command, controller.command);

  if(NewData == config_port.read(config))
    fromMsg(config, controller.constraints);

  controller.update(pose);

  // for the first guard_count cycles don't move!
  // really bad hack. should be moved to "outside"
  if(start_count < guard_time)
  {
    // set desired pose to current pose
    for(unsigned int i=0; i < nc; i++)
    {
      double chi = controller.chi(i);

      // TODO: change these constants into attributes
      double margin = (i < 3) ? 0.05 : 0.15;

      controller.command.pos_lo(i) = chi - margin;
      controller.command.pos_hi(i) = chi + margin;
      controller.command.weight(i) = 1.0;
    }
    // don't move!
    for(unsigned int i=0; i < nc; i++)
      controller.ydot(i) = 0.0;

    start_count++;
  }

  // interface with iTaSC

  // weight coupling matrix
  Wy.diagonal() = controller.weights.data;
  Wy_port.write(Wy);

  Jf_port.write(controller.Ht);
  ydot_port.write(controller.ydot);

  // status output for visualization and others
  toMsg(controller, state);
  state_port.write(state);
}
