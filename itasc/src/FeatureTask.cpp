#include "FeatureTask.hpp"
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>
#include <ocl/Component.hpp>
#include <rtt/ConnPolicy.hpp>

#include <ros/ros.h>

#include <kdl/utilities/svd_eigen_HH.hpp>


/*
 * Using this macro, only one component may live
 * in one library. If you have put your component
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(FeatureTask);

#define NC 6

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


FeatureTask::FeatureTask(const std::string& name) :
  SubTask(name,PreOperational),
  start_count(0),
  guard_time(200),
  Jf_total(NC),
  chi_f(NC),
  ydot(NC),
  feedback_gain(NC,1.0),weights(NC,1.0),
  desired_values(NC,1.0),
  Wy(Matrix<double,NC,NC>::Identity()),
  ros_prefix("left"),
  J_inv_t(6)
{
  properties()->addProperty("weights_property", weights).doc("local weights of the constraints");
  properties()->addProperty("gain", feedback_gain).doc("feedback gains for the constraints");
  properties()->addProperty("ros_prefix", ros_prefix).doc("prefix for the ROS topic names");
  properties()->addProperty("guard_time", ros_prefix).doc("number of cycles to wait before initialization");

  // to be taken from configuration objects

  axis_names.resize(6);
  axis_names[0] = "angle";
  axis_names[1] = "distance";
  axis_names[2] = "height";
  axis_names[3] = "roll";
  axis_names[4] = "pitch";
  axis_names[5] = "yaw";

  RTT_init();

  nc=NC;
}

// helper for connecting ports to both ROS and RTT
template <class P> void FeatureTask::ROS_add_port(const std::string &rtt_name,
						     const std::string &ros_name,
						     P &port)
{
  ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
  c.transport = 3;
  c.name_id = ros_name;

  ports()->addPort(rtt_name, port);
  port.createStream(c);
}


void FeatureTask::ROS_init()
{
  ROS_add_port("chif", ros_prefix+"/chi_f", ros_chi_f_port);
  ROS_add_port("chif_desired", ros_prefix+"/chi_f_desired", ros_chi_f_desired_port);
  ROS_add_port("chif_command", ros_prefix+"/chi_f_command", ros_chi_f_command_port);
  ROS_add_port("weight_command", ros_prefix+"/weight_command", ros_weight_command_port);
  ROS_add_port("constraint_state", "/constraint_state", ros_constraint_state_port);


  ROS_add_port("constraint_command", ros_prefix+"/constraint_command", ros_constraint_command_port);
  ROS_add_port("constraint_mode", ros_prefix+"/constraint_mode", ros_constraint_mode_port);
  ROS_add_port("constraint_select", ros_prefix+"/constraint_select", ros_constraint_select_port);

  ros_chi_f.data.resize(NC);
  ros_chi_f_desired.data.resize(NC);
  ros_weights.data.resize(NC);

  // used for display now
  ros_task_jacobian.columns.resize(NC);

  ros_constraint_command.pos_lo.resize(NC);
  ros_constraint_command.pos_hi.resize(NC);
  ros_constraint_command.weight.resize(NC);
  ros_mode.data   = 0;
  ros_select.data = 0;

  ros_constraint_state.joint_names.resize(nc);
  ros_constraint_state.chi.resize(nc);
  ros_constraint_state.chi_desired.resize(nc);
  ros_constraint_state.weights.resize(nc);

  for(unsigned int a=0; a < axis_names.size(); a++)
    ros_constraint_state.joint_names[a] = axis_names[a];

  // debugging
  ROS_add_port("o1o2_pose", ros_prefix+"/o1o2_pose", ros_o1o2_pose_port);
  ROS_add_port("task_twist", ros_prefix+"/task_twist", ros_task_twist_port);
  ROS_add_port("task_jacobian", ros_prefix+"/task_jacobian", ros_task_jacobian_port);
  ROS_add_port("weights", ros_prefix+"/weights", ros_weights_port);
  ROS_add_port("constraint_state", ros_prefix+"/constraint_state", ros_constraint_state_port);
}

void FeatureTask::RTT_init()
{
  for(unsigned int a=0; a < axis_names.size(); a++)
  {
    measured_ports.push_back(new RTT::OutputPort<double>());
    desired_ports.push_back(new RTT::InputPort<double>());
  }

  for(unsigned int a=0; a < axis_names.size(); a++)
  {
    std::string name_des = "desired_"+axis_names[a];
    std::string description_des = "Desired value of the "+axis_names[a];
    ports()->addPort(name_des, *(desired_ports[a])).doc(description_des);

    std::string name_msr = "measured_"+axis_names[a];
    std::string description_msr = "Measured value of the "+axis_names[a];
    ports()->addPort(name_msr, *(measured_ports[a])).doc(description_msr);
  }
}


void FeatureTask::ROS_publish()
{
  // copy data to ROS message
  for(unsigned int i=0; i < NC; i++)
  {
    ros_chi_f.data[i]=chi_f(i);
    ros_chi_f_desired.data[i]=desired_values[i];
    ros_weights.data[i]=weights[i];

    ros_constraint_state.chi[i] = chi_f(i);
    ros_constraint_state.chi_desired[i] = desired_values[i];
    ros_constraint_state.weights[i] = weights[i];
  }

  tf::PoseKDLToMsg(pose, ros_constraint_state.pose);

  ros_chi_f_port.write(ros_chi_f);
  ros_chi_f_desired_port.write(ros_chi_f_desired);
  ros_weights_port.write(ros_weights);
  ros_constraint_state_port.write(ros_constraint_state);


  // copy task jacobian to ROS message
  for(unsigned int i=0; i < NC; i++)
  {
    Twist t = Jf_total.getColumn(i);

    ros_task_jacobian.columns[i].linear.x = t.vel.x();
    ros_task_jacobian.columns[i].linear.y = t.vel.y();
    ros_task_jacobian.columns[i].linear.z = t.vel.z();

    ros_task_jacobian.columns[i].angular.x = t.rot.x();
    ros_task_jacobian.columns[i].angular.y = t.rot.y();
    ros_task_jacobian.columns[i].angular.z = t.rot.z();
  }
  ros_task_jacobian_port.write(ros_task_jacobian);
}


void FeatureTask::ROS_receive()
{
  if(NewData==ros_chi_f_command_port.read(ros_chi_f_command))
    for(unsigned int i=0; i < NC; i++)
      desired_values[i] = ros_chi_f_command.data[i];

  if(NewData==ros_weight_command_port.read(ros_weight_command))
    for(unsigned int i=0; i < NC; i++)
      weights[i] = ros_weight_command.data[i];

  ros_constraint_command_port.read(ros_constraint_command);
  ros_constraint_mode_port.read(ros_mode);
  ros_constraint_select_port.read(ros_select);
}

void FeatureTask::RTT_receive()
{
  double desired_value;
  for(unsigned int i=0; i < desired_ports.size(); i++)
    if(NewData==desired_ports[i]->read(desired_value))
      desired_values[i]=desired_value;
}

void FeatureTask::RTT_publish()
{
  // publish measured joint values
  for(unsigned int i=0; i < NC; i++)
    measured_ports[i]->write(chi_f(i));

  // interface with iTaSC
  Jf_port.write(J_inv_t);
  ydot_port.write(ydot);
}

FeatureTask::~FeatureTask()
{
  for(unsigned int i = 0; i < measured_ports.size(); ++i)
    delete measured_ports[i];
  for(unsigned int i = 0; i < desired_ports.size(); ++i)
    delete desired_ports[i];
}

bool FeatureTask::configureHook()
{
  Wy.diagonal() = Eigen::Matrix<double, 6, 1>::Map(&(weights)[0],
						   weights.size());
  Wy_port.write(Wy);

  ROS_init();

  return true;
}

bool FeatureTask::startHook(){

  Logger::In in(this->getName());
  //Check if task is well connected to the robot
  if (!Jq_qdot_port.connected() || !T_o1_o2_port.connected()) {
    log(Error) << "Read ports are not ready." << endlog();
    return false;
  }
  //Initialize ports
  SetToZero(ydot);
  ydot_port.write(ydot);

  T_o1_o2_port.read(pose);
  derive_features(pose);

  Cf_port.write(Matrix<double, 6, 6>::Identity());

  return true;
}


void FeatureTask::updateHook()
{
  Logger::In in(this->getName());

  T_o1_o2_port.read(pose);

  ROS_receive();
  RTT_receive();

  derive_features(pose);

  // weight coupling matrix
  Wy.diagonal() = Eigen::Matrix<double, 6, 1>::Map(&(weights)[0],
						   weights.size());
  Wy_port.write(Wy);

  if(ros_mode.data == 0)
    doControl();         // classical control with desired positions and weights
  else if(ros_mode.data == 1)
    doControl_ranges();  // control for desired ranges.
  else
    for(int i=0; i < 6; i++)
      ydot(i) = 0.0;     // unknown control method, stop the robot.


  // for the first guard_count cycles don't move!
  // really evil hack. should be moved to "outside"
  if(start_count < guard_time)
  {
    // set desired pose to current pose
    for(int i=0; i < 6; i++)
    {
      desired_values[i] = chi_f(i);

      double margin = (i < 3) ? 0.05 : 0.15;

      ros_constraint_command.pos_lo[i] = chi_f(i) - margin;
      ros_constraint_command.pos_hi[i] = chi_f(i) + margin;
      ros_constraint_command.weight[i] = 1.0;
    }
    // don't move!
    for(unsigned int i=0;i<NC;i++)
      ydot(i) = 0.0;

    start_count++;
  }


  RTT_publish();
  ROS_publish();
}


//! new range-controller.
//! this controller accepts ranges of accepted positions and
//! lowers the weight to zero when inside that range. 
void FeatureTask::doControl_ranges()
{
  double s = 0.05;
  for(int i=0; i < 6; i++)
  {
    if(ros_constraint_command.weight[i] == 0.0)
    {
      ydot(i) = 0.0;
      continue;
    }

    double value = chi_f(i);

    double lo = ros_constraint_command.pos_lo[i];
    double hi = ros_constraint_command.pos_hi[i];

    // adjust margin if range is too small
    double ss = (hi - lo < 2*s) ? (hi - lo) / 2 : s;

    if(value > hi - ss)
    {
      ydot(i) = feedback_gain[i]*(hi - ss - value);
      desired_values[i] = hi - ss;
    }
    else if(value < lo + ss)
    {
      ydot(i) = feedback_gain[i]*(lo + ss - value);
      desired_values[i] = lo + ss;
    }
    else
    {
      ydot(i) = 0.0;
      desired_values[i] = value;
    }


    if(value > hi || value < lo)
    {
      weights[i] = 1.0;
    }
    else
    {
      double w_lo = (1/s)*(-hi + value)+1;
      double w_hi = (1/s)*( lo - value)+1;

      w_lo = (w_lo > 0.0) ? w_lo : 0.0;
      w_hi = (w_hi > 0.0) ? w_hi : 0.0;

      weights[i] = (w_lo > w_hi) ? w_lo : w_hi;
    }
  }
}


//! old task angle position controller
void FeatureTask::doControl()
{
  for(unsigned int i=0;i<NC;i++)
    ydot(i)=feedback_gain[i]*(desired_values[i] - chi_f(i));
}



void compute_features_cyl(double *feature_values, KDL::Frame frame)
{
  Vector p = frame.p;

  double x0 = atan2(p.y(), p.x());
  double x1 = sqrt(p.x()*p.x() + p.y()*p.y());
  double x2 = p.z();

  // angle computation
  Vector vx = frame.M.UnitX();
  Vector vy = frame.M.UnitY();
  Vector vz = frame.M.UnitZ();

  double a0 = dot(-vy, Vector(0,0,1)); // front edge aliged <=> a0 == 0
  double a1 = dot(vz, Vector(0,0,1)); // side edge aligned <=> a1 == 0

  // tool direction:
  // * look from above -> project -vz onto x-y-plane , yields vzp
  // * take angle perpendicular to horizontal component of frame.p
  // (tool direction towards center <=> a3 == 0)

  Vector vzp = Vector(-vz.x(), -vz.y(), 0);
  Vector pxy = Vector(p.y(), -p.x(), 0);
  double a2 = dot(vzp, pxy) / (vzp.Norm() * pxy.Norm());

  feature_values[0] = x0;
  feature_values[1] = x1;
  feature_values[2] = x2;
  feature_values[3] = a0;
  feature_values[4] = a1;
  feature_values[5] = a2;
}

void compute_features_cart(double *feature_values, KDL::Frame frame)
{
  Vector p = frame.p;

  // angle computation
  Vector vx = frame.M.UnitX();
  Vector vy = frame.M.UnitY();
  Vector vz = frame.M.UnitZ();

  double a0 = dot(-vy, Vector(0,0,1)); // front edge aliged <=> a0 == 0
  double a1 = dot(vz, Vector(0,0,1)); // side edge aligned <=> a1 == 0

  // tool direction:
  // * look from above -> project -vz onto x-y-plane , yields vzp
  // * take angle perpendicular to horizontal component of frame.p
  // (tool direction towards center <=> a3 == 0)

  Vector vzp = Vector(-vz.x(), -vz.y(), 0);
  Vector pxy = Vector(p.y(), -p.x(), 0);
  double a2 = dot(vzp, pxy) / (vzp.Norm() * pxy.Norm());

  feature_values[0] = p.x();
  feature_values[1] = p.y();
  feature_values[2] = p.z();
  feature_values[3] = a0;
  feature_values[4] = a1;
  feature_values[5] = a2;
}


void FeatureTask::compute_features(double *feature_values, KDL::Frame frame)
{
  if(ros_select.data == 0)
    compute_features_cyl(feature_values, frame);
  else if(ros_select.data == 1)
    compute_features_cart(feature_values, frame);
}



// WARNING: this is NOT at all realtime-safe.
MatrixXd pinv(MatrixXd M, double eps=1e-15)
{
  int m=M.rows(), n=M.cols();
  //printf("jacobian has %d rows and %d columns.\n", m, n);

  // NOTE: this could be realtime safe if these matrices had max. sizes.
  MatrixXd U(m,n), V(n,n);
  MatrixXd M_inv(n,m);
  VectorXd S(n), Sp(n), tmp(n);

  svd_eigen_HH(M, U, S, V, tmp);

  for(int i=0; i <n; ++i)
      Sp(i) = (fabs(S(i)) > eps) ? 1.0 / S(i) : 0.0;

  M_inv = V * Sp.asDiagonal() * U.transpose();

  return M_inv;
}

//! Computes the current feature values and its derivative
//!
//! The results are stored in chi_f and J_inv_t, respectively.
//! This method uses compute_features().
void FeatureTask::derive_features(KDL::Frame frame, double dd)
{
  double feature_values[nc];
  double feature_values0[nc];

  compute_features(feature_values0, frame);

  for(unsigned int i=0; i < nc; i++)
    chi_f(i) = feature_values0[i];

  for(unsigned int i=0; i < 6; i++)
  {
    Twist t;
    t(i) = 1.0;
    Frame f = addDelta(frame, t, dd);
    f.p = (f.M * frame.M.Inverse()) * f.p; // change ref point to object

    compute_features(feature_values, f);

    for(unsigned int j=0; j < nc; j++)
      J_inv_t(i,j) = ( - feature_values0[j] + feature_values[j]) / dd;
  }

  // DEBUGGING

  // invert the inverted jacobian.
  Jf_total.data = pinv(J_inv_t.data.transpose());

  // send out the task twist
  tf::TwistKDLToMsg(Jf_total*ydot, ros_task_twist);
  ros_task_twist_port.write(ros_task_twist);

  geometry_msgs::PoseStamped pp;
  // stupid hack for rviz (running locally, wtf?!)
  pp.header.stamp = ros::Time::now() - ros::Duration(0.1);
  pp.header.frame_id = "/pancake";
  tf::PoseKDLToMsg(frame, pp.pose);
  ros_o1o2_pose_port.write(pp);
}

