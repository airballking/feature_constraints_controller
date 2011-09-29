#include "ChainTask.hpp"
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
ORO_CREATE_COMPONENT(ChainTask);

#define N_CHIF_BAKER 3
#define N_CHIF_SPATULA 3

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


ChainTask::ChainTask(const std::string& name) :
  SubTask(name,PreOperational),
  pose_valid(false),
  start_count(0),
  Jf_total(NC),
  Jf_baker(N_CHIF_BAKER),
  Jf_spatula(N_CHIF_SPATULA),
  chi_f_baker(N_CHIF_BAKER),
  chi_f_spatula(N_CHIF_SPATULA),
  chi_f_spatula_init(N_CHIF_SPATULA),
  chi_f(NC),
  ydot(NC),
  feedback_gain(NC,1.0),weights(NC,1.0),
  desired_values(NC,1.0),
  Wy(Matrix<double,NC,NC>::Identity()),
  ros_prefix("left"),
  // allocate helper matrices for angle representation
  U(3,3), V(3,3), jac(3,3), jacinv(3,3),
  S(3), Sp(3), tmp(3),
  new_rotation(false)
{
  chain_baker.addSegment(Segment(Joint(Joint::RotZ)));
  chain_baker.addSegment(Segment(Joint(Joint::TransX)));
  chain_baker.addSegment(Segment(Joint(Joint::TransZ),
				 Frame(Rotation::RotX(M_PI/2)*Rotation::RotY(-M_PI/2))));

  chain_spatula.addSegment(Segment(Joint(Joint::RotX)));
  chain_spatula.addSegment(Segment(Joint(Joint::RotY)));
  chain_spatula.addSegment(Segment(Joint(Joint::RotZ)));

  fksolver_baker = new ChainFkSolverPos_recursive(chain_baker);
  fksolver_spatula = new ChainFkSolverPos_recursive(chain_spatula);
  jacsolver_baker = new ChainJntToJacSolver(chain_baker);
  jacsolver_spatula = new ChainJntToJacSolver(chain_spatula);

  properties()->addProperty("weights_property", weights).doc("local weights of the constraints");
  properties()->addProperty("gain", feedback_gain).doc("feedback gains for the constraints");
  properties()->addProperty("ros_prefix", ros_prefix).doc("prefix for the ROS topic names");
  properties()->addProperty("new_rotation", new_rotation).doc("use new angle representation?");

  // to be taken from configuration objects
  axis_names.resize(2);
  axis_names[0].resize(3);
  axis_names[0][0] = "angle";
  axis_names[0][1] = "distance";
  axis_names[0][2] = "height";
  axis_names[1].resize(3);
  axis_names[1][0] = "roll";
  axis_names[1][1] = "pitch";
  axis_names[1][2] = "yaw";

  object_names.resize(2);
  object_names[0] = "object";
  object_names[1] = "tool";

  RTT_init();

  nc=NC;


}


template <class P> void ChainTask::ROS_add_port(const std::string &rtt_name,
						     const std::string &ros_name,
						     P &port)
{
  ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
  c.transport = 3;
  c.name_id = ros_name;

  ports()->addPort(rtt_name, port);
  port.createStream(c);
}


void ChainTask::ROS_init()
{
  ROS_add_port("chif", ros_prefix+"/chi_f", ros_chi_f_port);
  ROS_add_port("chif_desired", ros_prefix+"/chi_f_desired", ros_chi_f_desired_port);
  ROS_add_port("chif_command", ros_prefix+"/chi_f_command", ros_chi_f_command_port);
  ROS_add_port("weight_command", ros_prefix+"/weight_command", ros_weight_command_port);
  //ROS_add_port("chain_state", "/chain_state", ros_chainstate_port);


  ROS_add_port("constraint_command", ros_prefix+"/constraint_command", ros_constraint_command_port);
  ROS_add_port("constraint_mode", ros_prefix+"/constraint_mode", ros_constraint_mode_port);

  ros_chi_f.data.resize(NC);
  ros_chi_f_desired.data.resize(NC);
  ros_weights.data.resize(NC);

  ros_constraint_command.pos_lo.resize(NC);
  ros_constraint_command.pos_hi.resize(NC);
  ros_constraint_command.weight.resize(NC);
  ros_mode.data = 0;

  //ros_chain_state.joint_names.resize(NC);
  //ros_chain_state.position_measured.resize(NC);
  //ros_chain_state.position_desired.resize(NC);

  //for(unsigned int i=0, o=0; o < object_names.size(); o++)
  //  for(unsigned int a=0; a < axis_names[o].size(); a++, i++)
  //    ros_chain_state.joint_names[i] = axis_names[o][a];

  // debugging
  ROS_add_port("chain_pose", ros_prefix+"/chain_pose", ros_chain_pose_port);
  ROS_add_port("o1o2_pose", ros_prefix+"/o1o2_pose", ros_o1o2_pose_port);
  ROS_add_port("desired_pose", ros_prefix+"/desired_pose", ros_desired_pose_port);
  ROS_add_port("task_twist", ros_prefix+"/task_twist", ros_task_twist_port);
  ROS_add_port("weights", ros_prefix+"/weights", ros_weights_port);
}

void ChainTask::RTT_init()
{
  for(unsigned int i=0; i < 6; i++)
  {
    measured_ports.push_back(new RTT::OutputPort<double>());
    desired_ports.push_back(new RTT::InputPort<double>());
  }

  for(unsigned int i=0, o=0; o < object_names.size(); o++)
  {
    for(unsigned int a=0; a < axis_names[o].size(); a++, i++)
    {
      std::string name_des = "desired_"+object_names[o]+"_"+axis_names[o][a];
      std::string description_des = "Desired value of the "+object_names[o]+" "+axis_names[o][a];
      ports()->addPort(name_des, *(desired_ports[i])).doc(description_des);

      std::string name_msr = "measured_"+object_names[o]+"_"+axis_names[o][a];
      std::string description_msr = "Measured value of the "+object_names[o]+" "+axis_names[o][a];
      ports()->addPort(name_msr, *(measured_ports[i])).doc(description_msr);
    }
  }
}


void ChainTask::ROS_publish()
{
  // copy data to ROS message
  for(unsigned int i=0; i < NC; i++)
  {
    ros_chi_f.data[i]=chi_f(i);
    ros_chi_f_desired.data[i]=desired_values[i];
    ros_weights.data[i]=weights[i];

    //ros_chain_state.position_measured[i] = chi_f(i);
    //ros_chain_state.position_desired[i] = desired_values[i];
  }

  ros_chi_f_port.write(ros_chi_f);
  ros_chi_f_desired_port.write(ros_chi_f_desired);
  ros_weights_port.write(ros_weights);
  //ros_chainstate_port.write(ros_chain_state);
}


void ChainTask::ROS_receive()
{
  if(NewData==ros_chi_f_command_port.read(ros_chi_f_command))
    for(unsigned int i=0; i < NC; i++)
      desired_values[i] = ros_chi_f_command.data[i];

  if(NewData==ros_weight_command_port.read(ros_weight_command))
    for(unsigned int i=0; i < NC; i++)
      weights[i] = ros_weight_command.data[i];

  ros_constraint_command_port.read(ros_constraint_command);
  ros_constraint_mode_port.read(ros_mode);
}

void ChainTask::RTT_receive()
{
  double desired_value;
  for(unsigned int i=0; i < desired_ports.size(); i++)
    if(NewData==desired_ports[i]->read(desired_value))
      desired_values[i]=desired_value;
}

void ChainTask::RTT_publish()
{
  // publish measured joint values
  for(unsigned int i=0; i < NC; i++)
    measured_ports[i]->write(chi_f(i));

  // interface with iTaSC
  Jf_port.write(Jf_total);
  ydot_port.write(ydot);
}

ChainTask::~ChainTask(){
  delete fksolver_baker;
  delete fksolver_spatula;
  delete jacsolver_baker;
  delete jacsolver_spatula;
  for(unsigned int i = 0; i < measured_ports.size(); ++i)
    delete measured_ports[i];
  for(unsigned int i = 0; i < desired_ports.size(); ++i)
    delete desired_ports[i];
}

bool ChainTask::configureHook()
{
  Wy.diagonal() = Eigen::Matrix<double, 6, 1>::Map(&(weights)[0],
						   weights.size());
  Wy_port.write(Wy);

  ROS_init();

  return true;
}

bool ChainTask::startHook(){

  Logger::In in(this->getName());
  //Check if task is well connected to the robot
  if (!Jq_qdot_port.connected() || !T_o1_o2_port.connected()) {
    log(Error) << "Read ports are not ready." << endlog();
    return false;
  }
  //Initialize ports
  SetToZero(ydot);
  ydot_port.write(ydot);

  SetToZero(chi_f_spatula_init);

  T_o1_o2_port.read(pose);
  model_update();

  Cf_port.write(Matrix<double, 6, 6>::Identity());

  desired_values[0]=chi_f_baker(0);
  desired_values[1]=chi_f_baker(1);
  desired_values[2]=chi_f_baker(2);
  desired_values[3]=chi_f_spatula(0);
  desired_values[4]=chi_f_spatula(1);
  desired_values[5]=chi_f_spatula(2);

  initialized.write(true);

  return true;

}


void ChainTask::updateHook(){
  Logger::In in(this->getName());

  T_o1_o2_port.read(pose);

  model_update();

  // weight coupling matrix
  Wy.diagonal() = Eigen::Matrix<double, 6, 1>::Map(&(weights)[0],
						   weights.size());
  Wy_port.write(Wy);

  ROS_receive();
  RTT_receive();

  if(ros_mode.data == 0)
    doControl();
  else if(ros_mode.data == 1)
    doControl_ranges();
  else
    for(int i=0; i < 6; i++)
      ydot(i) = 0.0;


  // really evil hack. should be moved to "outside"
  if(start_count > 200 && !pose_valid)
  {
    for(int i=0; i < 6; i++)
    {
      desired_values[i] = chi_f(i);

      double margin = (i < 3) ? 0.05 : 0.15;

      ros_constraint_command.pos_lo[i] = chi_f(i) - margin;
      ros_constraint_command.pos_hi[i] = chi_f(i) + margin;
      ros_constraint_command.weight[i] = 1.0;
    }
    pose_valid = true;
  }
  else if(!pose_valid)
  {
    // when pose may be unvalid, don't move!
    start_count++;
    for(unsigned int i=0;i<NC;i++)
      ydot(i) = 0.0;
  }


  tf::TwistKDLToMsg(Jf_total*ydot, ros_task_twist);
  ros_task_twist_port.write(ros_task_twist);

  RTT_publish();
  ROS_publish();
}


//! new range-controller.
//! this controller accepts ranges of accepted positions and
//! lowers the weight to zero when inside that range. 
void ChainTask::doControl_ranges()
{
  /// HACK FOR THE RPY ANGLES, PART 1
  double middle[3], margin[3];
  for(int i=0; i < 3; i++)
  {
    double lo = ros_constraint_command.pos_lo[i+3];
    double hi = ros_constraint_command.pos_hi[i+3];
    middle[i] = (hi + lo)/2.0;
    margin[i] = (hi - lo)/2.0;
  }

  Rotation desired = Rotation::RPY(middle[0], middle[1], middle[2]);
  Rotation measured = Rotation::RPY(chi_f_spatula(0), chi_f_spatula(1), chi_f_spatula(2));
  Vector rot = diff(desired, measured);
  rot = measured.Inverse()*rot;
  /// ///

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

    if(i >= 3 && !new_rotation) /// HACK FOR THE RPY ANGLES, PART 2
    {
      lo = value + rot(i-3) - margin[i-3];
      hi = value + rot(i-3) + margin[i-3];
    } /// ///

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
void ChainTask::doControl()
{
  Rotation desired = Rotation::RPY(desired_values[3],desired_values[4],desired_values[5]);
  Rotation measured = Rotation::RPY(chi_f_spatula(0),chi_f_spatula(1),chi_f_spatula(2));
  Vector rot = diff(desired,measured);
  rot = measured.Inverse()*rot;

  for(unsigned int i=0;i<NC;i++) {
    if(i<N_CHIF_BAKER || new_rotation)
      ydot(i)=feedback_gain[i]*(desired_values[i] - chi_f(i));
    else
      ydot(i)=feedback_gain[i]*rot(i-N_CHIF_BAKER);
  }
}

void ChainTask::model_update(){
  Vector p = pose.p;
  chi_f_baker(0)=atan2(p.y(),p.x());
  chi_f_baker(1)=sqrt(p.x()*p.x()+p.y()*p.y());
  chi_f_baker(2)=p.z();


  Frame baker_pose;
  fksolver_baker->JntToCart(chi_f_baker,baker_pose);

  Rotation rot = (baker_pose.Inverse()*pose).M.Inverse();

  if(new_rotation)
    derive_angles(Frame(rot));
  else
    RPY_angles(rot);

  geometry_msgs::PoseStamped pp;

  // stupid hack for rviz (running locally, wtf?!)
  pp.header.stamp = ros::Time::now() - ros::Duration(0.1);
  pp.header.frame_id = "/pancake";
  tf::PoseKDLToMsg(baker_pose, pp.pose);
  ros_chain_pose_port.write(pp);

  tf::PoseKDLToMsg(pose, pp.pose);
  ros_o1o2_pose_port.write(pp);

  /*
  Frame spatula_pose;
  fksolver_spatula->JntToCart(chi_f_spatula,spatula_pose);

  if(baker_pose*spatula_pose.Inverse()!=pose)
    log(Warning)<<"model update failed for some reason :("<<endlog();
  */

  //reference frame is baker, reference point is end of baker chain
  jacsolver_baker->JntToJac(chi_f_baker,Jf_baker);

  //reference frame and reference point should be baker root
  changeRefPoint(Jf_baker,-baker_pose.p,Jf_baker);
  changeRefFrame(Jf_spatula,Frame(pose.M, pose.p),Jf_spatula);

  for(unsigned int i=0;i<NC;i++)
    if(i<N_CHIF_BAKER)
      Jf_total.setColumn(i,Jf_baker.getColumn(i));
    else
      Jf_total.setColumn(i,Jf_spatula.getColumn(i-N_CHIF_BAKER));


  // concatenate the two chi_f
  for(unsigned int i=0; i < 3; i++)
    chi_f(i) = chi_f_baker(i);
  for(unsigned int i=3; i < 6; i++)
    chi_f(i) = chi_f_spatula(i-3);

}

void ChainTask::RPY_angles(Rotation rot)
{
  rot.GetRPY(chi_f_spatula(0),chi_f_spatula(1),chi_f_spatula(2));
  
  geometry_msgs::PoseStamped pp;

  Frame ff = Frame(Rotation::RPY(desired_values[3],desired_values[4],desired_values[5]));

  // stupid hack for rviz (running locally, wtf?!)
  pp.header.stamp = ros::Time::now() - ros::Duration(0.1);
  pp.header.frame_id = "/spatula";
  tf::PoseKDLToMsg(ff, pp.pose);
  ros_desired_pose_port.write(pp);

  // compute Jacobian matrix

  //reference frame is spatula, reference point is end of spatula chain
  jacsolver_spatula->JntToJac(chi_f_spatula_init, Jf_spatula);
}


void ChainTask::compute_angles(Frame frame, double *a0, double *a1, double *a2)
{

  Vector rx = frame.M.UnitX();
  Vector ry = frame.M.UnitY();
  Vector rz = frame.M.UnitZ();

  if(a0)
    *a0 = dot(ry, Vector(1,0,0)); // front edge aliged <=> a0 == 0

  if(a1)
    *a1 = dot(ry, Vector(0,0,1)); // side edge aligned <=> a1 == 0

  // tool direction:
  // * look from above -> project rz onto y-z-plane , yields rzp
  // * take angle with z
  // (tool direction towards center <=> a3 == 1)

  Vector rzp = Vector(rx.z(), 0, rz.z());

  if(a2)
    *a2 = dot(rzp, Vector(0,0,1)) / rzp.Norm();
}

void ChainTask::derive_angles(Frame frame, double dd)
{
  double a0, a1, a2;

  compute_angles(frame, &a0, &a1, &a2);
  Vector an(a0, a1, a2);

  chi_f_spatula(0) = a0;
  chi_f_spatula(1) = a1;
  chi_f_spatula(2) = a2;

  Twist tx(Vector(0,0,0), Vector(1,0,0));
  Twist ty(Vector(0,0,0), Vector(0,1,0));
  Twist tz(Vector(0,0,0), Vector(0,0,1));

  Frame fx = addDelta(frame, tx, dd);
  compute_angles(fx, &a0, &a1, &a2);
  Vector ax(a0, a1, a2);
  
  Frame fy = addDelta(frame, ty, dd);
  compute_angles(fy, &a0, &a1, &a2);
  Vector ay(a0, a1, a2);

  Frame fz = addDelta(frame, tz, dd);
  compute_angles(fz, &a0, &a1, &a2);
  Vector az(a0, a1, a2);

  Vector dx = -(ax - an) / dd;
  Vector dy = -(ay - an) / dd;
  Vector dz = -(az - an) / dd;

  jacinv(0,0) = dx.x(); jacinv(0,1) = dy.x(); jacinv(0,2) = dz.x();
  jacinv(1,0) = dx.y(); jacinv(1,1) = dy.y(); jacinv(1,2) = dz.y();
  jacinv(2,0) = dx.z(); jacinv(2,1) = dy.z(); jacinv(2,2) = dz.z();

  // now invert this
  svd_eigen_HH(jacinv, U, S, V, tmp);

  double eps = 1e-7;
  for(int i=0; i < 3; ++i)
      Sp(i) = (S(i) > eps) ? 1.0 / S(i) : 0.0;

  jac = V * Sp.asDiagonal() * U.transpose();

  Jf_spatula.setColumn(0, Twist(Vector(0,0,0), Vector(jac(0,0), jac(1,0), jac(2,0))));
  Jf_spatula.setColumn(1, Twist(Vector(0,0,0), Vector(jac(0,1), jac(1,1), jac(2,1))));
  Jf_spatula.setColumn(2, Twist(Vector(0,0,0), Vector(jac(0,2), jac(1,2), jac(2,2))));
}
