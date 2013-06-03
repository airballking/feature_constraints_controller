#include <feature_constraints/Conversions.h>
#include <tf_conversions/tf_kdl.h>

// ROS message interface

// most functions exist twice to allow for realtime execution
// TODO: make sure the classes involving strings are handled right.

using namespace KDL;

// The functions taking two arguments _should_ be realtime safe:
// Only copying occurs, no memory allocation. We trust on
// strings to have copy-on-write semantics, however.

void fromMsg(const geometry_msgs::Vector3& msg, Vector& vec)
{
  vec[0] = msg.x;
  vec[1] = msg.y;
  vec[2] = msg.z;
}

Vector fromMsg(const geometry_msgs::Vector3& msg)
{
  return Vector(msg.x, msg.y, msg.z);
}


void fromMsg(const constraint_msgs::Feature& msg, Feature& feat)
{
  feat.name = msg.name;
  fromMsg(msg.position, feat.pos);
  fromMsg(msg.direction, feat.dir);
  fromMsg(msg.contact_direction, feat.contact_dir);
}

void fromMsg(const std::vector<constraint_msgs::Feature>& msg, std::vector<Feature>& features)
{
  assert(msg.size() == features.size());
  for(unsigned int i=0; i<msg.size(); i++)
  {
    fromMsg(msg[i], features[i]);
  }
}

Feature fromMsg(const constraint_msgs::Feature& msg)
{
  return Feature(msg.name, fromMsg(msg.position), fromMsg(msg.direction), fromMsg(msg.contact_direction));
}


void fromMsg(const constraint_msgs::Constraint& msg, Constraint& c)
{
  c.name = msg.name;
  c.setFunction(msg.function);
  fromMsg(msg.tool_feature, c.tool_feature);
  fromMsg(msg.world_feature, c.object_feature);
}


void fromMsg(const std::vector<constraint_msgs::Constraint>& msg, std::vector<Constraint>& c)
{
  for(unsigned int i=0; i < msg.size(); i++)
    fromMsg(msg[i], c[i]);
}


void fromMsg(const constraint_msgs::ConstraintConfig& msg, std::vector<Constraint>& cc)
{
  for(unsigned int i=0; i < msg.constraints.size(); i++)
    fromMsg(msg.constraints[i], cc[i]);
}

Constraint fromMsg(const constraint_msgs::Constraint& msg)
{
  return Constraint(msg.name, msg.function,
                    fromMsg(msg.tool_feature), fromMsg(msg.world_feature));
}


std::vector<Constraint> fromMsg(const std::vector<constraint_msgs::Constraint>& msg)
{
  std::vector<Constraint> cc(msg.size());
  fromMsg(msg, cc);
  return cc;
}


void fromMsg(const std::vector<double>& msg, KDL::JntArray& jnts)
{
  for(unsigned int i=0; i < msg.size(); i++)
    jnts(i) = msg[i];
}

KDL::JntArray fromMsg(std::vector<double>& msg)
{
  JntArray jnts(msg.size());
  fromMsg(msg, jnts);
  return jnts;
}

void fromMsg(const constraint_msgs::ConstraintCommand& msg, Ranges& ranges)
{
  fromMsg(msg.pos_lo, ranges.pos_lo);
  fromMsg(msg.pos_hi, ranges.pos_hi);
  fromMsg(msg.weight, ranges.weight);
  fromMsg(msg.max_vel, ranges.max_vel);
  fromMsg(msg.min_vel, ranges.min_vel);
}

Ranges fromMsg(constraint_msgs::ConstraintCommand& msg)
{
  Ranges ranges(msg.pos_lo.size());
  fromMsg(msg, ranges);
  return ranges;
}


void toMsg(KDL::Frame& frame, geometry_msgs::Pose& pose)
{
  tf::PoseKDLToMsg(frame, pose);
}


void toMsg(KDL::JntArray& joints, std::vector<double>& msg)
{
  for(unsigned int i=0; i < joints.rows(); i++)
    msg[i] = joints(i);
}


void toMsg(const KDL::Twist& t, geometry_msgs::Twist& t_msg)
{
  tf::TwistKDLToMsg(t, t_msg);
}


void toMsg(KDL::Jacobian& jac, std::vector<geometry_msgs::Twist>& jac_msg)
{
  for(unsigned int i=0; i < jac.columns(); i++)
    toMsg(jac.getColumn(i), jac_msg[i]);
}


void toMsg(Controller& c, constraint_msgs::ConstraintState& msg)
{
  toMsg(c.frame, msg.pose);
  toMsg(c.chi, msg.chi);
  toMsg(c.chi_dot, msg.chi_dot);
  toMsg(c.chi_desired, msg.chi_desired);
  toMsg(c.chi_dot_desired, msg.chi_dot_desired);
  toMsg(c.ydot, msg.ydot_desired);
  toMsg(c.weights, msg.weights);
  toMsg(c.J, msg.jacobian);
  toMsg(c.Ht, msg.interaction_matrix);
  toMsg(c.singularValues, msg.singular_values);
  toMsg(c.command.pos_lo, msg.lower_boundary);
  toMsg(c.command.pos_hi, msg.upper_boundary);
  toMsg(c.command.weight, msg.command_weights);
  for(unsigned int i=0; i < c.constraints.size(); i++)
    msg.constraint_names[i] = c.constraints[i].name;
}

void toMsg(JointLimitAvoidanceController& c, constraint_msgs::JointAvoidanceState& c_msg)
{
  assert(c.joint_names_.size() == c_msg.joint_names.size());
  for(unsigned int i=0; i<c.joint_names_.size(); i++)
  {
    c_msg.joint_names[i] = c.joint_names_[i];
  }

  toMsg(c.q_, c_msg.q);
  toMsg(c.command_.pos_lo, c_msg.q_lower_limits);
  toMsg(c.command_.pos_hi, c_msg.q_upper_limits);
  toMsg(c.q_desired_, c_msg.q_desired);
  toMsg(c.weights_, c_msg.weights);
  toMsg(c.q_dot_desired_, c_msg.q_dot_desired);
}

// convenience functions

geometry_msgs::Pose toMsg(KDL::Frame& frame)
{
  geometry_msgs::Pose pose;
  toMsg(frame, pose);
  return pose;
}


std::vector<double> toMsg(KDL::JntArray& joints)
{
  std::vector<double> msg(joints.rows());
  toMsg(joints, msg);
  return msg;
}


geometry_msgs::Twist toMsg(KDL::Twist& t)
{
  geometry_msgs::Twist t_msg;
  toMsg(t, t_msg);
  return t_msg;
}


std::vector<geometry_msgs::Twist> toMsg(KDL::Jacobian jac)
{
  std::vector<geometry_msgs::Twist> jac_msg(jac.columns());
  toMsg(jac, jac_msg);
  return jac_msg;
}


constraint_msgs::ConstraintState toMsg(Controller& c)
{
  constraint_msgs::ConstraintState c_msg;
  resize(c_msg, c.constraints.size());
  toMsg(c, c_msg);
  return c_msg;
}

void resize(constraint_msgs::ConstraintState &msg, unsigned int number_constraints)
{
  msg.chi.resize(number_constraints);
  msg.chi_dot.resize(number_constraints);
  msg.chi_desired.resize(number_constraints);
  msg.chi_dot_desired.resize(number_constraints);
  msg.ydot_desired.resize(number_constraints);
  msg.weights.resize(number_constraints);
  msg.lower_boundary.resize(number_constraints);
  msg.upper_boundary.resize(number_constraints);
  msg.command_weights.resize(number_constraints);
  msg.jacobian.resize(number_constraints);
  msg.interaction_matrix.resize(number_constraints);
  msg.singular_values.resize(6);
  msg.constraint_names.resize(number_constraints);
}

void resize(constraint_msgs::JointAvoidanceState &msg, unsigned int number_joints)
{
  msg.joint_names.resize(number_joints);
  msg.q.resize(number_joints);
  msg.q_lower_limits.resize(number_joints);
  msg.q_upper_limits.resize(number_joints);
  msg.q_desired.resize(number_joints);
  msg.weights.resize(number_joints);
  msg.q_dot_desired.resize(number_joints);
}
