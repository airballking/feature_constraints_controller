#include <robot_kinematics.h>

bool RobotKinematics::init(ros::NodeHandle& n)
{
  // get frame names from parameter server
  if (!n.getParam("tool_frame", tool_frame_id_)){
    ROS_ERROR("No tool_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  
  if (!n.getParam("base_frame", base_frame_id_)){
    ROS_ERROR("No base_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // get kinematic chain via urdf from parameter server
  KDL::Tree tree;
  if (!kdl_parser::treeFromParam("robot_description", tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  tree.getChain(base_frame_id_, tool_frame_id_, chain_);

  // init kdl solvers for forward kinematics and jacobian
  jnt_to_pose_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(chain_);

  return true;
}

std::vector<std::string> RobotKinematics::getJointNames()
{
  std::vector<std::string> result;
  for(uint i=0; i < chain_.getNrOfSegments(); ++i)
  {
    KDL::Joint jnt = chain_.getSegment(i).getJoint();
    if(jnt.getType() != KDL::Joint::None)
    {
      result.push_back(jnt.getName());
    }
  }
  return result;
}

void RobotKinematics::getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac)
{
  jnt_to_jac_solver_->JntToJac(q, jac);
}

void RobotKinematics::getForwardKinematics(const KDL::JntArray& q, KDL::Frame& frame)
{
  jnt_to_pose_solver_->JntToCart(q, frame);
}

unsigned int RobotKinematics::getNumberOfJoints()
{
  return chain_.getNrOfJoints();
}

JointStateInterpreter::JointStateInterpreter(std::vector<std::string> joint_names)
  : joint_names_(joint_names)
{
}

bool JointStateInterpreter::parseJointState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q)
{
  assert(msg != NULL);
  assert(q.rows() == joint_names_.size());

  unsigned int i, jnt;
  for(jnt=0, i=0; i < msg->name.size() && jnt < joint_names_.size(); i++)
    if(msg->name[i] == joint_names_[jnt])
      q(jnt++) = msg->position[i];

  return (jnt == joint_names_.size());
}
