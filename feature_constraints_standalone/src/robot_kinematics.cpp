#include <robot_kinematics.h>

RobotKinematics::RobotKinematics()
  : jnt_to_pose_solver_(0), jnt_to_jac_solver_(0)
{

}


RobotKinematics::~RobotKinematics()
{
  delete jnt_to_pose_solver_;
  delete jnt_to_jac_solver_;
}


bool RobotKinematics::init(ros::NodeHandle& n)
{
  // get frame names from parameter server
  // TODO: change that name into 'kinematics_EE_frame'
  // note: launch-file...
  if (!n.getParam("tool_frame", tool_frame_id_)){
    ROS_ERROR("No tool_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
 
  // TODO: change that name into 'kinematics_base_frame'
  // note: launch-file... 
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
  : joint_name_to_index_map_(), aux_iterator_(), joint_names_(joint_names)
{
  // building map to get from joint names to their index position in q-vector
  joint_name_to_index_map_.clear();
  for(unsigned int i=0; i<joint_names_.size(); i++)
  {
    joint_name_to_index_map_.insert(JointNameIndexPair(joint_names_[i], i));
    ROS_INFO("Adding joint '%s' to joint-index-map at index %d.", joint_names_[i].c_str(), i);
  }
}

bool JointStateInterpreter::parseJointState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q)
{
  assert(msg != NULL);
  assert(q.rows() == joint_names_.size());

  // iterate over message and use joint names -> joint index map to check for
  // each joint whether it is part of the current kinematic chain.
  // if yes, update the corresponding entry in q.
  unsigned int joint_counter = 0;
  for(unsigned int i=0; i<msg->name.size(); i++)
  {
    unsigned int jnt = getJointIndex(msg->name[i]);
    if(jnt < joint_names_.size())
    {
      // joint is one of the wanted ones
      q(jnt) = msg->position[i];
      joint_counter++;
    }
  }
  return (joint_counter == joint_names_.size());
}

unsigned int JointStateInterpreter::getJointIndex(const std::string joint_name)
{
  aux_iterator_ = joint_name_to_index_map_.find(joint_name);
  if(aux_iterator_ != joint_name_to_index_map_.end())
    return aux_iterator_->second; 
  else
    return joint_names_.size() + 1;
}
