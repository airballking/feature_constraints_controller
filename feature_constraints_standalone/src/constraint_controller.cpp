
/*! \mainpage
 *  \section Standalone Feature Constraint Controller
 *
 *  This package demonstrates, how a feature-grounded constraint based
 *  controller can be instantiated.
 *
 *  \image html doc_images/control-coordinates.png
 *
 *  It assumes that a tool which is fixed to a robot shall be moved w.r.t. a
 *  static object in the world. The robot state is read from /joint_states,
 *  all other transforms are (assumed to be) taken from /tf.
 *
 *  \image html doc_images/controller-structure.png
 *
 *  The first step in defining constraints is the definition of an appropriate
 *  set of coordinates (constraint-coordinates). Then, during the movement, allowed
 *  ranges for these constraints are specified.
 *
 *  A python script sets up cylinder coordinates and three orientation constraints
 *  which can be controlled with a game controller
 *  (when starting joystick_commands.launch).
 *
 *  The constraint coordinates are visualized using Markers.
 */


#include <constraint_controller.h>
#include <tf_conversions/tf_kdl.h>

using namespace Eigen;

// transform an interaction matrix
Matrix<double, 6, 6> inverse_twist_proj(KDL::Frame f)
{
  // (transposed) Rotation matrix of f
  Matrix3d Rt = Map<Matrix3d>(f.M.data);

  double x = f.p.x(), y = f.p.y(), z = f.p.z();

  // Skew symmetric matrix of p, [p]_x for expressing a cross product
  Matrix3d px;
  px << 0, -z,  y,
        z,  0, -x,
       -y,  x,  0;

  // the inverse twist projection matrix
  Matrix<double, 6, 6> Mi;
  Mi.block<3,3>(0,0) = Rt;
  Mi.block<3,3>(3,3) = Rt;
  Mi.block<3,3>(0,3) = -Rt*px;
  Mi.block<3,3>(3,0) = Matrix3d::Zero();

  return Mi;
}



FeatureConstraintsController::FeatureConstraintsController()
   : robot_kinematics_(0), joint_state_interpreter_(0)
{

}


FeatureConstraintsController::~FeatureConstraintsController()
{
  delete joint_state_interpreter_;
  delete robot_kinematics_;
}


bool FeatureConstraintsController::init(ros::NodeHandle &n)
{
  // set frames from tf to dummy value
  // TODO: later put this into separate thread listening to TF
 
  // set correspondes between constraint function names and constraint functions
  Constraint::init();
 
  // construct and init robot kinematics
  robot_kinematics_ = new RobotKinematics();
  if(!robot_kinematics_->init(n))
  {
    ROS_ERROR("Init of robot_kinematics_ failed.!");
    return false;
  }

  // construct joint state interpreter
  joint_state_interpreter_ = new JointStateInterpreter(robot_kinematics_->getJointNames());
  ROS_INFO("Number of joints in the kinematic chain...%d", (int)robot_kinematics_->getJointNames().size());

  // subscribe to joint state topic
  joint_state_subscriber_ = n.subscribe<sensor_msgs::JointState>("/joint_states", 2, &FeatureConstraintsController::joint_state_callback, this);

  // subscribe to feature constraint topic
  feature_constraints_subscriber_ = n.subscribe<constraint_msgs::ConstraintConfig>("constraint_config", 1, &FeatureConstraintsController::feature_constraints_callback, this);

  // subscribe to constraint command topic
  constraint_command_subscriber_ = n.subscribe<constraint_msgs::ConstraintCommand>("constraint_command", 1, &FeatureConstraintsController::constraint_command_callback, this);

  // subscribe to tool offset pose
  tool_offset_subscriber_ = n.subscribe<geometry_msgs::Pose>("tool_offset", 1, &FeatureConstraintsController::tool_offset_callback, this);

  // subscribe to world offset pose
  object_offset_subscriber_ = n.subscribe<geometry_msgs::Pose>("object_offset", 1, &FeatureConstraintsController::object_offset_callback, this);

  // subscribe to robot arm offset pose
  arm_offset_subscriber_ = n.subscribe<geometry_msgs::Pose>("arm_offset", 1, &FeatureConstraintsController::robot_arm_offset_callback, this);

  // subscribe to robot arm offset pose
  base_pose_subscriber_ = n.subscribe<geometry_msgs::Pose>("base_pose", 1, &FeatureConstraintsController::robot_base_callback, this);

  // advertise publisher
  qdot_publisher_ = n.advertise<std_msgs::Float64MultiArray>("qdot", 1);
  state_publisher_ = n.advertise<constraint_msgs::ConstraintState>("constraint_state", 1);
  limit_avoidance_state_publisher_ = n.advertise<constraint_msgs::JointAvoidanceState>("joint_limit_avoidance_state", 1);
  // resize internal robot representation
  unsigned int dof = robot_kinematics_->getNumberOfJoints();
  q_.resize(dof);
  qdot_.resize(dof);
  jacobian_robot_.resize(dof);
  setupJointLimitAvoidanceController();

  // resize msg that will be used to publish desired joint velocities
  qdot_msg_.data.resize(dof);

  // resize size of weights for joints in solver
  // identity means that all joints will be considered with equal weights
  Wq_ = Eigen::MatrixXd::Identity(dof, dof);

  n.param<std::string>("object_frame", state_msg_.header.frame_id, "/base_link");

  // set flags to signal that we are not ready, yet
  configured_ = false;
  started_ = false;

  return true;
}

/*! This triggers an update.
 */
void FeatureConstraintsController::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // parse message with joint state interpreter
  if(joint_state_interpreter_->parseJointState(msg, q_))
    // assertion: parsing was successful
    update();
  else
    ROS_WARN("[Feature Controller] Could not parse joint state message.");
}

void FeatureConstraintsController::tool_offset_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  tf::PoseMsgToKDL(*msg, T_tool_in_ee_);
}

void FeatureConstraintsController::object_offset_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  tf::PoseMsgToKDL(*msg, T_object_in_world_);
}

void FeatureConstraintsController::robot_arm_offset_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  tf::PoseMsgToKDL(*msg, T_arm_in_base_);
}

void FeatureConstraintsController::robot_base_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  tf::PoseMsgToKDL(*msg, T_base_in_world_);
}
/*! This updates the controller and solver.
 *
 *  The controller is halted until a new, fitting constraint command is received.
 *
 *  \todo Both components do memory allocation, so this is not realtime safe.
 *  Reserving enough space at initialization time might be a solution...
 */
void FeatureConstraintsController::feature_constraints_callback(const constraint_msgs::ConstraintConfig::ConstPtr& msg)
{
  unsigned int num_constraints = msg->constraints.size();

  // get new constraints into controller and re-prepare it
  // i.e. adjust size of internal variables
  fromMsg(*msg, feature_controller_.constraints);
  feature_controller_.prepare();
  resize(state_msg_, num_constraints);

  for(unsigned int i=0; i < num_constraints; i++)
  {
    feature_controller_.gains(i) = 1.0;
  }
  // resize weights for constraints according to the number of weights
  // (note: will be used by solver)
  Wy_ = Eigen::MatrixXd::Zero(num_constraints, num_constraints);

  // resize constraint matrix and interaction matrix
  A_ = Eigen::MatrixXd::Zero(num_constraints, q_.rows());
  H_ = Eigen::MatrixXd::Zero(num_constraints, 6);

  // resize solver
  solver_.reinitialise(num_constraints, q_.rows());
  // set flags to indicate that we are configured but not started, yet
  configured_ = true;
  started_ = false;
}


/*! This updates the Controller 'set points'. A fitting message of this type
 *  must be received after a configuration update in order to activate the
 *  controller.
 */
void FeatureConstraintsController::constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg)
{
  if(configured_ && (msg->pos_lo.size() == feature_controller_.command.pos_lo.rows())){
    fromMsg(*msg, feature_controller_.command);
    started_ = true;
  }else{
    started_ = false;
  }
}

void FeatureConstraintsController::update()
{
  if(configured_ && started_)
  {
    // assemble frame between tool and object
    KDL::Frame ff_kinematics;
    robot_kinematics_->getForwardKinematics(q_, ff_kinematics);
    robot_kinematics_->getJacobian(q_, jacobian_robot_);

    // transform robot jacobian (ref frame base, ref point base)
    jacobian_robot_.changeRefPoint(-ff_kinematics.p);

    KDL::Frame T_tool_in_object = T_object_in_world_.Inverse() * T_base_in_world_ * T_arm_in_base_ * ff_kinematics * T_tool_in_ee_;

    //ROS_INFO("[FeatureConstraintsController] Update Point 1");

    if(feature_controller_.constraints.size() > 0)
    {  
      // evaluate constraints
      feature_controller_.update(T_tool_in_object);
      //ROS_INFO("[FeatureConstraintsController] Update Point 2");

      // transform interaction matrix (ref frame base, ref point base)
      // TODO: Discuss why we are transposing H_transpose here. --> Not intuitive.
      //       Update: It just dawned on me that we did this so that we could use
      //       KDL::Jacobian as an internal representation. @Ingo: Right?
      H_ = feature_controller_.Ht.data.transpose();
      H_ = H_*inverse_twist_proj(T_object_in_world_);
      //ROS_INFO("[FeatureConstraintsController] Update Point 2");

      // call solver with values from constraint controller and Jacobian
      A_ = H_ * jacobian_robot_.data;
      Wy_.diagonal() = feature_controller_.weights.data;
      //  ROS_INFO("[FeatureConstraintsController] Update Point 3");

      // TODO: do joint avoidance here!
      control(qdot_joint_, weights_joint_, q_desired_joint_, q_, joint_limit_command_, gains_joint_); 

      // some old debug output
      ROS_DEBUG_STREAM("A_"<<A_<<"\n");
      ROS_DEBUG_STREAM("ydot_"<<feature_controller_.ydot.data<<"\n");
      ROS_DEBUG_STREAM("Wq_"<<Wq_<<"\n");
      ROS_DEBUG_STREAM("Wy_"<<Wy_<<"\n");
      ROS_DEBUG_STREAM("qdot_"<<qdot_.data<<"\n");
      ROS_DEBUG_STREAM("chi_"<<feature_controller_.chi.data<<"\n");
 
      solver_.solve(A_, feature_controller_.ydot.data, Wq_, Wy_, qdot_.data);
      //ROS_INFO("[FeatureConstraintsController] Update Point 4");

      // publish state of joint limit avoidance
      // TODO: so need to refactor this into a conversion function, once we have an extra joint limit avoidance controller
      assert(q_.rows() == joint_avoidance_state_msg_.q.size());
      assert(joint_limit_command_.pos_lo.rows() == joint_avoidance_state_msg_.q_lower_limits.size());
      assert(joint_limit_command_.pos_hi.rows() == joint_avoidance_state_msg_.q_upper_limits.size());
      assert(q_desired_joint_.rows() == joint_avoidance_state_msg_.q_desired.size());
      assert(weights_joint_.rows() == joint_avoidance_state_msg_.weights.size());
      assert(qdot_joint_.rows() == joint_avoidance_state_msg_.q_dot_desired.size());
      for(unsigned int i=0; i<q_.rows(); i++)
      { 
        joint_avoidance_state_msg_.q[i] = q_(i);
      }
      for(unsigned int i=0; i<joint_limit_command_.pos_lo.rows(); i++)
      { 
        joint_avoidance_state_msg_.q_lower_limits[i] = joint_limit_command_.pos_lo(i);
      }
      for(unsigned int i=0; i<joint_limit_command_.pos_hi.rows(); i++)
      { 
        joint_avoidance_state_msg_.q_upper_limits[i] = joint_limit_command_.pos_hi(i);
      }
      for(unsigned int i=0; i<q_desired_joint_.rows(); i++)
      { 
        joint_avoidance_state_msg_.q_desired[i] = q_desired_joint_(i);
      }
      for(unsigned int i=0; i<weights_joint_.rows(); i++)
      { 
        joint_avoidance_state_msg_.weights[i] = weights_joint_(i);
      }
      for(unsigned int i=0; i<qdot_joint_.rows(); i++)
      { 
        joint_avoidance_state_msg_.q_dot_desired[i] = qdot_joint_(i);
      }
      // publish desired joint velocities    
      assert(qdot_.rows() == qdot_msg_.data.size());
      for(unsigned int i=0; i<qdot_.rows(); i++)
      {
        qdot_msg_.data[i] = qdot_(i);
      }
      joint_avoidance_state_msg_.header.stamp = ros::Time::now();
      limit_avoidance_state_publisher_.publish(joint_avoidance_state_msg_);

      qdot_publisher_.publish(qdot_msg_);
      // publish constraint state for debugging purposes
      ///feature_controller_.Ht.data = (feature_controller_.Ht.data.transpose() * inverse_twist_proj(KDL::Frame(-T_tool_in_object.p))).transpose();
      toMsg(feature_controller_, state_msg_);
      state_msg_.header.stamp = ros::Time::now();
      state_publisher_.publish(state_msg_);
    }  
  }
}

// sets up the structures for internal joint limit avoidance
// TODO: Refactoring in either a) an own joint limit avoidance class
//       or b) (even better) into the constraint-msg interface are
//       desirable for the future. Right now, I --Georg-- just wanna
//       see this working
void FeatureConstraintsController::setupJointLimitAvoidanceController()
{
  // get necessary input from the robot kinematics
  unsigned int dof = robot_kinematics_->getNumberOfJoints();
  std::vector<double> lower_limits = robot_kinematics_->getSoftLowerJointLimits();
  std::vector<double> upper_limits = robot_kinematics_->getSoftUpperJointLimits();
  std::vector<std::string> joint_names = robot_kinematics_->getJointNames();

  // check sanity of input
  assert(dof == lower_limits.size());
  assert(dof == upper_limits.size());
  assert(dof == joint_names.size());
  assert(dof > 0);

  // resize internal structures
  joint_limit_command_.resize(dof);
  A_joint_ = Eigen::MatrixXd::Identity(dof, dof);
  Wy_joint_ = Eigen::MatrixXd::Zero(dof, dof);
  qdot_joint_.resize(dof);
  q_desired_joint_.resize(dof);
  weights_joint_.resize(dof);
  gains_joint_.resize(dof);

  // set gains for joint limit avoidance
  for(unsigned int i=0; i<gains_joint_.rows(); i++)
  {
    gains_joint_(i) = 1.0;
  }

  // adjust the command according to input from robot kinematics
  for(unsigned int i=0; ((i<lower_limits.size())&& (i<upper_limits.size())); i++)
  {
    if((lower_limits[i]==upper_limits[i])&&(lower_limits[i]==0.0))
    {
      // this is a joint without joint limits!
      ROS_WARN("Joint '%s' is interpreted to be without limits!", joint_names[i].c_str());
      joint_limit_command_.weight(i) = 0.0;
    }
    else
    {
      // joint has limits
      joint_limit_command_.weight(i) = 1.0;
      joint_limit_command_.pos_lo(i) = lower_limits[i];
      joint_limit_command_.pos_hi(i) = upper_limits[i];
    }
  }

  // resize msgs that will be used to publish state of joint limit avoidance
  joint_avoidance_state_msg_.joint_names.resize(dof);
  joint_avoidance_state_msg_.q.resize(dof);
  joint_avoidance_state_msg_.q_lower_limits.resize(dof);
  joint_avoidance_state_msg_.q_upper_limits.resize(dof);
  joint_avoidance_state_msg_.q_desired.resize(dof);
  joint_avoidance_state_msg_.weights.resize(dof);
  joint_avoidance_state_msg_.q_dot_desired.resize(dof);

  // already fill in the names of the joints in the state message
  for(unsigned int i=0; i<joint_avoidance_state_msg_.joint_names.size(); i++)
  {
    joint_avoidance_state_msg_.joint_names[i] = joint_names[i];
  }
}

// The entry to our application...
int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_controller");
  ros::NodeHandle n("~");
  FeatureConstraintsController myController;
  myController.init(n);
  ros::spin();
  return 0;
}
