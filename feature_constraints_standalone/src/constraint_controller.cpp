
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
  Identity_joints_ = Eigen::MatrixXd::Identity(dof, dof);

  // resize internal joint limit avoidance structures and controller 
  joint_limit_avoidance_controller_.prepare(robot_kinematics_->getJointNames(),
  robot_kinematics_->getSoftLowerJointLimits(), robot_kinematics_->getSoftUpperJointLimits());

  // read from the parameter server whether the joint limit avoidance shall be switched on
  if (!n.getParam("joint_limit_avoidance", joint_limit_avoidance_on_)){
    ROS_ERROR("Flag for joint_limit_avoidance not given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }
  if(!joint_limit_avoidance_on_)
    ROS_WARN("Joint limit avoidance controller has been deactivated.");

  // resize size of message that reports the state of the joint limit avoidance
  resize(joint_avoidance_state_msg_, dof);

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

  feature_controller_.constraints.resize(num_constraints);

  // get new constraints into controller and re-prepare it
  // i.e. adjust size of internal variables
  fromMsg(*msg, feature_controller_.constraints);
  feature_controller_.prepare();
  resize(state_msg_, num_constraints);

  // set gains of feature controller to whatever we want
  for(unsigned int i=0; i < num_constraints; i++)
  {
    feature_controller_.gains(i) = 25.0;
  }

  // resize interaction matrix for feature controller
  H_feature_ = Eigen::MatrixXd::Zero(num_constraints, 6);

  // resize matrices provided to solver and the solver itself
  A_ = Eigen::MatrixXd::Zero(num_constraints, q_.rows());
  A_inv_ = Eigen::MatrixXd::Zero(q_.rows(), num_constraints);
  Wy_ = Eigen::MatrixXd::Zero(num_constraints, num_constraints);
  ydot_ = Eigen::MatrixXd::Zero(num_constraints, 1);
  ROS_INFO("Solver gets dimensions %dx%d", num_constraints, q_.rows());
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
  if(configured_)
  {
    // assemble frame between tool and object
    KDL::Frame ff_kinematics;
    robot_kinematics_->getForwardKinematics(q_, ff_kinematics);
    robot_kinematics_->getJacobian(q_, jacobian_robot_);

    // transform robot jacobian (ref frame base, ref point base)
    jacobian_robot_.changeRefPoint(-ff_kinematics.p);

    KDL::Frame T_tool_in_object = T_object_in_world_.Inverse() * T_base_in_world_ * T_arm_in_base_ * ff_kinematics * T_tool_in_ee_;

    // calculate output of joint limit avoidance controller for this cycle
    joint_limit_avoidance_controller_.update(q_);

    // evaluate constraints, i.e. update the feature constraints and 
    // (depending on started_-flag) run feature controller for this cycle
    feature_controller_.update(T_tool_in_object, started_);

    // if we have enough constraints and have been started: do the remaining calculations
    // and send commands to the robot
    // TODO: check if this check for constraints.size() is still necessary
    if((feature_controller_.constraints.size() > 0) && started_)
    {
      // and clamp desired feature velocities
      feature_controller_.clampOutput();

      // transform interaction matrix (ref frame base, ref point base)
      // NOTE: Discuss why we are transposing H_transpose here. --> Not intuitive.
      //       Update: It just dawned on me that we did this so that we could use
      //       KDL::Jacobian as an internal representation.
      H_feature_ = feature_controller_.Ht.data.transpose();
      H_feature_ = H_feature_*inverse_twist_proj(T_object_in_world_);

      // assemble problem for solver out of interaction matrix and robot jacobian
      A_ = H_feature_ * jacobian_robot_.data;
      
      // get desired constraint velocities and weights from feature controller
      ydot_ = feature_controller_.ydot.data;
      Wy_.diagonal() = feature_controller_.weights.data;

      // call the solver to get desired joint angles that achieve task and
      // weighted pseudoinverse of A_ to perform Nullspace projection
      solver_.solve(A_, ydot_, Wq_, Wy_, qdot_.data, A_inv_);
            
      // NULLSPACE PROJECTION: joint limit avoidance
      if(joint_limit_avoidance_on_)
      {
        // perform actual Nullspace projection
        assert(q_.rows() == qdot_.rows());
        assert(Identity_joints_.rows() == q_.rows());
	assert(Identity_joints_.cols() == q_.rows());
        assert(A_inv_.rows() == qdot_.rows());
        assert(A_inv_.cols() == A_.rows());
        assert(A_.cols() == qdot_.rows());
	assert(qdot_.rows() == joint_limit_avoidance_controller_.q_dot_desired_.rows());

        qdot_.data += (Identity_joints_ - A_inv_*A_)*joint_limit_avoidance_controller_.q_dot_desired_.data;
      }

      // publish desired joint velocities    
      assert(qdot_.rows() == qdot_msg_.data.size());
      for(unsigned int i=0; i<qdot_.rows(); i++)
      {
        qdot_msg_.data[i] = qdot_(i);
      }
      qdot_publisher_.publish(qdot_msg_);
    }  
    // END OF ACTUAL CONTROLLER -- START OF DEBUG PUBLISHING
    // publish constraint state for debugging purposes
    toMsg(feature_controller_, state_msg_);
    state_msg_.header.stamp = ros::Time::now();
    state_publisher_.publish(state_msg_);
    // publish state of joint limit avoidance
    toMsg(joint_limit_avoidance_controller_, joint_avoidance_state_msg_);
    joint_avoidance_state_msg_.header.stamp = ros::Time::now();
    limit_avoidance_state_publisher_.publish(joint_avoidance_state_msg_);  
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
