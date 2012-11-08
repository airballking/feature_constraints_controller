
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

  // advertise publisher
  qdot_publisher_ = n.advertise<std_msgs::Float64MultiArray>("qdot", 1);
  state_publisher_ = n.advertise<constraint_msgs::ConstraintState>("constraint_state", 1);

  // resize internal robot representation
  unsigned int dof = robot_kinematics_->getNumberOfJoints();
  q_.resize(dof);
  qdot_.resize(dof);
  jacobian_robot_.resize(dof);

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
    started_ = true;
  }else{
    started_ = false;
  }

  fromMsg(*msg, feature_controller_.command);
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

    KDL::Frame T_tool_in_object = T_object_in_world_.Inverse() * T_base_in_world_ * ff_kinematics * T_tool_in_ee_;

    //ROS_INFO("[FeatureConstraintsController] Update Point 1");

    if(feature_controller_.constraints.size() > 0)
    {  
      // evaluate constraints
      feature_controller_.update(T_tool_in_object);
      //ROS_INFO("[FeatureConstraintsController] Update Point 2");

      // transform interaction matrix (ref frame base, ref point base)
      H_ = feature_controller_.Ht.data.transpose();
      H_ = H_*inverse_twist_proj(T_object_in_world_);
      //ROS_INFO("[FeatureConstraintsController] Update Point 2");

      // call solver with values from constraint controller and Jacobian
      A_ = H_ * jacobian_robot_.data;
      Wy_.diagonal() = feature_controller_.weights.data;
      //  ROS_INFO("[FeatureConstraintsController] Update Point 3");

      ROS_DEBUG_STREAM("A_"<<A_<<"\n");
      ROS_DEBUG_STREAM("ydot_"<<feature_controller_.ydot.data<<"\n");
      ROS_DEBUG_STREAM("Wq_"<<Wq_<<"\n");
      ROS_DEBUG_STREAM("Wy_"<<Wy_<<"\n");
      ROS_DEBUG_STREAM("qdot_"<<qdot_.data<<"\n");
      ROS_DEBUG_STREAM("chi_"<<feature_controller_.chi.data<<"\n");
 
      solver_.solve(A_, feature_controller_.ydot.data, Wq_, Wy_, qdot_.data);
      //ROS_INFO("[FeatureConstraintsController] Update Point 4");
      if(started_)
      {
        // publish desired joint velocities    
        assert(qdot_.rows() == qdot_msg_.data.size());
        for(unsigned int i=0; i<qdot_.rows(); i++)
        {
          qdot_msg_.data[i] = qdot_(i);
        }
        qdot_publisher_.publish(qdot_msg_);
      }
      // publish constraint state for debugging purposes
      ///feature_controller_.Ht.data = (feature_controller_.Ht.data.transpose() * inverse_twist_proj(KDL::Frame(-T_tool_in_object.p))).transpose();
      toMsg(feature_controller_, state_msg_);
      state_publisher_.publish(state_msg_);
    }  
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
