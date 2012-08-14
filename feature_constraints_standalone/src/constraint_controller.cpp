
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

  // set ready-flag to signal that we are not ready, yet
  ready_ = false;

  return true;
}

/*! This triggers an update.
 */
void FeatureConstraintsController::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // parse message with joint state interpreter
  if(joint_state_interpreter_->parseJointState(msg, q_))
  {
    // assertion: parsing was successful
    update();
  }
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
  // get new constraints into controller and re-prepare it
  // i.e. adjust size of internal variables
  fromMsg(*msg, feature_controller_.constraints);
  feature_controller_.prepare();
  for(unsigned int i=0; i<feature_controller_.gains.rows(); i++)
  {
    feature_controller_.gains(i) = 1.0;
  }
  // resize weights for constraints according to the number of weights
  // (note: will be used by solver)
  Wy_ = Eigen::MatrixXd::Zero(feature_controller_.constraints.size(), feature_controller_.constraints.size());
  // resize solver
  solver_.reinitialise(feature_controller_.constraints.size(), q_.rows());
  // set ready-flag to indicate that we are ready to solve constraints
  ready_ = false;
}


/*! This updates the Controller 'set points'. A fitting message of this type
 *  must be received after a configuration update in order to activate the
 *  controller.
 */
void FeatureConstraintsController::constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg)
{
  if(msg->pos_lo.size() == feature_controller_.command.pos_lo.rows()){
    ready_ = true;
  }else{
    ready_ = false;
  }

  fromMsg(*msg, feature_controller_.command);
}

void FeatureConstraintsController::update()
{
  // assemble frame between tool and object
  KDL::Frame ff_kinematics;
  robot_kinematics_->getForwardKinematics(q_, ff_kinematics);
  robot_kinematics_->getJacobian(q_, jacobian_robot_);

  KDL::Frame T_tool_in_object = T_object_in_world_.Inverse() * T_base_in_world_ * ff_kinematics * T_tool_in_ee_;

  if(feature_controller_.constraints.size() > 0){  
    // evaluate constraints
    feature_controller_.update(T_tool_in_object);
 
    // call solver with values from constraint controller and Jacobian
 
    A_ = feature_controller_.Ht.data.transpose() * jacobian_robot_.data;
    Wy_.diagonal() = feature_controller_.weights.data;
 
    ROS_DEBUG_STREAM("A_"<<A_<<"\n");
    ROS_DEBUG_STREAM("ydot_"<<feature_controller_.ydot.data<<"\n");
    ROS_DEBUG_STREAM("Wq_"<<Wq_<<"\n");
    ROS_DEBUG_STREAM("Wy_"<<Wy_<<"\n");
    ROS_DEBUG_STREAM("qdot_"<<qdot_.data<<"\n");
    ROS_DEBUG_STREAM("chi_"<<feature_controller_.chi.data<<"\n");
 
    solver_.solve(A_, feature_controller_.ydot.data, Wq_, Wy_, qdot_.data);
 
    if(ready_){
      // publish desired joint velocities    
      assert(qdot_.rows() == qdot_msg_.data.size());
      for(unsigned int i=0; i<qdot_.rows(); i++)
      {
        qdot_msg_.data[i] = qdot_(i);
      }
      qdot_publisher_.publish(qdot_msg_);
    }

    // publish constraint state for debugging purposes
    state_publisher_.publish(toMsg(feature_controller_));
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
