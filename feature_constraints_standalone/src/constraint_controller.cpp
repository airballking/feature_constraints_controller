
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
   : tf_lookup_thread_(0), robot_kinematics_(0), joint_state_interpreter_(0)
{

}


FeatureConstraintsController::~FeatureConstraintsController()
{
  // stop and delete thread that does the tf-lookup
  stop_tf_lookup_ = true;
  tf_lookup_thread_->join();
  delete tf_lookup_thread_;

  // clean up all the rest
  delete joint_state_interpreter_;
  delete robot_kinematics_;
}


bool FeatureConstraintsController::init(ros::NodeHandle &n)
{
  // remember nodeHandle
  n_ = n;
  
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

  // load relevant tf-frame-names from parameter server
  if(!load_frame_names(n))
    return false;
  
  // start thread that looks up tf-frames in paused-state
  stop_tf_lookup_ = false;
  pause_tf_lookup_ = true;
  tf_lookup_thread_ = new boost::thread( boost::bind( &FeatureConstraintsController::tf_poses_lookup, this) );
  
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

  // read controller-id from parameter server
  if (!n.getParam("controller_id", feature_controller_.controller_id_)){
    ROS_ERROR("No controller_id given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // resize size of message that reports the state of the joint limit avoidance
  resize(joint_avoidance_state_msg_, dof);

  // resize msg that will be used to publish desired joint velocities
  qdot_msg_.data.resize(dof);

  // resize size of weights for joints in solver
  // identity means that all joints will be considered with equal weights
  Wq_ = Eigen::MatrixXd::Identity(dof, dof);
  n.param<std::string>("object_frame", state_msg_.header.frame_id, "/base_link");

  // construct namespace to find filter-chain parameters (provided to and used
  // by feature_controller)
  filter_namespace_ = n.getNamespace() + std::string("/filter_chain");

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
  {
    // assertion: parsing was successful
    // calculate delta_t between the control cycles, i.e. joint-state messages
    ros::Time time = msg->header.stamp;
    ros::Duration dt = time - last_time_;
    last_time_ = time;
    
    // perform actual controller calculation
    update(dt.toSec());
  }
  else
    ROS_WARN("[Feature Controller] Could not parse joint state message.");
}

void FeatureConstraintsController::tf_poses_lookup()
{
  double tf_update_duration = 20.0;
  n_.getParam("tf_update_duration", tf_update_duration);
  ros::Rate rate(tf_update_duration);
 
  while(ros::ok() && !stop_tf_lookup_)
  {
    // sleep to not swamp the CPU
    rate.sleep();
    
    // actual functionality
    if(!pause_tf_lookup_)
    {
    boost::mutex::scoped_lock scoped_lock(tf_lookup_mutex_);
    lookup_tf_frames();
    }
  }
}

bool FeatureConstraintsController::lookup_tf_frames()
{
  // lookup pose of tool-frame in ee-frame
  try
  {
    tf_listener_.lookupTransform(arm_ee_frame_, tool_frame_,
                                 ros::Time(0), aux_transform_);
  }
  catch (tf::TransformException ex)
  {
    if(started_) 
      ROS_ERROR("Lookup of pose of tool-frame in ee-frame failed.");
    return false;
  }
  tf::TransformTFToKDL(aux_transform_, T_tool_in_ee_);
    
  // lookup pose of object-frame in world-frame
  try
  {
    tf_listener_.lookupTransform(world_frame_, object_frame_, 
                                 ros::Time(0), aux_transform_);
  }
  catch (tf::TransformException ex)
  {
    if(started_)
      ROS_ERROR("Lookup of pose of object-frame in world-frame failed.");
    return false;
  }
  tf::TransformTFToKDL(aux_transform_, T_object_in_world_);

  // lookup pose of base-frame in world-frame
  try
  {
    tf_listener_.lookupTransform(world_frame_, robot_base_frame_,
                                 ros::Time(0), aux_transform_);
  }
  catch (tf::TransformException ex)
  {
    if(started_)
      ROS_ERROR("Lookup of pose of robot-base-frame in world-frame failed.");
    return false;
  }
  tf::TransformTFToKDL(aux_transform_, T_base_in_world_);

  // lookup pose of arm-base-frame in robot-base-frame
  try
  {
  tf_listener_.lookupTransform(robot_base_frame_, arm_base_frame_,
                               ros::Time(0), aux_transform_);
  }
  catch (tf::TransformException ex)
  {
    if(started_)
      ROS_ERROR("Lookup of pose of arm-base-frame in robot-base-frame failed.");
    return false;
  }
  tf::TransformTFToKDL(aux_transform_, T_arm_in_base_);

  // all lookups worked out fine
  return true;
}

bool FeatureConstraintsController::load_frame_names(ros::NodeHandle& n)
{
  // get frame names from parameter server
  // TODO: change that name into 'arm_tool_frame'
  // note: launch-file...
  if (!n.getParam("tool_frame", arm_ee_frame_)){
    ROS_ERROR("No tool_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // TODO: change that name into 'arm_base_frame'
  // note: launch-file... 
  if (!n.getParam("base_frame", arm_base_frame_)){
    ROS_ERROR("No base_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // note: launch-file... 
  if (!n.getParam("robot_base_frame", robot_base_frame_)){
    ROS_ERROR("No robot_base_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // note: launch-file... 
  if (!n.getParam("map_frame", world_frame_)){
    ROS_ERROR("No map_frame given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // note: will be overwritten with data frame constraint_config
  tool_frame_ = arm_ee_frame_;
  object_frame_ = arm_base_frame_;

  return true;
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
  boost::mutex::scoped_lock scoped_lock(tf_lookup_mutex_);
 
  // check if we should really process this message
  if(feature_controller_.controller_id_.compare(msg->controller_id))
  {
    ROS_ERROR("Received constraint configuration with non-matching controller-id!: My id: %s, received id: %s Aborting...", 
      feature_controller_.controller_id_.c_str(), msg->controller_id.c_str());
    return;
  }

  // remember the current movement-id
  feature_controller_.movement_id_ = msg->movement_id;
 
  // get the number of constraints 
  unsigned int num_constraints = msg->constraints.size();
  if(num_constraints == 0)
  {
    ROS_ERROR("Received constraint configuration with 0 constraints. Aborting...");
    return;
  }

  // stop controller because we received a new configuration and will resize
  // the data structure inside the controller
  started_ = false;
  pause_tf_lookup_ = true;

  // extract frame_ids of tool and object from configs
  // first check, if they are all the same
  std::string temp_tool_frame = msg->constraints[0].tool_feature.frame_id;
  std::string temp_object_frame = msg->constraints[0].world_feature.frame_id;

  for(unsigned int i=0; i<num_constraints; i++)
  {
    if(temp_tool_frame != msg->constraints[i].tool_feature.frame_id){
      ROS_ERROR("Constraint Configuration contained two different tool frame-ids: '%s' and '%s'",
        temp_tool_frame.c_str(), msg->constraints[i].tool_feature.frame_id.c_str());
      configured_ = false;
      return;
    }
    if(temp_object_frame != msg->constraints[i].world_feature.frame_id){
      ROS_ERROR("Constraint Configuration contained two different object frame-ids: '%s' and '%s'",
        temp_object_frame.c_str(), msg->constraints[i].world_feature.frame_id.c_str());
      configured_ = false;
      return;
    }
  }
  // now copy frame-ids into class variables
  object_frame_ = temp_object_frame;
  tool_frame_ = temp_tool_frame;

  // start looking up tf-frames
  if(!lookup_tf_frames())
  {
    ROS_ERROR("Aborting configuration of controller because not all tf-transformations could be looked up.");
    return;
  }
  pause_tf_lookup_ = false;
  
  // resize the constraints inside the controller
  // NOTE: As implemented, this is not realtime safe!
  feature_controller_.constraints.resize(num_constraints);

  // get new constraints into controller and re-prepare it
  // i.e. adjust size of internal variables
  fromMsg(*msg, feature_controller_.constraints);
  bool success = feature_controller_.prepare(filter_namespace_);

  // clear command, since we have no data to fill it, yet.
  // NOTE: We currently require, that the config arrives first.
  //       This should probably be revisited!
  feature_controller_.command.reset();
  feature_controller_.intermediate_command.reset();

  // resize message used to report state of controller
  resize(state_msg_, num_constraints);
  // TODO(Georg): refactor this because we broke the abstraction between constraint controller and feature controller
  state_msg_.robot_jacobian.resize(7);
  state_msg_.transformed_interaction_matrix.resize(num_constraints);

  // set gains of feature controller to whatever we want
  for(unsigned int i=0; i < num_constraints; i++)
  {
    feature_controller_.p_gains(i) = 100.0;
    feature_controller_.d_gains(i) = 20.0;
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

  // set flags to indicate that we are configured 
  if(success)
  {
    ROS_INFO("Received Contraint Configuration, successfully configured controller.");
    configured_ = true;
  }
  else
  {
    ROS_INFO("Received Contraint Configuration, but configurating failed. Aborting.");
    configured_ = false;
  }
}

/*! This updates the Controller 'set points'. A fitting message of this type
 *  must be received after a configuration update in order to activate the
 *  controller.
 */
void FeatureConstraintsController::constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg)
{
  // check if we should really process this message
  if(feature_controller_.controller_id_.compare(msg->controller_id))
  {
    ROS_ERROR("Received constraint command with non-matching controller-id!: My id: %s, received id: %s Not starting...", 
      feature_controller_.controller_id_.c_str(), msg->controller_id.c_str());
    return;
  }

  // check if the movement-id of the command matches the one for which we got configured
  if(feature_controller_.movement_id_ != msg->movement_id)
  {
    ROS_ERROR("Received constraint command with non-matching movement-id!: My id: %ld, received id: %ld Not starting...", 
      feature_controller_.movement_id_, msg->movement_id);
    return;
  }

  // TODO(Georg): this sanity check for correspondence of config and command is naive
  //              we already encountered race conditions with this
  if(configured_ && (msg->pos_lo.size() == feature_controller_.command.pos_lo.rows())){
    ROS_INFO("Received valid Constraint Command, starting controller.");
    fromMsg(*msg, feature_controller_.command);
    started_ = true;
  }else{
    ROS_ERROR("Received Constraint Command, but the controller has not been properly configured. Not Starting.");
    started_ = false;
  }
}

void FeatureConstraintsController::update(double dt)
{
  boost::mutex::scoped_lock scoped_lock(tf_lookup_mutex_);

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

    // make sure that dt has a useful content
    if(!(dt > 0.0))
    {
      ROS_ERROR("[FeatureControllerStandalone] Skipping update because dt is not bigger than 0.0s: '%f'", dt);
      return;
    }

    // evaluate constraints, i.e. update the feature constraints and 
    // (depending on started_-flag) run feature controller for this cycle
    // TODO(Ingo): Why not always run the controller? If we ensure that the
    //            command is never 'evil', this should do no harm!
    feature_controller_.update(T_tool_in_object, true, dt);

    // if we have enough constraints and have been started: do the remaining calculations
    // and send commands to the robot
    // TODO: check if this check for constraints.size() is still necessary
    if((feature_controller_.constraints.size() > 0) && started_)
    {
      // and clamp desired feature velocities
      // TODO(Georg): put this back in
      //feature_controller_.clampOutput();

      // transform interaction matrix (ref frame base, ref point base)
      // NOTE: Discuss why we are transposing H_transpose here. --> Not intuitive.
      //       Update: It just dawned on me that we did this so that we could use
      //       KDL::Jacobian as an internal representation.
      H_feature_ = feature_controller_.Ht.data.transpose();

      // project interaction matrix from object-frame and to arm-base-frame because we want
      // to combine it with the robot_jacobian which we already transformed into arm-base-frame above
      // NOTE: long explanation...
      //       originally, the interaction matrix (H) maps from task-values-changes to twists.
      //       hence, the columns of H can be interpreted as twists.
      //       right now, these twists are expressed w.r.t. to the object frame.
      //       plücker coordinate transformation X_A_B map such twists from frame B to frame A.
      //       formula is: t_b = X_B_A * t_a.
      //       now, we have the transpose of a twist in object frame and want to change the reference frame to the base of the arm.
      //       in general, this means:
      //       t_b.transpose = (X_B_A * t_a).transpose
      //                     = t_a.transpose * X_B_A.transpose
      //       I just looked at page 39 of the handbook of robotics and proved to myself that:
      //       X_B_A.transpose = X_A_B
      //       Therefore, we use post-multiplication with the inverse twist projection to change the reference frame of a transposed twist.
      H_feature_ = H_feature_*inverse_twist_proj(T_arm_in_base_.Inverse() * T_base_in_world_.Inverse() * T_object_in_world_);
      
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
    // publish constraint state for debugging and visualization purposes
    toMsg(feature_controller_, state_msg_);
     
    // TODO(Georg): refactor this because we broke the abstraction between constraint_controller and feature_controller
    toMsg(jacobian_robot_, state_msg_.robot_jacobian);
    for(unsigned int i=0; i<H_feature_.rows(); i++)
    {
      toMsg(KDL::Twist(KDL::Vector(H_feature_(i,0), H_feature_(i,1), H_feature_(i,2)),
                       KDL::Vector(H_feature_(i,3), H_feature_(i,4), H_feature_(i,5))), state_msg_.transformed_interaction_matrix[i]);
    }
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
