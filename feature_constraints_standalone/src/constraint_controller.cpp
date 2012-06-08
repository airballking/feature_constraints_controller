#include <constraint_controller.h>


bool FeatureConstraintsController::init(ros::NodeHandle &n)
{
  // set frames from tf to dummy value
  // TODO: later put this into separate thread listening to TF
  
  // construct and init robot kinematics
  robot_kinematics_ = new RobotKinematics();
  if(!robot_kinematics_->init(n))
  {
    ROS_ERROR("Init of robot_kinematics_ failed.!");
    return false;
  }

  // construct joint state interpreter
  joint_state_interpreter_ = new JointStateInterpreter(robot_kinematics_->getJointNames());

  // subscribe to joint state topic
  joint_state_subscriber_ = n.subscribe<sensor_msgs::JointState>("/joint_states", 2, &FeatureConstraintsController::joint_state_callback, this);

  // subscribe to feature constraint topic
  feature_constraints_subscriber_ = n.subscribe<constraint_msgs::ConstraintConfig>("constraint_config", 1, &FeatureConstraintsController::feature_constraints_callback, this);

  // subscribe to constraint command topic
  constraint_command_subscriber_ = n.subscribe<constraint_msgs::ConstraintCommand>("constraint_command", 1, &FeatureConstraintsController::constraint_command_callback, this);
  // advertise publisher
  qdot_publisher_ = n.advertise<std_msgs::Float64MultiArray>("qdot", 1);

  // resize internal robot representation
  unsigned int dof = robot_kinematics_->getNumberOfJoints();
  q_.resize(dof);
  qdot_.resize(dof);

  // resize msg that will be used to publish desired joint velocities
  qdot_msg_.data.resize(dof);

  // resize size of weights for joints in solver
  // identity means that all joints will be considered with equal weights
  Wq_ = Eigen::MatrixXd::Identity(dof, dof);

  // set ready-flag to signal that we are not ready, yet
  ready_ = false;

  return true;
}

void FeatureConstraintsController::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // parse message with joint state interpreter
  if(joint_state_interpreter_->parseJointState(msg, q_))
  {
    // assertion: parsing was successful
    update();
  }
}

void FeatureConstraintsController::feature_constraints_callback(const constraint_msgs::ConstraintConfig::ConstPtr& msg)
{
  // get new constraints into controller and re-prepare it
  // i.e. adjust size of internal variables
  fromMsg(*msg, feature_controller_.constraints);
  feature_controller_.prepare();
  // resize weights for constraints according to the number of weights
  // (note: will be used by solver)
  Wy_ = Eigen::MatrixXd::Zero(feature_controller_.constraints.size(), feature_controller_.constraints.size());
  // resize solver
  solver_.reinitialise(feature_controller_.constraints.size(), q_.rows());
  // set ready-flag to indicate that we are ready to solve constraints
  ready_ = true;
}

void FeatureConstraintsController::constraint_command_callback(const constraint_msgs::ConstraintCommand::ConstPtr& msg)
{
  assert(msg->pos_lo.size() == feature_controller_.command.pos_lo.rows());

  fromMsg(*msg, feature_controller_.command);
}


void FeatureConstraintsController::update()
{
  if(ready_)
  { 
    // assemble frame between tool and object
    KDL::Frame ff_kinematics;
    robot_kinematics_->getForwardKinematics(q_, ff_kinematics);
    robot_kinematics_->getJacobian(q_, jacobian_robot_);

    // TODO: check order
    KDL::Frame T_tool_in_object = T_object_in_world_.Inverse() * T_base_in_world_ * ff_kinematics * T_tool_in_ee_;
    
    // evaluate constraints
    feature_controller_.update(T_tool_in_object);

    // call solver with values from constraint controller and Jacobian
    A_ = feature_controller_.Ht.data.transpose() * jacobian_robot_.data;
    Wy_.diagonal() = feature_controller_.weights.data;
    solver_.solve(A_, feature_controller_.ydot.data, Wq_, Wy_, qdot_.data);

    // publish desired joint velocities    
    assert(qdot_.rows() == qdot_msg_.data.size());
    for(unsigned int i=0; i<qdot_.rows(); i++)
    {
      qdot_msg_.data[i] = qdot_(i);
    }
    qdot_publisher_.publish(qdot_msg_);
  }
}

// The entry to our application...
int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_constraints_demo");
  ros::NodeHandle n;
  FeatureConstraintsController myController;
  myController.init(n);
  ros::spin();
  return 0;
}
