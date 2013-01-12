#include <ias_mechanism_controllers_calibration/robot_arm.h>

#include <string>

RobotArm::RobotArm(ros::NodeHandle &n) : init_successful_(false), n_(n)
{
  arm_action_client_ = new TrajectoryClient("arm_trajectory_action", true);

  while(!arm_action_client_->waitForServer())
    ROS_INFO("Waiting for the joint_trajectory_action server.");
}

RobotArm::~RobotArm()
{
  if(arm_action_client_)
    delete arm_action_client_;
}

bool RobotArm::init()
{
  // get action server namespace
  std::string as_ns;
  if(!n_.getParam("arm_action_server_namespace", as_ns))
  {
    ROS_ERROR("Parameter 'arm_action_server_namespace' not given in namespace: '%s'", n_.getNamespace().c_str());
    init_successful_ = false;
    return false;
  }
  ROS_INFO("Namespace of arm action server: %s", as_ns.c_str());

  // get joint names controllerd by action server
  using namespace XmlRpc;

  XmlRpc::XmlRpcValue joint_names;
  if(!n_.getParam(as_ns + std::string("/joints"), joint_names))
  {
    ROS_ERROR("Parameter '/joints' not given in namespace: '%s'.", as_ns.c_str());
    init_successful_ = false;
    return false;
  }

  if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Extracted joint names are not stored in an array!");
    init_successful_ = false;
    return false;
  }

  // print joint names and write them into the goal
  ROS_INFO("Joints that will be moved in the first motion:");
  for(int i=0; i<joint_names.size(); i++)
  {
    XmlRpcValue &name_value = joint_names[i];
    if(name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Some joint name was not of type string.");
      init_successful_ = false;
      return false;
    }
    ROS_INFO("'%s'", ((std::string)name_value).c_str());
    goal_.trajectory.joint_names.push_back((std::string)name_value);
  }

  // get joint target configuration from parameter server and write goal
  // (assume only one goal configuration)
  goal_.trajectory.points.resize(1);
  goal_.trajectory.points[0].positions.resize(goal_.trajectory.joint_names.size());
  goal_.trajectory.points[0].velocities.resize(goal_.trajectory.joint_names.size());

  for(unsigned int i=0; i<goal_.trajectory.joint_names.size(); i++)
  {
    // construct parameter name for goal positions and velocities of joints
    std::string q_goal_name = std::string("/test_goals") + std::string("/") + goal_.trajectory.joint_names[i] + "/q";
    std::string qdot_goal_name = std::string("/test_goals") + std::string("/") + goal_.trajectory.joint_names[i] + "/qdot";

    // read q_goal from parameter server and write it into goal message
    if(!n_.getParam(as_ns + q_goal_name, goal_.trajectory.points[0].positions[i]))
    {
      ROS_ERROR("Parameter '%s' not given in namespace '%s'",q_goal_name.c_str(), as_ns.c_str());
      return false;
    }

    // read qdot_goal from parameter server and write it into goal message
    if(!n_.getParam(as_ns + qdot_goal_name, goal_.trajectory.points[0].velocities[i]))
    {
      ROS_WARN("Parameter '%s' not given in namespace '%s'. Setting to 0.",qdot_goal_name.c_str(), as_ns.c_str());

      goal_.trajectory.points[0].velocities[i] = 0.0;
    }
  }

  // get desired execution time from parameter server
  double executionTime;
  if(!n_.getParam(as_ns + std::string("/test_execution_time"), executionTime))
  {
    ROS_ERROR("Parameter '/test_execution_time' not found in namespace '%s'.", as_ns.c_str());
    return false;
  }
  goal_.trajectory.points[0].time_from_start = ros::Duration(executionTime);

  // set flag and return successful
  init_successful_ = true;
  return true;
}

bool RobotArm::gotoDesiredConfiguration()
{
  if(!init_successful_)
  {
    ROS_ERROR("Trying to send action goal even though init wasn't successful.");
    return false;
  }

  // TODO: finish this by setting start and execution time
  //       check return status
  goal_.trajectory.header.stamp =ros::Time::now() + ros::Duration(1.0);
  arm_action_client_->sendGoal(goal_);

  // wait for action to return
  bool finished_before_timeout = arm_action_client_->waitForResult(goal_.trajectory.points[0].time_from_start + ros::Duration(2.0));

  if(finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = arm_action_client_->getState();
    ROS_INFO("Action finished with state: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action timedout");
    return false;
  }

  // return reporting success
  return true;
}

bool RobotArm::startControllerPlugin()
{
  // instruct pr2 controller manager to start standard controllers
  pr2_mechanism_msgs::SwitchController srv;
  srv.request.start_controllers.push_back("l_arm_controller");
  srv.request.start_controllers.push_back("r_arm_controller");
  srv.request.strictness = srv.request.STRICT;
  if(ros::service::call("/pr2_controller_manager/switch_controller", srv))
  {
    if(!srv.response.ok)
      ROS_ERROR("PR2 controller manager reported error when starting controllers.");
    return srv.response.ok;
  }
  else
  {
    ROS_ERROR("Failed to call PR2 controller manager.");
    return false;
  }
}

bool RobotArm::stopControllerPlugin()
{
  // instruct controller manager to stop standard controllers
  pr2_mechanism_msgs::SwitchController srv;
  srv.request.stop_controllers.push_back("l_arm_controller");
  srv.request.stop_controllers.push_back("r_arm_controller");
  srv.request.strictness = srv.request.STRICT;
  if(ros::service::call("/pr2_controller_manager/switch_controller", srv))
  {
    if(!srv.response.ok)
      ROS_ERROR("PR2 controller manager reported error when starting controllers.");
    return srv.response.ok;
  }
  else
  {
    ROS_ERROR("Failed to call PR2 controller manager.");
    return false;
  }
}
