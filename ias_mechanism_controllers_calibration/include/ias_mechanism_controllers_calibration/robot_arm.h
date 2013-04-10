/**
 * Highly inspired by one of the tutorials on ros.org:
 * http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
 *
 */

// basic ros stuff
#include <ros/ros.h>
// action client basics
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages and goals used
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_mechanism_msgs/SwitchController.h>

// aux typedef to avoid writing this long type
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajectoryClient;

class RobotArm
{
private:
  // pointer to action client to standard JointTrajectoryAction
  TrajectoryClient* arm_action_client_;

  // little flag to remember successful initiation
  bool init_successful_;

  // goal send to the server
  pr2_controllers_msgs::JointTrajectoryGoal goal_;

  // our NodeHandle to access parameter server
  ros::NodeHandle n_;

public:
  // constructs action client
  RobotArm(ros::NodeHandle &n);  

  // tidies up pointer
  ~RobotArm();
 
  // reads all necessary parameters from parameter server and creates
  bool init();

  // sends goal to go to configuration given on parameter server
  bool gotoDesiredConfiguration();
  
  // tells controller manager to start controller
  bool startControllerPlugin();

  // tells controller manager to stop controller
  bool stopControllerPlugin();
};
