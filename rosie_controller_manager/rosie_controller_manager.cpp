/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pr2_gazebo_plugins/gazebo_ros_controller_manager.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>

#include <vector>
#include <map>

#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_controller_manager/controller_manager.h"
#include "pr2_gazebo_plugins/SetModelsJointsStates.h"
#include "pr2_mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"

#include <ros/ros.h>

#include <urdf/model.h>
#include <map>

namespace gazebo {

class GazeboRosControllerManager : public gazebo::Controller
{
public:
  GazeboRosControllerManager(Entity *parent);
  virtual ~GazeboRosControllerManager();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;
  pr2_hardware_interface::HardwareInterface hw_;
  pr2_controller_manager::ControllerManager *cm_;

  /// @todo The fake state helps Gazebo run the transmissions backwards, so
  ///       that it can figure out what its joints should do based on the
  ///       actuator values.
  pr2_mechanism_model::RobotState *fake_state_;
  std::vector<gazebo::Joint*>  joints_;

  /// \brief Service Call Name
  private: ParamT<std::string> *setModelsJointsStatesServiceNameP;
  private: std::string setModelsJointsStatesServiceName;

  /*
   * \brief read pr2.xml for actuators, and pass tinyxml node to mechanism control node's initXml.
   */
  void ReadPr2Xml(XMLConfigNode *node);

  /*
   *  \brief pointer to ros node
   */
  ros::NodeHandle* rosnode_;

  /// \brief ros service
  private: ros::ServiceServer setModelsJointsStatesService;

  ///\brief ros service callback
  private: bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
                                      pr2_gazebo_plugins::SetModelsJointsStates::Response &res);

  ///\brief ros service callback
  /*
   *  \brief tmp vars for performance checking
   */
  double wall_start_, sim_start_;

  /// \brief set topic name of robot description parameter
  ParamT<std::string> *robotParamP;
  ParamT<std::string> *robotNamespaceP;
  std::string robotParam;
  std::string robotNamespace;

#ifdef USE_CBQ
  private: ros::CallbackQueue controller_manager_queue_;
  private: void ControllerManagerQueueThread();
  private: boost::thread* controller_manager_callback_queue_thread_;
#endif
  private: void ControllerManagerROSThread();
  private: boost::thread* ros_spinner_thread_;
};




GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_controller_manager", GazeboRosControllerManager);

GazeboRosControllerManager::GazeboRosControllerManager(Entity *parent)
  : Controller(parent), hw_(), fake_state_(NULL)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboRosControllerManager controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotParamP = new ParamT<std::string>("robotParam", "robot_description", 0);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->setModelsJointsStatesServiceNameP = new ParamT<std::string>("setModelsJointsStatesServiceName","set_models_joints_states", 0);
  Param::End();

  if (getenv("CHECK_SPEEDUP"))
  {
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
    wall_start_ = Simulator::Instance()->GetWallTime().Double();
    sim_start_  = Simulator::Instance()->GetSimTime().Double();
#else
    wall_start_ = Simulator::Instance()->GetWallTime();
    sim_start_  = Simulator::Instance()->GetSimTime();
#endif
  }

}

/// \brief callback for setting models joints states
bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
                           pr2_gazebo_plugins::SetModelsJointsStates::Response &res)
{

  return true;
}


GazeboRosControllerManager::~GazeboRosControllerManager()
{
  delete this->robotParamP;
  delete this->robotNamespaceP;
  delete this->cm_; 
  delete this->rosnode_;
#ifdef USE_CBQ
  delete this->controller_manager_callback_queue_thread_;
#endif
  delete this->ros_spinner_thread_;
}

void GazeboRosControllerManager::LoadChild(XMLConfigNode *node)
{
  // get parameter name
  this->robotParamP->Load(node);
  this->robotParam = this->robotParamP->GetValue();
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);  // namespace comes from gazebo_model spawner
  ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s",this->robotNamespace.c_str());

  this->cm_ = new pr2_controller_manager::ControllerManager(&hw_,*this->rosnode_);

  // read pr2 urdf
  // setup actuators, then setup mechanism control node
  ReadPr2Xml(node);

  // Initializes the fake state (for running the transmissions backwards).
  this->fake_state_ = new pr2_mechanism_model::RobotState(&this->cm_->model_);

  // The gazebo joints and mechanism joints should match up.
  for (unsigned int i = 0; i < this->cm_->state_->joint_states_.size(); ++i)
  {
    std::string joint_name = this->cm_->state_->joint_states_[i].joint_->name;

    // fill in gazebo joints pointer
    gazebo::Joint *joint = this->parent_model_->GetJoint(joint_name);
    if (joint)
    {
      this->joints_.push_back(joint);
    }
    else
    {
      //ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
      this->joints_.push_back(NULL);
    }

  }

#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime().Double());
#else
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
#endif
}

void GazeboRosControllerManager::InitChild()
{
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime().Double());
#else
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
#endif
#ifdef USE_CBQ
  // start custom queue for controller manager
  this->controller_manager_callback_queue_thread_ = new boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerQueueThread,this ) );
#endif

  // pr2_etherCAT calls ros::spin(), we'll thread out one spinner here to mimic that
  this->ros_spinner_thread_ = new boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerROSThread,this ) );

}

void GazeboRosControllerManager::UpdateChild()
{
  if (getenv("CHECK_SPEEDUP"))
  {
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
    double wall_elapsed = Simulator::Instance()->GetWallTime().Double() - wall_start_;
    double sim_elapsed  = Simulator::Instance()->GetSimTime().Double()  - sim_start_;
#else
    double wall_elapsed = Simulator::Instance()->GetWallTime() - wall_start_;
    double sim_elapsed  = Simulator::Instance()->GetSimTime()  - sim_start_;
#endif
    std::cout << " real time: " <<  wall_elapsed
              << "  sim time: " <<  sim_elapsed
              << "  speed up: " <<  sim_elapsed / wall_elapsed
              << std::endl;
  }
  assert(this->joints_.size() == this->fake_state_->joint_states_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->joints_[i])
      continue;

    double damping_coef;
    if (this->cm_->state_->joint_states_[i].joint_->dynamics)
      damping_coef = this->cm_->state_->joint_states_[i].joint_->dynamics->damping;
    else
      damping_coef = 0;

    this->fake_state_->joint_states_[i].measured_effort_ = this->fake_state_->joint_states_[i].commanded_effort_;

    switch(this->joints_[i]->GetType())
    {
    case Joint::HINGE: {
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
      Joint *hj = this->joints_[i];
      this->fake_state_->joint_states_[i].position_ = this->fake_state_->joint_states_[i].position_ +
                    angles::shortest_angular_distance(this->fake_state_->joint_states_[i].position_,hj->GetAngle(0).GetAsRadian());
      this->fake_state_->joint_states_[i].velocity_ = hj->GetVelocity(0);
#else
      HingeJoint *hj = (HingeJoint*)this->joints_[i];
      this->fake_state_->joint_states_[i].position_ = this->fake_state_->joint_states_[i].position_ +
                    angles::shortest_angular_distance(this->fake_state_->joint_states_[i].position_,hj->GetAngle());
      this->fake_state_->joint_states_[i].velocity_ = hj->GetAngleRate();
#endif
      break;
    }
    case Joint::SLIDER: {
      static double torso_hack_damping_threshold = 1000.0; /// FIXME: if damping is greater than this value, do some unconventional smoothing to prevent instability due to safety controller
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
      Joint *sj = this->joints_[i];
      if (damping_coef > torso_hack_damping_threshold)
      {
        this->fake_state_->joint_states_[i].position_ *= (1.0 - torso_hack_damping_threshold / damping_coef);
        this->fake_state_->joint_states_[i].position_ += (torso_hack_damping_threshold/damping_coef)*sj->GetAngle(0).GetAsRadian();
        this->fake_state_->joint_states_[i].velocity_ *= (1.0 - torso_hack_damping_threshold / damping_coef);
        this->fake_state_->joint_states_[i].velocity_ += (torso_hack_damping_threshold/damping_coef)*sj->GetVelocity(0);
      }
      else
      {
        this->fake_state_->joint_states_[i].position_ = sj->GetAngle(0).GetAsRadian();
        this->fake_state_->joint_states_[i].velocity_ = sj->GetVelocity(0);
      }
      break;
#else
      SliderJoint *sj = (SliderJoint*)this->joints_[i];
      if (damping_coef > torso_hack_damping_threshold)
      {
        this->fake_state_->joint_states_[i].position_ *= (1.0 - torso_hack_damping_threshold / damping_coef);
        this->fake_state_->joint_states_[i].position_ += (torso_hack_damping_threshold/damping_coef)*sj->GetPosition();
        this->fake_state_->joint_states_[i].velocity_ *= (1.0 - torso_hack_damping_threshold / damping_coef);
        this->fake_state_->joint_states_[i].velocity_ += (torso_hack_damping_threshold/damping_coef)*sj->GetPositionRate();
      }
      else
      {
        this->fake_state_->joint_states_[i].position_ = sj->GetPosition();
        this->fake_state_->joint_states_[i].velocity_ = sj->GetPositionRate();
      }
      break;
#endif
    }
    default:
      abort();
    }
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  this->fake_state_->propagateJointPositionToActuatorPosition();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime().Double());
#else
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
#endif
  try
  {
    this->cm_->update();
  }
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
      ROS_WARN("pid controller reports divide by zero error");
    else
      ROS_WARN("unknown const char* exception: %s", c);
  }

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  this->fake_state_->propagateActuatorEffortToJointEffort();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->joints_[i])
      continue;

    double effort = this->fake_state_->joint_states_[i].commanded_effort_;

    double damping_coef;
    if (this->cm_->state_->joint_states_[i].joint_->dynamics)
      damping_coef = this->cm_->state_->joint_states_[i].joint_->dynamics->damping;
    else
      damping_coef = 0;

    switch (this->joints_[i]->GetType())
    {
    case Joint::HINGE: {
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
      Joint *hj = this->joints_[i];
      double current_velocity = hj->GetVelocity(0);
      double damping_force = damping_coef * current_velocity;
      double effort_command = effort - damping_force;
      hj->SetForce(0,effort_command);
#else
      HingeJoint *hj = (HingeJoint*)this->joints_[i];
      double current_velocity = hj->GetAngleRate();
      double damping_force = damping_coef * current_velocity;
      double effort_command = effort - damping_force;
      hj->SetTorque(effort_command);
#endif
      break;
    }
    case Joint::SLIDER: {
#if GAZEBO_MAJOR_VERSION == 0 && GAZEBO_MINOR_VERSION >= 10
      Joint *sj = this->joints_[i];
      double current_velocity = sj->GetVelocity(0);
      double damping_force = damping_coef * current_velocity;
      double effort_command = effort-damping_force;
      (this->joints_[i])->SetForce(0,effort_command);
#else
      SliderJoint *sj = (SliderJoint*)this->joints_[i];
      double current_velocity = sj->GetPositionRate();
      double damping_force = damping_coef * current_velocity;
      double effort_command = effort-damping_force;
      sj->SetSliderForce(effort_command);
#endif
      break;
    }
    default:
      abort();
    }
  }
}

void GazeboRosControllerManager::FiniChild()
{
  ROS_DEBUG("Calling FiniChild in GazeboRosControllerManager");

  //pr2_hardware_interface::ActuatorMap::const_iterator it;
  //for (it = hw_.actuators_.begin(); it != hw_.actuators_.end(); ++it)
  //  delete it->second; // why is this causing double free corrpution?
  this->cm_->~ControllerManager();
  //delete this->fake_state_;  // why is this causing double free corrpution?
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->controller_manager_queue_.clear();
  this->controller_manager_queue_.disable();
  this->controller_manager_callback_queue_thread_->join();
#endif
  this->ros_spinner_thread_->join();
}

void GazeboRosControllerManager::ReadPr2Xml(XMLConfigNode *node)
{

  std::string urdf_param_name;
  std::string urdf_string;
  // search and wait for robot_description on param server
  while(urdf_string.empty())
  {
    ROS_WARN("gazebo controller manager plugin is waiting for urdf: %s on the param server.", this->robotParam.c_str());
    if (this->rosnode_->searchParam(this->robotParam,urdf_param_name))
    {
      this->rosnode_->getParam(urdf_param_name,urdf_string);
      ROS_DEBUG("found upstream\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
    }
    else
    {
      this->rosnode_->getParam(this->robotParam,urdf_string);
      ROS_DEBUG("found in node namespace\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
    }
    usleep(100000);
  }



  ROS_INFO("gazebo controller manager got pr2.xml from param server, parsing it...");
  //std::cout << urdf_string << std::endl;

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()))
  {
    ROS_ERROR("Could not load the gazebo controller manager plugin's configuration file: %s\n",
            urdf_string.c_str());
    abort();
  }
  //std::cout << *(doc.RootElement()) << std::endl;

  // Pulls out the list of actuators used in the robot configuration.
  struct GetActuators : public TiXmlVisitor
  {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
      if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      return true;
    }
  } get_actuators;
  doc.RootElement()->Accept(&get_actuators);

  // Places the found actuators into the hardware interface.
  std::set<std::string>::iterator it;
  for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
  {
    //std::cout << " adding actuator " << (*it) << std::endl;
    pr2_hardware_interface::Actuator* pr2_actuator = new pr2_hardware_interface::Actuator(*it);
    pr2_actuator->state_.is_enabled_ = true;
    this->hw_.addActuator(pr2_actuator);
  }

  // Setup mechanism control node
  this->cm_->initXml(doc.RootElement());

  for (unsigned int i = 0; i < this->cm_->state_->joint_states_.size(); ++i)
    this->cm_->state_->joint_states_[i].calibrated_ = true;
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// custom callback queue
void GazeboRosControllerManager::ControllerManagerQueueThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->controller_manager_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

void GazeboRosControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  //ros::Rate rate(1000);

  while (this->rosnode_->ok())
  {
    //rate.sleep(); // using rosrate gets stuck on model delete
    usleep(1000);
    ros::spinOnce();
  }
}
} // namespace gazebo
