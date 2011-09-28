/*
 * (C) 2010 Ruben Smits, ruben.smits@mech.kuleuven.be, Department of Mechanical
 Engineering, Katholieke Universiteit Leuven, Belgium.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 Alternatively, the contents of this file may be used under the terms of
 either of the New BSD License
 */
#ifndef _ITASC_ROS_ROBOT_HPP_
#define _ITASC_ROS_ROBOT_HPP_

#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

#include "SubRobot.hpp"


namespace iTaSC {
class ROSLimitAvoider: public RTT::TaskContext {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ROSLimitAvoider(const std::string& name = "ROSLimitAvoider");
	~ROSLimitAvoider();

	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:
	bool readLimits();
	void avoidLimits();
	void projectAction();
	void rescaleAvoidance();

	std::vector<double> lim_min;
	std::vector<double> lim_max;

    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qdot_task;
    KDL::JntArray qdot_out;
	KDL::Jacobian jac;

	std_msgs::Float64MultiArray qdot_ros;
	std_msgs::Float64MultiArray qdot_mixed_ros;
	std_msgs::Float64MultiArray qdot_raw_ros;

	RTT::InputPort<KDL::JntArray> q_port;
	RTT::InputPort<KDL::JntArray> qdot_task_port;
	RTT::InputPort<KDL::Jacobian> jac_port;
	RTT::OutputPort<KDL::JntArray> qdot_port;
	RTT::OutputPort<std_msgs::Float64MultiArray> qdot_ros_port;
	RTT::OutputPort<std_msgs::Float64MultiArray> qdot_mixed_ros_port;
	RTT::OutputPort<std_msgs::Float64MultiArray> qdot_raw_ros_port;

	int nq;
	double d, h;
	double K_task, K_avoid;
	std::string base_name, ee_name;
};

}
#endif
