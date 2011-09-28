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
class ROSRobot: public SubRobot {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ROSRobot(const std::string& name = "ROSRobot");
	~ROSRobot();

	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:
    bool findState();
	bool readState();

	//TODO: partially obsolete
	std::vector<double> q_std;
	std::vector<int> q_index;
	std::vector<int> jnt_index;
	unsigned int q_index_max, jnt_index_first;
	sensor_msgs::JointState state;
	std_msgs::Float64MultiArray qdot_msg;
	std_msgs::Float64MultiArray q_msg;
    int first_state_entry, first_joint_segment;

	RTT::OutputPort<std_msgs::Float64MultiArray> qdot_to_robot;
	RTT::InputPort<sensor_msgs::JointState> state_from_robot;
	RTT::OutputPort<KDL::Jacobian> jac_kdl;
	RTT::OutputPort<KDL::JntArray> q_port;
	RTT::OutputPort<std_msgs::Float64MultiArray> q_debug_port;
	RTT::InputPort<KDL::Jacobian> jac_from_robot;
	RTT::InputPort<geometry_msgs::Pose> tool_frame;

	KDL::Frame tool;
	std::vector<double> Wq_vec;
	std::vector<double,Eigen::aligned_allocator<double> > Wq_vec_aligned;

	KDL::JntArray q_kdl;
	KDL::Chain chain;
	KDL::JntArray qdot;
	KDL::ChainFkSolverPos_recursive* fk;
	KDL::ChainJntToJacSolver* jnt2jac;
	KDL::Jacobian Jq_kdl, Jq_kdl_b_ee;
	KDL::Frame Tb_ee;
	Eigen::Matrix<double, 7, 7> Wq;

    std::string base_name, ee_name;
};

}
#endif
