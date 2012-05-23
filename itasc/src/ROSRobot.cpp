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

#include "ROSRobot.hpp"

#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <ocl/Component.hpp>

typedef unsigned int uint;

ORO_CREATE_COMPONENT( iTaSC::ROSRobot )
;

using namespace std;
using namespace KDL;
using namespace RTT;
using namespace OCL;
using namespace Eigen;

namespace iTaSC {
ROSRobot::ROSRobot(const std::string& name) :
	SubRobot(name, PreOperational), q_index_max(-1), jnt_index_first(-1), first_state_entry(-1), tool(
			Frame::Identity()) {

	this->ports()->addPort("qdot_cmd", qdot_to_robot);

	this->ports()->addPort("jointstate", state_from_robot);
	this->ports()->addPort("jac", jac_from_robot);
	this->ports()->addPort("jac_ee", jac_kdl);
	this->ports()->addPort("tool", tool_frame);
	this->ports()->addPort("q", q_port);
	this->ports()->addPort("q_debug", q_debug_port);

	ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
	c.transport = 3;

	c.name_id = string("/joint_states");
	state_from_robot.createStream(c);

	c.name_id = string("/tool_pose");
	tool_frame.createStream(c);

	c.name_id = string("/q_dot_cmd");
	qdot_to_robot.createStream(c);

	c.name_id = string("/q_debug");
	q_debug_port.createStream(c);

	this->properties()->addProperty("W", Wq_vec).doc(
			"diagonal of joint weighting matrix (do not fill with negative values)");
	this->properties()->addProperty("base_name", base_name).doc(
			"name of the robots base link in the URDF");
	this->properties()->addProperty("ee_name", ee_name).doc(
			"name of the robots end effecot link in the URDF");
}

ROSRobot::~ROSRobot() {
}

bool ROSRobot::configureHook()
{
	Tree tree;
	if (!kdl_parser::treeFromParam("robot_description", tree))
	{
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	tree.getChain(base_name, ee_name, chain);

	for(uint i=0; i < chain.getNrOfSegments(); ++i)
	{
		Joint j = chain.getSegment(i).getJoint();
		if(j.getType() != Joint::None)
		{
			printf("joint %d: %s\n", i, j.getName().c_str());
			//log(Error) << "joint " << j << ": " << j.getName() << endlog();
		}
	}

	fk = new ChainFkSolverPos_recursive(chain);
	jnt2jac = new ChainJntToJacSolver(chain);

	nq = chain.getNrOfJoints();

	q_std = std::vector<double>(nq, 0.0);
	qdot_msg.data.resize(nq);
	q_msg.data.resize(nq);

	Wq_vec = std::vector<double>(nq, 1.0);
	Wq_vec_aligned = std::vector<double, Eigen::aligned_allocator<double> >(nq, 1.0);
	q_kdl = JntArray(nq);
	qdot = JntArray(nq);
	Jq_kdl = Jacobian(nq);
	Jq_kdl_b_ee = Jacobian(nq);

	Wq.setZero();
	Wq_vec_aligned.assign(Wq_vec.begin(),Wq_vec.end());
	Wq.diagonal() = VectorXd::Map(&Wq_vec_aligned[0], Wq_vec_aligned.size());
	Wq_port.write(Wq);

	Jq_port.write(Jq_kdl);
	jac_kdl.write(Jq_kdl_b_ee);

	return true;
}
bool ROSRobot::startHook()
{
	return true;
}

bool ROSRobot::readState()
{
	state_from_robot.read(state);

    // initialize "parser" if necessary
	if(q_index.size() == 0)
        if(!findState())
            return false;

	// check if the message is big enough
	if(state.name.size() < (uint) q_index_max + 1)
		return false;

    // check if the first name matches
	if(state.name[q_index[0]] !=
      chain.getSegment(jnt_index_first).getJoint().getName())
		return false;

    // names match, copy position
	for(uint i=0; i < q_index.size(); ++i)
		q_kdl(i) = state.position[q_index[i]];

    return true;
}

bool ROSRobot::findState()
{
    for(uint i=0; i < chain.getNrOfSegments(); ++i)
	{
        Joint jnt = chain.getSegment(i).getJoint();
        if(jnt.getType() != Joint::None)
		{
			// remember the first joint segment
			if(q_index.size() == 0)
				jnt_index_first = i;

			// search for the joint name
			for(uint j=0; j < state.name.size(); ++j)
			{
				if(state.name[j] == jnt.getName())
				{
					q_index.push_back(j);
					jnt_index.push_back(i);
						q_index_max = (j > q_index_max) ? j : q_index_max;
					break;
				}
			}
		}
	}
	return (q_index.size() != 0);
}

void ROSRobot::updateHook()
{
	//Get the latest joint positions and use them to update the
	//robots pose and jacobian
	readState();
	q_port.write(q_kdl);

	for (unsigned int i = 0; i < q_kdl.rows(); i++)
		q_msg.data[i] = q_kdl(i);
	q_debug_port.write(q_msg);

    geometry_msgs::Pose tool_msg;
	if(NewData == tool_frame.read(tool_msg))
        tf::PoseMsgToKDL(tool_msg, tool);

	fk->JntToCart(q_kdl, Tb_ee);

	// Jacobian with ref frame base, ref point ee
	jnt2jac->JntToJac(q_kdl, Jq_kdl_b_ee);
    // change ref point to tool
	Jq_kdl_b_ee.changeRefPoint(Tb_ee*tool.p - Tb_ee.p);

	jnt2jac->JntToJac(q_kdl, Jq_kdl);
	// transform Jacobian -> ref frame base, ref point base
	Jq_kdl.changeRefPoint(-Tb_ee.p);

    // change frame to tool
	Tb_ee = Tb_ee * tool;

	Wq_vec_aligned.assign(Wq_vec.begin(),Wq_vec.end());
	Wq.diagonal() = VectorXd::Map(&Wq_vec_aligned[0], Wq_vec_aligned.size());
	Wq_port.write(Wq);

	Jq_port.write(Jq_kdl);
	jac_kdl.write(Jq_kdl_b_ee);
	T_b_e_port.write(Tb_ee);

	//Get the desired joint velocities and send them to the robot:
	qdot_port.read(qdot);
	for (unsigned int i = 0; i < qdot.rows(); i++)
		qdot_msg.data[i] = qdot(i);

	qdot_to_robot.write(qdot_msg);
}

void ROSRobot::stopHook()
{

}

void ROSRobot::cleanupHook()
{
	delete fk;
	delete jnt2jac;
}

}

