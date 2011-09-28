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
#ifndef _ITASC_SUBROBOT_HPP_
#define _ITASC_SUBROBOT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {
using namespace Eigen;


class SubRobot: public RTT::TaskContext {
public:
	SubRobot(const std::string& name, TaskState initial_state = Stopped) :
		TaskContext(name, initial_state) {
		this->ports()->addPort("qdot", qdot_port);//In: connected to solve

		this->ports()->addPort("JuXudot", JuXudot_port);//Out: connected to solver
		this->ports()->addPort("Wq", Wq_port);//Out: connected to solver
		this->ports()->addPort("Jq", Jq_port);//Out: connected to solver
		this->ports()->addPort("T_b_e", T_b_e_port);//Out: connected to subtask

		this->provides()->addAttribute("nq", nq);
	}
	;

	~SubRobot() {
	}
	;

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;

protected:
	//Input
	RTT::InputPort<KDL::JntArray> qdot_port;
	//Output
	RTT::OutputPort<KDL::Twist> JuXudot_port;
	RTT::OutputPort<MatrixXd> Wq_port;
	RTT::OutputPort<KDL::Jacobian> Jq_port;
	RTT::OutputPort<KDL::Frame> T_b_e_port;
	//Constant Attribute
        unsigned int nq;

};
}
#endif
