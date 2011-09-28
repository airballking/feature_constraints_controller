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

#ifndef _ITASC_SUBTASK_HPP_
#define _ITASC_SUBTASK_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {

class SubTask: public RTT::TaskContext {
public:
	SubTask(std::string name, TaskState initial_state = Stopped) :
		TaskContext(name, initial_state) {
		this->ports()->addPort("T_o1_o2", T_o1_o2_port);
		this->ports()->addPort("Jq_qdot", Jq_qdot_port);
		this->ports()->addPort("JuXudot", JuXudot_port);

		this->ports()->addPort("ydot", ydot_port);
		this->ports()->addPort("Cf", Cf_port);
		this->ports()->addPort("Jf", Jf_port);
		this->ports()->addPort("Wy", Wy_port);

		this->ports()->addPort("initialized", initialized);

		this->provides()->addAttribute("nc", nc);

	}
	;

	virtual ~SubTask() {
	}
	;

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;

protected:
	RTT::InputPort<KDL::Frame> T_o1_o2_port;
	RTT::InputPort<KDL::Twist> Jq_qdot_port;

	//Output
	RTT::OutputPort<KDL::JntArray> ydot_port;
	RTT::OutputPort<KDL::Twist> JuXudot_port;
	RTT::OutputPort<Eigen::MatrixXd> Cf_port;
	RTT::OutputPort<KDL::Jacobian> Jf_port;
	RTT::OutputPort<Eigen::MatrixXd> Wy_port;

	unsigned int nc;

	RTT::OutputPort<bool> initialized;

};

}//end of namespace

#endif

