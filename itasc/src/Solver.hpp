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
#ifndef _ITASC_SOLVER_HPP_
#define _ITASC_SOLVER_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>

#include <Eigen/Core>

namespace iTaSC {

class Solver: public RTT::TaskContext {
protected:
	RTT::InputPort<Eigen::MatrixXd> A_port;
	RTT::InputPort<Eigen::MatrixXd> Wy_port;
	RTT::InputPort<Eigen::MatrixXd> Wq_port;
	RTT::InputPort<Eigen::VectorXd> ydot_port;
	RTT::OutputPort<Eigen::VectorXd> qdot_port;

	unsigned int nc;
	unsigned int nq;

	virtual bool solve()=0;

public:
	Solver(const std::string& name) :
		TaskContext(name, Stopped), A_port("A"), Wy_port("Wy"), Wq_port("Wq"),
				ydot_port("ydot"), qdot_port("qdot") {
		this->ports()->addPort(A_port);
		this->ports()->addPort(Wy_port);
		this->ports()->addPort(Wq_port);
		this->ports()->addPort(ydot_port);
		this->ports()->addPort(qdot_port);

		this->provides()->addAttribute("nc",nc);
		this->provides()->addAttribute("nq",nq);

		this->addOperation("solve", &Solver::solve, this, RTT::ClientThread).doc(
				"Solve the Scene's constraints");
	}
	;
	virtual ~Solver() {
	}
	;

	virtual bool configureHook()=0;
	virtual bool startHook()=0;
	virtual void updateHook()=0;
	virtual void stopHook()=0;
	virtual void cleanupHook()=0;
};
}
#endif
