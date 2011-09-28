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

#ifndef _ITASC_SCENE_HPP
#define _ITASC_SCENE_HPP

#include <rtt/TaskContext.hpp>
#include <map>
#include "SubRobot.hpp"
#include "SubTask.hpp"
#include "Solver.hpp"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>

namespace iTaSC {

class Scene: public RTT::TaskContext {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Scene(const std::string& name);
	~Scene() {
	}
	;

	virtual bool configureHook();
	virtual bool startHook() {
		return true;
	}
	virtual void updateHook();
	virtual void stopHook() {
	}
	;
	virtual void cleanupHook();

	bool addRobot(const std::string& PeerName, const KDL::Frame& T_w_b);
	bool addSubTask(const std::string& PeerName, const std::string& Object1,
			const std::string& Object2);
	bool addConstraint(const std::string& CoordinateName);
	bool addSolver(const std::string& PeerName);

	bool connectTaskToRobot(const std::string& TaskName,
			const std::string& RobotName, bool one);
	void prepareSystem();

	RTT::OperationCaller<bool(void)> solve;

private:
	std::string iTaSC_configuration;
	RTT::OutputPort<Eigen::MatrixXd> A_port;
	RTT::OutputPort<Eigen::MatrixXd> Wy_port;
	RTT::OutputPort<Eigen::MatrixXd> Wq_port;
	RTT::OutputPort<Eigen::VectorXd> ydot_port;
	RTT::InputPort<Eigen::VectorXd> qdot_port;

	struct Robot {
	public:
		TaskContext* peer;
		RTT::OutputPort<KDL::JntArray> qdot_port;
		RTT::InputPort<KDL::Twist> JuXudot_port;
		RTT::InputPort<Eigen::MatrixXd> Wq_port;
		RTT::InputPort<KDL::Jacobian> Jq_port;
		RTT::InputPort<KDL::Frame> T_b_e_port;
		RTT::Attribute<Eigen::MatrixXd> Wq_global;
		RTT::Attribute<KDL::Frame> T_w_b;
		KDL::Jacobian Jq;
		KDL::JntArray qdot;
		Eigen::MatrixXd Wq_local;
		unsigned int start_index, nq;

		Robot(TaskContext* peer_in, const KDL::Frame& T_w_b_in,
				unsigned int nq_in, unsigned int start_index_in) :
			peer(peer_in), qdot_port(peer->getName() + "_qdot"), JuXudot_port(
					peer->getName() + "_JuXudot"), Wq_port(peer->getName()
					+ "_Wq_local"), Jq_port(peer->getName() + "_Jq"),
					T_b_e_port(peer->getName() + "_T_b_e"), Wq_global(
							peer->getName() + "_Wq_global"), T_w_b(
							peer->getName() + "_T_w_b", T_w_b_in), start_index(
							start_index_in), nq(nq_in) {
			if (nq > 0) {
				Jq.resize(nq);
				SetToZero(Jq);
				qdot.resize(nq);
				SetToZero(qdot);
				qdot_port.write(qdot);
				Wq_global.set(MatrixXd::Identity(nq, nq));
				Wq_local = MatrixXd::Identity(nq, nq);
			}
		}

	};
	struct Task {
	public:
		TaskContext* peer;
		RTT::InputPort<KDL::JntArray> ydot_port;
		RTT::InputPort<Eigen::MatrixXd> Cf_port;
		RTT::InputPort<KDL::Jacobian> Jf_port;
		RTT::InputPort<Eigen::MatrixXd> Wy_port;
		RTT::InputPort<KDL::Twist> JuXudot_port;
		RTT::OutputPort<KDL::Twist> Jq_qdot_port;
		RTT::OutputPort<KDL::Frame> T_o1_o2_port;
		RTT::Attribute<double> Wy_global;
		KDL::Jacobian Jf;
		unsigned int start_index, nc;
		Robot* robot1;
		Robot* robot2;
		KDL::JntArray y_dot_local;
		Eigen::MatrixXd Cf_local, Wy_local;

		Task(TaskContext* peer_in, Robot* robot1_in, Robot* robot2_in,
				unsigned int nc_in, unsigned int start_index_in) :
			peer(peer_in), ydot_port(peer->getName() + "_ydot"), Cf_port(
					peer->getName() + "_Cf"), Jf_port(peer->getName() + "_Jf"),
					Wy_port(peer->getName() + "_Wy_local"), JuXudot_port(
							peer->getName() + "_JuXudot"), Jq_qdot_port(
							peer->getName() + "_Jq_qdot"), T_o1_o2_port(
							peer->getName() + "_T_o1_o2"), Wy_global(
							peer->getName() + "_Wy_global", 0.0), Jf(6),
					start_index(start_index_in), nc(nc_in), robot1(robot1_in),
					robot2(robot2_in), y_dot_local(nc), Cf_local(
							Eigen::MatrixXd::Zero(nc, nc)), Wy_local(
							Eigen::MatrixXd::Zero(nc, nc)) {
			SetToZero(Jf);
		}

	};

	typedef std::map<std::string, Robot*> RobotMap;
	typedef std::map<std::string, Task*> TaskMap;

	RobotMap robots;
	TaskMap tasks;

	unsigned int nq_total, nc_total;

	Eigen::MatrixXd A_total, Wy_total, Wq_total, tmpJq, tmpCfJf, tmpJf, Uf, Vf;
	Eigen::VectorXd qdot_total, ydot_total, Sf, temp;

	// INGO: hacked in to get joint limit avoidance working
    KDL::Jacobian arm_Jq, arm_J;
	RTT::OutputPort<KDL::Jacobian> arm_J_port;
};
}
#endif
