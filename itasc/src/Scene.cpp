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

#include "Scene.hpp"
#include "Solver.hpp"
#include "eigen_toolkit.hpp"

#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <ocl/Component.hpp>

#include <kdl/frames_io.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>

ORO_CREATE_COMPONENT( iTaSC::Scene )
;

namespace iTaSC {

using namespace RTT;
using namespace std;
using namespace KDL;
using namespace Eigen;

Scene::Scene(const string& name) :
	TaskContext(name, PreOperational), iTaSC_configuration(
			"iTaSC_configuration.xml"), nq_total(0), nc_total(0) {
	this->properties()->addProperty("iTaSC_configuration", iTaSC_configuration).doc(
			"file containing the configuration of robots and virtual linkages");
	this->ports()->addPort("A", A_port);
	this->ports()->addPort("Wy", Wy_port);
	this->ports()->addPort("Wq", Wq_port);
	this->ports()->addPort("ydot", ydot_port);
	this->ports()->addPort("qdot", qdot_port);

    nc_task=6;
	this->properties()->addProperty("nc_task", nc_task).doc(
			"number of constraints for the feature task");

	// INGO: hacked in to get joint limit avoidance working
	this->ports()->addPort("arm_J", arm_J_port);
	arm_J = Jacobian(6);
	arm_Jq = Jacobian(7);

}

void Scene::cleanupHook() {
	//delete all robot structures and clear the map
	/*
	 for (RobotMap::iterator it = robots.begin(); it != robots.end(); ++it) {
	 delete (it->second);
	 }
	 */
	robots.clear();
	//delete all task structures and clear the map
	/*
	 for (TaskMap::iterator it = tasks.begin(); it != tasks.end(); ++it) {
	 delete (it->second);
	 }
	 */
	tasks.clear();
}

bool Scene::configureHook() {
	Logger::In in(this->getName());

	PropertyBag from_file;
	log(Info) << "Loading iTaSC configuration from " << iTaSC_configuration
			<< "." << endlog();
	marsh::PropertyDemarshaller demarshaller(iTaSC_configuration);

	try {
		//Read in configuratrion file
		if (demarshaller.deserialize(from_file)) {
			if (from_file.empty()) {
				log(Error) << "Configuration was empty !" << endlog();
				return false;
			}

			//First look for robot-structures
			for (PropertyBag::iterator it = from_file.begin(); it
					!= from_file.end(); it++) {
				Property<PropertyBag> robot = *it;
				//The item should be at least a structure, simples
				//are not recognised
				if (!robot.ready()) {
					log(Error) << "Property '" << (*it)->getName()
							<< "' should be a struct." << endlog();
					return false;
				}
				//Check if the type of the item is 'Robot'
				if (robot.rvalue().getType() == "Robot") {
					Property<string> name = robot.rvalue().getProperty(
							"PeerName");
					//Check if the item has a property PeerName
					if (!name.ready()) {
						log(Error) << "Property '" << (*it)->getName()
								<< "' does not contain a PeerName" << endlog();
						return false;
					}
					Property<PropertyBag> loc_bag = robot.rvalue().getProperty(
							"BaseLocation");
					Property<Frame> loc(loc_bag.getName(),
							loc_bag.getDescription());
					//Check if the item has a property BaseLocation
					if(!loc.getTypeInfo()->composeType(loc_bag.getDataSource(),loc.getDataSource()))
					{	log(Warning) << "Could not compose BaseLocation for '" << (*it)->getName()<<"', using Frame::Identity()"<< endlog();}
					log(Debug) << "Robot '" << name.rvalue()<< "' is located at " << loc.rvalue() << endlog();

					//Add the robot to the scene
					if (addRobot(name.rvalue(), loc.rvalue()))
						log(Info) << "SubRobot '" << name.rvalue()
								<< "' successfully added." << endlog();
					else {
						log(Error) << "Failed to add SubRobot'"
								<< name.rvalue() << "'." << endlog();
						return false;
					}
				}
			}//End of iterator loop to find robot structures

			//Look for the task-structures
			for (PropertyBag::iterator it = from_file.begin(); it
					!= from_file.end(); it++) {
				Property<PropertyBag> task = *it;
				// The property should at least be a struct before
				// we continue
				if (!task.ready()) {
					log(Error) << "Property '" << (*it)->getName()
							<< "' should be a struct." << endlog();
					return false;
				}
				//Check if the type of the bag is 'Task'
				if (task.rvalue().getType() == "Task") {
					//Check if the bag contains a property 'PeerName'
					Property<string> name = task.rvalue().getProperty(
							"PeerName");
					if (!name.ready()) {
						log(Error) << "Property<string> 'PeerName' not found"
								<< endlog();
						return false;
					}
					//Check if the task-bag contains a property 'Object1'
					Property<string> object1 = task.rvalue().getProperty(
							"Object1");
					if (!object1.ready()) {
						log(Info) << "Property<string> 'Object1' not found"
								<< endlog();
						return false;
					}
					//Check if the task-bag contains a property 'Object2'
					Property<string> object2 = task.rvalue().getProperty(
							"Object2");
					if (!object2.ready()) {
						log(Info) << "Property<string> 'Object2' not found"
								<< endlog();
						return false;
					}

					//Add the task to the Scene
					if (addSubTask(name.rvalue(), object1.rvalue(),
							object2.rvalue()))
						log(Info) << "SubTask '" << name.rvalue()
								<< "' successfully added." << endlog();
					else {
						log(Error) << "Failed to add SubTask '"
								<< name.rvalue() << "'." << endlog();
						return false;
					}

				}
			}//End of iteration over subtask structure

			//At last look for the one and only solver
			Property<PropertyBag> solver_bag = from_file.getProperty("Solver");
			//The item should be at least a structure, simples
			//are not recognised
			if (!solver_bag.ready()) {
				log(Error) << "Property '" << solver_bag
						<< "' should be a struct." << endlog();
				return false;
			}
			//Check if the type of the item is 'Solver'
			if (solver_bag.rvalue().getType() == "Solver") {
				Property<string> name = solver_bag.rvalue().getProperty(
						"PeerName");
				//Check if the item has a property PeerName
				if (!name.ready()) {
					log(Error) << "Property '" << solver_bag
							<< "' does not contain a PeerName" << endlog();
					return false;
				}
				//Add connect the solvers solve method to the scene
				if (addSolver(name.rvalue()))
					log(Info) << "Solver '" << name.rvalue()
							<< "' succesfully added." << endlog();
			}

		} else {
			log(Error) << "Some error occured while parsing "
					<< iTaSC_configuration << endlog();
			return false;
		}
	} catch (...) {
		log(Error) << "Uncaught exception while configuring '"
				<< this->getName() << "'." << endlog();
		return false;
	}

	//Prepare everything for the solver:
	//We have to create A, qdot and ydot
	this->prepareSystem();

	return true;
}

bool Scene::addRobot(const string& PeerName, const Frame& T_w_b) {
	log(Info) << "Adding SubRobot '" << PeerName << "', located at " << T_w_b
			<< endlog();

	//Check if the Scene is connected to this robot
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}
	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid SubRobot class
	SubRobot* robotp = dynamic_cast<SubRobot*> (peer);
	if (robotp == NULL) {
		//if(!peer.attributes()->hasAttribute("nq")||
		//   NULL==peer.ports()->getPort("qdot")||
		//   NULL==peer.ports()->getPort("Jq")||
		//   NULL==peer.ports()->getPort("T_w_ee"))
		log(Error) << "Component '" << PeerName
				<< "'is not a valid SubRobot component." << endlog();
		return false;
	}

	//Get the number of joints and configure the jointrange
	Attribute<unsigned int> nq = robotp->attributes()->getAttribute("nq");
	if (!nq.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have an necessary attribute nq" << endlog();
		return false;
	}
	//Insert new robot structure in the scene's robotmap

	Robot* robot = new Robot(robotp, T_w_b, nq.get(), nq_total);
	pair<RobotMap::iterator, bool> result = robots.insert(RobotMap::value_type(
			PeerName, robot));
	if (!result.second) {
		log(Error) << "SubRobot with name '" << PeerName << "' already exists."
				<< endlog();
		//If inserting the robot did not succeed, delete robot and return
		delete robot;
		return false;
	}

	//Connect the ports of the Scene to the ports of the robot
	bool connected = true;
	this->ports()->addPort(robot->qdot_port);
	connected &= robot->qdot_port.connectTo(robotp->ports()->getPort("qdot"));
	this->ports()->addPort(robot->Wq_port);
	connected &= robot->Wq_port.connectTo(robotp->ports()->getPort("Wq"));
	this->ports()->addPort(robot->Jq_port);
	connected &= robot->Jq_port.connectTo(robotp->ports()->getPort("Jq"));
	this->ports()->addPort(robot->T_b_e_port);
	connected &= robot->T_b_e_port.connectTo(robotp->ports()->getPort("T_b_e"));
	this->ports()->addPort(robot->JuXudot_port);
	connected &= robot->JuXudot_port.connectTo(robotp->ports()->getPort(
			"JuXudot"));

	if (!connected) {
		log(Error) << "Could not connect to SubRobot '" << PeerName
				<< "' ports." << endlog();
		//If connecting did not succeed, remove Robot from RobotMap
		delete robot;
		robots.erase(PeerName);
		return false;
	}

	nq_total += nq.get();

	return true;
}

bool Scene::addSubTask(const string& PeerName, const string& Object1,
		const string& Object2) {

	log(Info) << "Adding Task '" << PeerName << "' between '" << Object1
			<< "' and '" << Object2 << endlog();
	//Check if we are connected to this task
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}

	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid SubTask class
	SubTask* taskp = dynamic_cast<SubTask*> (peer);
	if (taskp == NULL) {
		log(Error) << "Component '" << PeerName
				<< "'is not a valid SubTask component." << endlog();
		return false;
	}

	//Check if Object1 exists
	RobotMap::const_iterator robot1 = robots.find(Object1);
	if (robot1 == robots.end()) {
		log(Error) << "Robot '" << Object1 << "' does not exist in the Scene."
				<< endlog();
		return false;
	}
	//Check if Object2 exists
	RobotMap::const_iterator robot2 = robots.find(Object2);
	if (robot2 == robots.end()) {
		log(Error) << "Robot '" << Object2 << "' does not exist in the Scene."
				<< endlog();
		return false;
	}

	//Get the number of joints and configure the jointrange
	Attribute<unsigned int> nc = taskp->provides()->getAttribute("nc");
	if (!nc.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have an necessary attribute nc" << endlog();
		return false;
	}

	//Add the task to the scene's taskmap
	Task* task = new Task(taskp, robot1->second, robot2->second, nc.get(),
			nc_total);
	pair<TaskMap::iterator, bool> result = tasks.insert(TaskMap::value_type(
			PeerName, task));
	if (!result.second) {
		log(Error) << "SubTask with name '" << PeerName << "' already exists."
				<< endlog();
		//delete task if insertion did not succeed
		delete task;
		return false;
	}

	bool connected = true;
	//Connect Task to Scene
	this->ports()->addPort(task->ydot_port);
	connected &= task->ydot_port.connectTo(taskp->ports()->getPort("ydot"));
	this->ports()->addPort(task->Jq_qdot_port);
	connected &= task->Jq_qdot_port.connectTo(
			taskp->ports()->getPort("Jq_qdot"));
	this->ports()->addPort(task->Cf_port);
	connected &= task->Cf_port.connectTo(taskp->ports()->getPort("Cf"));
	this->ports()->addPort(task->Jf_port);
	connected &= task->Jf_port.connectTo(taskp->ports()->getPort("Jf"));

	this->ports()->addPort(task->Wy_port);
	connected &= task->Wy_port.connectTo(taskp->ports()->getPort("Wy"));

	this->ports()->addPort(task->T_o1_o2_port);
	connected &= task->T_o1_o2_port.connectTo(
			taskp->ports()->getPort("T_o1_o2"));

	this->provides()->addAttribute(task->Wy_global);

	if (!connected) {
		log(Error) << "Could not connect to SubTask '" << PeerName
				<< "' ports." << endlog();
		//delete task and remove from map
		delete task;
		tasks.erase(PeerName);
		return false;
	}

	nc_total += nc.get();
	return true;
}

bool Scene::addSolver(const string& PeerName) {
	log(Info) << "Adding Solver '" << PeerName << "'." << endlog();

	//Check if the Scene is connected to this robot
	if (!this->hasPeer(PeerName)) {
		log(Error) << "Scene '" << this->getName()
				<< "' does not have a peer named '" << PeerName << "'."
				<< endlog();
		return false;
	}
	TaskContext* peer = this->getPeer(PeerName);
	//check if peer is a valid Solver class
	Solver* solverp = dynamic_cast<Solver*> (peer);
	if (solverp == NULL) {
		//if(!peer.provides()->hasAttribute("nq")||
		//   NULL==peer.ports()->getPort("qdot")||
		//   NULL==peer.ports()->getPort("Jq")||
		//   NULL==peer.ports()->getPort("T_w_ee"))
		log(Error) << "Component '" << PeerName
				<< "'is not a valid Solver component." << endlog();
		return false;
	}
	//Check if the peer has the solve method
	solve = peer->provides()->getOperation("solve");
	if (!solve.ready()) {
		log(Error) << "Component '" << PeerName
				<< "' does not have the solve method" << endlog();
		return false;
	}
	//Set the nc and nq attributes of the Solver
	Attribute<unsigned int> nc = peer->provides()->getAttribute("nc");
	nc.set(nc_total);
	Attribute<unsigned int> nq = peer->provides()->getAttribute("nq");
	nq.set(nq_total);

	//connect the ports of the scene with the ports of the solver
	A_port.connectTo(peer->ports()->getPort("A"));
	Wy_port.connectTo(peer->ports()->getPort("Wy"));
	Wq_port.connectTo(peer->ports()->getPort("Wq"));
	ydot_port.connectTo(peer->ports()->getPort("ydot"));
	qdot_port.connectTo(peer->ports()->getPort("qdot"));

	return true;
}

void Scene::prepareSystem() {
	log(Debug) << "preparing the Scene, resize all matrices/vectors"
			<< endlog();
	A_total = MatrixXd((int) nc_total, (int) nq_total);
	A_total.setZero();
	Wy_total = MatrixXd((int) nc_total, (int) nc_total);
	Wy_total.setZero();
	Wq_total = MatrixXd((int) nq_total, (int) nq_total);
	Wq_total.setZero();
	qdot_total = VectorXd((int) nq_total);
	qdot_total.setZero();
	ydot_total = VectorXd((int) nc_total);
	ydot_total.setZero();
	tmpJq = MatrixXd(6, (int) nq_total);
	tmpJq.setZero();
	tmpCfJf = MatrixXd((int) nc_total, 6);
	tmpCfJf.setZero();
	tmpJf = MatrixXd(nc_task, 6);
	tmpJf.setZero();
	Uf = MatrixXd(6, 6);
	Uf.setIdentity();
	Vf = MatrixXd(6, 6);
	Vf.setIdentity();
	Sf = VectorXd(6);
	Sf.setZero();
	temp = VectorXd(6);
	temp.setZero();
}


Matrix<double, 6, 6> inverse_twist_proj(KDL::Frame f)
{
  // (transposed) Rotation matrix of f
  Matrix3d Rt = Map<Matrix3d>(f.M.data);

  double x = f.p.x(), y = f.p.y(), z = f.p.z();

  // Skew symmetric matrix of p, [p]_x for expressing a cross product
  Matrix3d px;
  px << 0, -z,  y,
        z,  0, -x,
       -y,  x,  0;

  // the inverse twist projection matrix
  Matrix<double, 6, 6> Mi;
  Mi.block<3,3>(0,0) = Rt;
  Mi.block<3,3>(3,3) = Rt;
  Mi.block<3,3>(0,3) = -Rt*px;
  Mi.block<3,3>(3,0) = Matrix3d::Zero();

  return Mi;
}


void Scene::updateHook() {
	Logger::In in(this->getName());

	//Get all robot jacobians
	RobotMap::iterator robotp;
	for (robotp = robots.begin(); robotp != robots.end(); ++robotp) {
		Robot* robot = robotp->second;
		//log(Debug) << "For '" << robot->peer->getName()<< "', get jacobian and weighting matrix" << endlog();
		if (robot->nq != 0) {
			robot->Jq_port.read(robot->Jq);
			robot->Jq.changeRefFrame(robot->T_w_b.get());
			robot->Wq_port.read(robot->Wq_local);
			Wq_total.block(robot->start_index, robot->start_index, robot->nq,
					robot->nq)
					= (robot->Wq_global.get() * robot->Wq_local).lazy();
		}
		// INGO: hacked in to get joint limit avoidance working
		if(robot->peer->getName() == "LeftArm")
			arm_Jq = robot->Jq;
	}
	//Fill A, and ydot
	TaskMap::iterator taskp;
	for (taskp = tasks.begin(); taskp != tasks.end(); ++taskp) {
		Task* task = taskp->second;
		//Calculate robot poses:
		Frame T_b_e;
		task->robot1->T_b_e_port.read(T_b_e);
		Frame T_w_o1 = task->robot1->T_w_b.get() * T_b_e;
		task->robot2->T_b_e_port.read(T_b_e);
		Frame T_w_o2 = task->robot2->T_w_b.get() * T_b_e;
		Frame T_o1_o2 = T_w_o1.Inverse() * T_w_o2;
		task->T_o1_o2_port.write(T_o1_o2);

		//get ydot and store in ydot_total
		//log(Debug) << "For '" << task->peer->getName()<< "', get ydot and store in ydot_total" << endlog();
		task->ydot_port.read(task->y_dot_local);
		ydot_total.segment(task->start_index, task->nc)
				= task->y_dot_local.data;

		//store -Cf*Jf^-1*Jq in A
		//log(Debug) << "For '" << task->peer->getName()<< "', store -Cf*Jf^-1*Jq in A" << endlog();

		//Get Jf, transform it to the base and invert it.
		task->Jf_port.read(task->Jf);
		//task->Jf.changeRefFrame(T_w_o1);
        //INGO: changing the reference frame for the inverted (and transposed) jacobian
        task->Jf.data = (task->Jf.data.transpose()*inverse_twist_proj(T_w_o1)).transpose();


		// INGO: hacked in to get joint limit avoidance working
		if(task->peer->getName() == "Chef") {
			arm_J.data = task->Jf.data * arm_Jq.data;
			arm_J_port.write(arm_J);
		}

        // INGO: inversion takes place inside the Feature Task
        /*
		if (0 != svd_eigen_HH(task->Jf.data, Uf, Sf, Vf, temp)) {
			log(Error) << "Could not invert virtual linkage jacobian of task "
					<< task->peer->getName();
			this->fatal();
		}

		//log(Debug) << "Jf inverse of task '" << task->peer->getName()<< endlog();
		//log(Info) << "Jf: " << task->Jf.data << endlog();
		//log(Debug) << "Uf: " << Uf << endlog();
		//log(Debug) << "Sf: " << Sf << endlog();
		//log(Debug) << "Vf: " << Vf << endlog();

		for (unsigned int j = 0; j < 6; j++)
			if (Sf(j) > 0)
				Uf.col(j) *= 1 / Sf(j);
			else
				Uf.col(j).setZero();
		*/

		tmpJf = task->Jf.data.transpose(); //(Vf * Uf.transpose()).lazy();
		//log(Info)<<"Jf_inv"<<tmpJf<<endlog();

		task->Cf_port.read(task->Cf_local);
		tmpCfJf.block(task->start_index, 0, task->nc, 6) = -(task->Cf_local
				* tmpJf).lazy();
		//log(Debug) << "CfJf: " << tmpCfJf.block(task->start_index, 0, task->nc,6) << endlog();

		//get ydot and store in ydot_total
#ifndef NDEBUG
		//log(Debug) << "For '" << task->peer->getName()<< "', get ydot and store in ydot_total" << endlog();
#endif
		task->ydot_port.read(task->y_dot_local);
		ydot_total.segment(task->start_index, task->nc)
				= task->y_dot_local.data;

		//Add Cf*Jf_inv*JuXudot to ydot
		Eigen::Matrix<double, 6, 1> JuXudot;
		Twist JuXudot_kdl;
		//Part of robot 1
		task->robot1->JuXudot_port.read(JuXudot_kdl);
		//Transform to world reference frame
		JuXudot_kdl = task->robot1->T_w_b.get() * JuXudot_kdl;
		JuXudot << Map<Vector3d> (JuXudot_kdl.vel.data), Map<Vector3d> (
				JuXudot_kdl.rot.data);
		ydot_total.segment(task->start_index, task->nc) += (tmpCfJf.block(
				task->start_index, 0, task->nc, 6) * JuXudot).lazy();
		//Part of robot2
		task->robot2->JuXudot_port.read(JuXudot_kdl);
		//Transform to world reference frame
		JuXudot_kdl = task->robot2->T_w_b.get() * JuXudot_kdl;
		JuXudot << Map<Vector3d> (JuXudot_kdl.vel.data), Map<Vector3d> (
				JuXudot_kdl.rot.data);
		ydot_total.segment(task->start_index, task->nc) -= (tmpCfJf.block(
				task->start_index, 0, task->nc, 6) * JuXudot).lazy();

		if (task->robot1->nq != 0)
			A_total.block(task->start_index, task->robot1->start_index,
					task->nc, task->robot1->nq) = (tmpCfJf.block(
					task->start_index, 0, task->nc, 6) * task->robot1->Jq.data).lazy();
		if (task->robot2->nq != 0)
			A_total.block(task->start_index, task->robot2->start_index,
					task->nc, task->robot2->nq) = -(tmpCfJf.block(
					task->start_index, 0, task->nc, 6) * task->robot2->Jq.data).lazy();

		//store weight for this task in Wy_total
		//log(Debug) << "For '" << task->peer->getName()<< "', store weight for this task in Wy_total" << endlog();
		task->Wy_port.read(task->Wy_local);
		Wy_total.block(task->start_index, task->start_index, task->nc, task->nc)
				= (task->Wy_global.get() * task->Wy_local).lazy();
	}

	//Solve the System
	//log(Debug) << "Solve the total system" << endlog();
	A_port.write(A_total);
	Wy_port.write(Wy_total);
	ydot_port.write(ydot_total);
	Wq_port.write(Wq_total);

	//log(Debug) << "A: " << A_total << endlog();

	solve();

	qdot_port.read(qdot_total);

	//log(Debug) << "qdot" << qdot_total << endlog();

	//Send result to robots and tasks:
	for (robotp = robots.begin(); robotp != robots.end(); ++robotp) {
		Robot* robot = robotp->second;
		//log(Debug) << "For '" << robot->peer->getName()<< "', send resulting qdot to robot" << endlog();
		if (robot->nq != 0) {
			robot->qdot.data
					= qdot_total.segment(robot->start_index, robot->nq);
			robot->qdot_port.write(robot->qdot);
		}
	}
	for (taskp = tasks.begin(); taskp != tasks.end(); ++taskp) {
		Task* task = taskp->second;
		//log(Debug) << "For '" << task->peer->getName()<< "', send resulting Jq_qdot to robot" << endlog();
		//Expressed in o1
		Twist t1 = Twist::Zero();
		Twist t2 = Twist::Zero();
		if (task->robot1->nq != 0)
			MultiplyJacobian(task->robot1->Jq, task->robot1->qdot, t1);
		if (task->robot2->nq != 0)
			MultiplyJacobian(task->robot2->Jq, task->robot2->qdot, t2);
		task->Jq_qdot_port.write(t1 - t2);
	}

}

}
//End of namespace


