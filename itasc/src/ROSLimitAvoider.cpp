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

#include "ROSLimitAvoider.hpp"

#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <ocl/Component.hpp>

typedef unsigned int uint;

ORO_CREATE_COMPONENT( iTaSC::ROSLimitAvoider )
;

using namespace std;
using namespace KDL;
using namespace RTT;
using namespace OCL;
using namespace Eigen;

namespace iTaSC {
ROSLimitAvoider::ROSLimitAvoider(const std::string& name) :
	TaskContext(name, PreOperational), nq(0), d(0.7), h(0.8),
    K_task(1.0), K_avoid(1.0), ros_prefix("/right") {

	this->ports()->addPort("q", q_port);
	this->ports()->addPort("jac", jac_port);
	this->ports()->addPort("qdot", qdot_port);

	this->ports()->addPort("qdot_task", qdot_task_port);
	this->ports()->addPort("qdot_ros", qdot_ros_port);
	this->ports()->addPort("qdot_mixed_ros", qdot_mixed_ros_port);
	this->ports()->addPort("qdot_raw_ros", qdot_raw_ros_port);



	this->properties()->addProperty("d", d).doc(
			"distance from joint limit where repelling starts");
	this->properties()->addProperty("h", h).doc(
			"maximum repelling force");

	this->properties()->addProperty("base_name", base_name).doc(
	        "name of the robots base link in the URDF");
	this->properties()->addProperty("ee_name", ee_name).doc(
	        "name of the robots end effecot link in the URDF");


	this->properties()->addProperty("K_task", K_task).doc(
			"mixing factor for task velocities");
	this->properties()->addProperty("K_avoid", K_avoid).doc(
			"mixing factor for limit avoidance velocities");
	this->properties()->addProperty("ros_prefix", ros_prefix).doc(
	        "prefix for the ROS topic names");

	d=0.7;
    h=0.8;
}


ROSLimitAvoider::~ROSLimitAvoider()
{
}


bool ROSLimitAvoider::configureHook()
{
	if(!readLimits())
		return false;
	nq = lim_min.size();


	ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
	c.transport = 3;

	c.name_id = string(ros_prefix+"/qdot_ros");
	qdot_ros_port.createStream(c);

	c.name_id = string(ros_prefix+"/qdot_mixed_ros");
	qdot_mixed_ros_port.createStream(c);

	c.name_id = string(ros_prefix+"/qdot_raw_ros");
	qdot_raw_ros_port.createStream(c);


	q = JntArray(nq);
	qdot = JntArray(nq);
	qdot_task = JntArray(nq);
	qdot_out = JntArray(nq);
	jac = Jacobian(nq);

	qdot_ros.data.resize(nq);
	qdot_mixed_ros.data.resize(nq);
	qdot_raw_ros.data.resize(nq);

	return true;
}


bool ROSLimitAvoider::startHook()
{
	return true;
}


void ROSLimitAvoider::cleanupHook()
{
}


bool ROSLimitAvoider::readLimits()
{
	urdf::Model robotModel;
	if(!robotModel.initParam("robot_description"))
	{
		ROS_ERROR("Failed to read robot description");
		return false;
	}

    Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robotModel, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

	Chain chain;
	tree.getChain(base_name, ee_name, chain);

	for(uint i=0; i < chain.getNrOfSegments(); ++i)
	{
		Joint j = chain.getSegment(i).getJoint();
		if(j.getType() != Joint::None)
		{
			string name = j.getName();
			robotModel.getJoint(j.getName());
			const boost::shared_ptr<urdf::JointSafety> safety =
			  robotModel.getJoint(j.getName())->safety;
			if(safety)
			{
				lim_min.push_back(safety->soft_lower_limit);
				lim_max.push_back(safety->soft_upper_limit);
			}
			else
			{
				lim_min.push_back(0.0);
				lim_max.push_back(0.0);
			}
			printf("joint %d: (%s) : %f .. %f\n", i, j.getName().c_str(),
			       lim_min[lim_min.size()-1], lim_max[lim_max.size()-1]);
		}
	}
	return true;
}

void ROSLimitAvoider::avoidLimits()
{
	double a = h / (d*d);
    for(unsigned int i=0; i < q.rows(); i++)
    {
        double q_lo = lim_min[i] + d;
        double q_hi = lim_max[i] - d;
        if(q(i) < q_lo && q(i) < q_hi)
            qdot(i) =    a * (q_lo - q(i))*(q_lo - q(i));
        else if(q(i) > q_hi && q(i) > q_lo)
            qdot(i) = - (a * (q(i) - q_hi)*(q(i) - q_hi));
        else
            qdot(i) = 0;
    }
}

void ROSLimitAvoider::projectAction()

{
    // pseudoinverse:
	// [u*s*v'] = svd(x)
    // pinv = v*s_p*u'
	// s_p: take every diagonal element and divide by 0
	// if diagonal element is zero, write zero

	// THATS how the tool features behaved...

	// then do qdot_out = (I - J^+ * J) * qdot_in

	int m=jac.rows(), n=jac.columns();
	//printf("jacobian has %d rows and %d columns.\n", m, n);

	MatrixXd U(m,n), V(n,n);
	MatrixXd jacinv(n,m);
	VectorXd S(n), Sp(n), tmp(n);

	MatrixXd tt(n,n);

	svd_eigen_HH(jac.data, U, S, V, tmp);

	double eps = 1e-7;
	for(int i=0; i <n; ++i)
		Sp(i) = (S(i) > eps) ? 1.0 / S(i) : 0.0;

	jacinv = V * Sp.asDiagonal() * U.transpose();

	qdot_out.data = (MatrixXd::Identity(n, n) - jacinv*jac.data)*qdot.data;
}

void ROSLimitAvoider::rescaleAvoidance()
{
	double factor = 0.0;
	double mag_sum = 0.0;
	for(int i=0; i < nq; ++i)
	{
		double reversed = ( sign(qdot(i)) != sign(qdot_out(i)) ) ? 1.0 : -1.0;
		double mag = abs(qdot(i)*qdot_out(i));
		factor  += -reversed*mag;
		mag_sum +=  mag;
	}
	if(mag_sum > 1e-6)
		factor /= mag_sum;

	for(int i=0; i < nq; ++i)
		qdot_out(i) *= factor;
}

void ROSLimitAvoider::updateHook()
{
	qdot_task_port.read(qdot_task);
	q_port.read(q);
    jac_port.read(jac);

    avoidLimits();
	projectAction();
	rescaleAvoidance();

	qdot_port.write(qdot_out);


	for (unsigned int i = 0; i < qdot_out.rows(); i++)
	{
		qdot_ros.data[i] = qdot_out(i);
		qdot_mixed_ros.data[i] = K_avoid*qdot_out(i) + K_task*qdot_task(i);
		qdot_raw_ros.data[i] = qdot(i);
	}

	qdot_ros_port.write(qdot_ros);
	qdot_mixed_ros_port.write(qdot_mixed_ros);
	qdot_raw_ros_port.write(qdot_raw_ros);
}

void ROSLimitAvoider::stopHook()
{

}


}

