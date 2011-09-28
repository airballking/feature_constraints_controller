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
#include "WDLSVelocitySolver.hpp"

#include <ocl/Component.hpp>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <rtt/Logger.hpp>

#include <rtt/os/TimeService.hpp>

ORO_CREATE_COMPONENT( iTaSC::WDLSVelocitySolver )
;

namespace iTaSC {
using namespace Eigen;
using namespace std;
using namespace KDL;
using namespace RTT;

WDLSVelocitySolver::WDLSVelocitySolver(const string& name) :
	Solver(name), lambda("lambda", 0.1) {
	provides()->addAttribute(lambda);
}

bool WDLSVelocitySolver::configureHook() {
	nc_local = nc;
	nq_local = nq;

	A.resize(nc_local, nq_local);
	Wy.resize(nc_local, nc_local);
	Wq.resize(nq_local, nq_local);
	qdot.resize(nq_local);
	ydot.resize(nc_local);

	A_Wq.resize(nc_local, nq_local);
	Wy_A_Wq.resize(nc_local, nq_local);
	U.resize(nc_local, nc_local);
	U.setZero();
	U2.resize(nc_local, nq_local); //test
	U2.setZero(); //test
	V.resize(nq_local, nq_local);
	V.setIdentity();
	Sinv.resize(nq_local, nc_local);
	Sinv.setZero();
	Wy_U.resize(nc_local, nc_local);
	Wq_V.resize(nq_local, nq_local);
	S.resize(min(nc_local, nq_local));
	S.setConstant(1.0);
	S2.resize( nq_local); //test
	S2.setConstant(1.0); //test
	tmp.resize(nq_local);
	tmp.setZero(); //initialize
	Ut_Wyt_ydot.resize(nc_local);
	Sinv_Ut_Wyt_ydot.resize(nq_local);

	return true;
}

bool WDLSVelocitySolver::solve() {
	Logger::In in(this->getName());

	// Create the Weighted jacobian
	A_port.read(A);
	Wq_port.read(Wq);
	Wy_port.read(Wy);
	A_Wq = (A * Wq).lazy();
	Wy_A_Wq = (Wy * A_Wq).lazy();
#ifndef NDEBUG
	log(Debug) << "Wy_A_Wq: " << Wy_A_Wq << endlog();
#endif
	// Compute the SVD of the weighted jacobian
	int ret = svd_eigen_HH(Wy_A_Wq, U2, S2, V, tmp);
	if (ret < 0) {
		log(Error)<<"svd_eigen_HH went fatal"<<endlog();
		this->fatal();
		return false;
	}
	//put U2 and S2 into U and S
	U = U2.block(0, 0, nc_local, nc_local);
	S = S2.segment(0,nc_local);

	for(int j = 0;j<U.rows();j++)
	{
		if(U2(j,nq_local-1)!=0)
		{
			log(Warning) << "Element " <<j << " of the last column of U2 is not equal to zero, but = " <<U2(j,nq_local-1) <<endlog();
		}
	}
	if(S2(nq_local-1)!=0)
	{
		log(Warning)<< "Last value of S2 is not equal to zero, but = " << S2(nq_local-1) <<endlog();
	}

#ifndef NDEBUG
	log(Debug) << "S: " << S << endlog();
	log(Debug) << "U:" << U << endlog();
	log(Debug) << "V:" << V << endlog();
#endif
	//Pre-multiply U and V by the task space and joint space weighting matrix respectively
	Wy_U = (Wy * U).lazy();
	Wq_V = (Wq * V).lazy();

#ifndef NDEBUG
	log(Debug) << "Wy_U: " << Wy_U << endlog();
	log(Debug) << "Wq_V: " << Wq_V << endlog();
#endif

	for (int i = 0; i < S.rows(); i++)
		Sinv(i, i) = (S(i) / (S(i) * S(i) + lambda.get() * lambda.get()));

#ifndef NDEBUG
	log(Debug) << "Sinv: " << Sinv << endlog();
#endif

	//U'*Wy'*ydot
	ydot_port.read(ydot);
	Ut_Wyt_ydot = (Wy_U.transpose() * ydot).lazy();
	//S^-1*U'*Wy'*ydot
	Sinv_Ut_Wyt_ydot = (Sinv * Ut_Wyt_ydot).lazy();
	//qdot=Wq*V*S^-1*U'*Wy'*ydot
	qdot = (Wq_V * Sinv_Ut_Wyt_ydot).lazy();

	qdot_port.write(qdot);
	return true;

}

}

