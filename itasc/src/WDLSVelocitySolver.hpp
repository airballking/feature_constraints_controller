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
#ifndef _ITASC_WDLSVELOCITYSOLVER_HPP
#define _ITASC_WDLSVELOCITYSOLVER_HPP

#include <rtt/TaskContext.hpp>

#include "Solver.hpp"

namespace iTaSC {

class WDLSVelocitySolver: public Solver {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	WDLSVelocitySolver(const std::string& name);
	~WDLSVelocitySolver() {
	}
	;

	virtual bool configureHook();
	virtual bool startHook() {
		return true;
	}
	;
	virtual void updateHook() {
	}
	;
	virtual void stopHook() {
	}
	;
	virtual void cleanupHook() {
	}
	;

	virtual bool solve();

private:
	unsigned int nc_local, nq_local;

	Eigen::MatrixXd A, Wq, Wy;
	Eigen::VectorXd qdot;
	Eigen::VectorXd ydot;

	Eigen::MatrixXd A_Wq, Wy_A_Wq;
	Eigen::MatrixXd U, V, Sinv, Wy_U, Wq_V, U2;
	Eigen::VectorXd S, tmp, Ut_Wyt_ydot, Sinv_Ut_Wyt_ydot, S2;

	RTT::Attribute<double> lambda;

};

}
#endif
