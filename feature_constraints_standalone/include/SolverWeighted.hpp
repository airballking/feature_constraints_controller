/* FROM: Georg Bartels, georg.bartels@cs.tum.edu
 *
 * This is a modified and combined version of WDLSVelocitySolver.hpp and Solver.hpp
 * from the iTaSC framework of KU Leuven. I modified their solver to use it in my
 * code with using their overall framework. Note, that this is just for testing
 * purposes. So, there's no warranty that this code works. Here comes KU Leuven's
 * disclaimer form WDLSVelocitySolver.hpp:
 */

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

/*
 * SolverWeighted.hpp
 *
 *  Created on: Mar 1, 2012
 *      Author: bartelsg
 */

#ifndef SOLVERWEIGHTED_HPP_
#define SOLVERWEIGHTED_HPP_

#include <Eigen/Core>

class SolverWeighted {
public:
	SolverWeighted();
	SolverWeighted(const unsigned int num_constraints,
			const unsigned int num_joints);
	~SolverWeighted();

	/* This solves the equation A qdot = ydot for qdot using the weighted
	 * pseudoinverse, where Wq denotes the weights of the joints and
	 * Wy denotes the weights of the constraints.
	 *
	 * Note: Wq is a num_joints x num_joints matrix.
	 * 		 Wy is a num_constraints x num_constraints matrix.
	 */

	bool solve(const Eigen::MatrixXd &A, const Eigen::VectorXd &ydot,
			const Eigen::MatrixXd &Wq, const Eigen::MatrixXd & Wy,
			Eigen::VectorXd &qdot);

	void reinitialise(const unsigned int num_constraints, const unsigned int num_joints);

private:
	unsigned int num_constraints, num_joints;

	// value used during inversion of the diagonal matrix
	// I got this value 0.1 from Ingo; he looked it up in some robot-specific config file
	// the file also said:
	// lambda = 0.01 --> high tracking accuracy
	// lambda = 0.5 --> high clearance of singularities
	double lambda;

	Eigen::MatrixXd A_Wq, Wy_A_Wq;
	Eigen::MatrixXd U, V, Sinv, Wy_U, Wq_V, U2, Sinv2;
	Eigen::VectorXd S, tmp, Ut_Wyt_ydot, Sinv_Ut_Wyt_ydot, S2;
};

#endif /* SOLVERWEIGHTED_HPP_ */
