/*
 * SolverWeighted.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: bartelsg
 */

#include <SolverWeighted.hpp>
#include <ros/ros.h>
#include <kdl/utilities/svd_eigen_HH.hpp>

SolverWeighted::SolverWeighted() {
	// set lambda parameter
	lambda = 0.1;

	// TODO: include some mechanism to catch the error of someone calling the un-initialised solve...
}
SolverWeighted::SolverWeighted(unsigned int num_constraints,
		unsigned int num_joints) {
	this->num_constraints = num_constraints;
	this->num_joints = num_joints;

	// set lambda parameter
	lambda = 0.1;

	// resize and initialise all internal temporary structures
	A_Wq.resize(num_constraints, num_joints);
	Wy_A_Wq.resize(num_constraints, num_joints);
	U.setZero(num_constraints, num_constraints);
	U2.setZero(num_constraints, num_joints);
	V.setIdentity(num_joints, num_joints);
	Sinv.setZero(num_joints, num_constraints);
	Wy_U.resize(num_constraints, num_constraints);
	Wq_V.resize(num_joints, num_joints);

	S.setConstant(std::min(num_constraints, num_joints), 1.0);
	S2.setConstant(num_joints, 1.0);
	tmp.setZero(num_joints);
	Ut_Wyt_ydot.resize(num_constraints);
	Sinv_Ut_Wyt_ydot(num_joints);
}

SolverWeighted::~SolverWeighted() {

}

void SolverWeighted::reinitialise(const unsigned int num_constraints,
		const unsigned int num_joints) {
	// basically, just redo what the constructor does for the new dimensions

	this->num_constraints = num_constraints;
	this->num_joints = num_joints;

	// resize and initialise all internal temporary structures
	A_Wq.resize(num_constraints, num_joints);
	Wy_A_Wq.resize(num_constraints, num_joints);
	U.setZero(num_constraints, num_constraints);
	U2.setZero(num_constraints, num_joints);
	V.setIdentity(num_joints, num_joints);
	Sinv.setZero(num_joints, num_constraints);
	Wy_U.resize(num_constraints, num_constraints);
	Wq_V.resize(num_joints, num_joints);

	S.setConstant(std::min(num_constraints, num_joints), 1.0);
	S2.setConstant(num_joints, 1.0);
	tmp.setZero(num_joints);
	Ut_Wyt_ydot.resize(num_constraints);
	Sinv_Ut_Wyt_ydot.resize(num_joints);
}

/* This solves the equation A qdot = ydot for qdot using the weighted
 * pseudoinverse, where Wq denotes the weights of the joints and
 * Wy denotes the weights of the constraints.
 *
 * Note: Wq is a num_joints x num_joints matrix.
 * 		 Wy is a num_constraints x num_constraints matrix.
 */
bool SolverWeighted::solve(const Eigen::MatrixXd &A,
		const Eigen::VectorXd &ydot, const Eigen::MatrixXd &Wq,
		const Eigen::MatrixXd &Wy, Eigen::VectorXd &qdot) {
	// TODO:implement this
	// Create the Weighted jacobian
	A_Wq = (A * Wq);
	Wy_A_Wq = (Wy * A_Wq);
	// Compute the SVD of the weighted jacobian
	int ret = KDL::svd_eigen_HH(Wy_A_Wq, U2, S2, V, tmp);
	if (ret < 0) {
		// TODO: throw some error message
		return false;
	}

	//put U2 and S2 into U and S
	U = U2.block(0, 0, num_constraints, num_constraints);
	S = S2.segment(0, num_constraints);

	for (int j = 0; j < U.rows(); j++) {
		if (U2(j, num_joints - 1) != 0) {
			// TODO: print some ROS warning
			/*
			 log(Warning) << "Element " << j
			 << " of the last column of U2 is not equal to zero, but = "
			 << U2(j, nq_local - 1) << endlog();
			 */
		}
	}
	if (S2(num_joints - 1) != 0) {
		// TODO: print some ROS warning
		/*
		 log(Warning) << "Last value of S2 is not equal to zero, but = "
		 << S2(nq_local - 1) << endlog();
		 */
	}

	//Pre-multiply U and V by the task space and joint space weighting matrix respectively
	Wy_U = (Wy * U);
	Wq_V = (Wq * V);

	for (int i = 0; i < S.rows(); i++)
		Sinv(i, i) = (S(i) / (S(i) * S(i) + lambda * lambda));

	//U'*Wy'*ydot
	Ut_Wyt_ydot = (Wy_U.transpose() * ydot);
	//S^-1*U'*Wy'*ydot
	Sinv_Ut_Wyt_ydot = (Sinv * Ut_Wyt_ydot);
	//qdot=Wq*V*S^-1*U'*Wy'*ydot
	qdot = (Wq_V * Sinv_Ut_Wyt_ydot);

	return true;
}
