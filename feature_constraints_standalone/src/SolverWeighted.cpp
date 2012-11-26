/*
 * SolverWeighted.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: bartelsg
 */

#include <SolverWeighted.hpp>
#include <ros/ros.h>
#include <kdl/utilities/svd_eigen_HH.hpp>

// NOTE: EPSILON is only used for warning the user,
// the computation itself uses lambda!
#define EPSILON 1e-6

SolverWeighted::SolverWeighted()
{
	// set lambda parameter
	lambda = 0.1;

	// TODO: include some mechanism to catch the error of someone calling the un-initialised solve...
}


SolverWeighted::SolverWeighted(unsigned int num_constraints,
		unsigned int num_joints)
{
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
        Sinv2.setZero(num_joints, num_joints);
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
		const unsigned int num_joints)
{
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
        Sinv2.setZero(num_joints, num_joints);
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
		const Eigen::MatrixXd &Wy, Eigen::VectorXd &qdot)
{
	// Create the Weighted Jacobian
	A_Wq = (A * Wq);
	Wy_A_Wq = (Wy * A_Wq);
	// Compute the SVD of the weighted jacobian
	int ret = KDL::svd_eigen_HH(Wy_A_Wq, U2, S2, V, tmp);
	if (ret < 0) {
        ROS_ERROR("SVD decomposition failed");
		return false;
	}

/* NOTE: ORIGINAL 'OPTIMIZED' CODE FROM LEUVEN HAD PROBLEMS WITH OVERSPECIFIED CASES
	// put U2 and S2 into U and S
	int block_size = std::min(num_constraints, num_joints);
	U.block(0, 0, block_size, block_size) = U2.block(0, 0, block_size, block_size);
	S.segment(0, block_size) = S2.segment(0, block_size);

	for (int j = 0; j < U.rows(); j++) {
		if (fabs(U2(j, num_joints - 1)) > EPSILON) {
            ROS_WARN("Element %d of the last column of U2 is not equal to zero, but = %f", j, U2(j, num_joints - 1));
		}
	}
	// FIXME: this check should be done for all entries of S2 from 7 to end.
	if (fabs(S2(num_joints - 1)) > EPSILON) {
        ROS_WARN("Last value of S2 is not equal to zero, but = %f", S2(num_joints - 1));
	}

	// Pre-multiply U and V by the task space and joint space weighting matrix respectively
	Wy_U = (Wy * U);
	Wq_V = (Wq * V);

	for (int i = 0; i < S.rows(); i++)
		Sinv(i, i) = (S(i) / (S(i) * S(i) + lambda * lambda));

	// qdot = Wq*V * S^-1 * U'*Wy' * ydot
	qdot = (Wq_V * Sinv * Wy_U.transpose() * ydot);

*/

       // BEGINNING RE-IMPLEMENTATION WITHOUT OPTIMIZATION

       // Pre-multiply U and V by the task space and joint space weighting matrix respectively
       Wy_U = (Wy * U2);
       Wq_V = (Wq * V);
 
       // invert S2 and also be aware of very small diagonale values
       for (unsigned int i = 0; i < S2.rows(); i++)
         Sinv2(i, i) = (S2(i) / (S2(i) * S2(i) + lambda * lambda));

       // qdot = Wq*V * S^-1 * U'*Wy' * ydot
       qdot = (Wq_V * Sinv2 * Wy_U.transpose() * ydot);

       return true;
}
