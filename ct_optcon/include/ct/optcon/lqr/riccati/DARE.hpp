/*
 * DARE.hpp
 *
 *  Created on: 15.09.2017
 *      Author: gimarkus
 */

#ifndef CT_OPTCON_INCLUDE_CT_OPTCON_LQR_RICCATI_DARE_HPP_
#define CT_OPTCON_INCLUDE_CT_OPTCON_LQR_RICCATI_DARE_HPP_

#include "DynamicRiccatiEquation.hpp"


namespace ct {
namespace optcon {


/*!
 * \ingroup LQR
 *+
 * \brief Discrete-Time Algebraic Riccati Equation
 *
 * solves the discrete-time Infinite-Horizon Algebraic Riccati Equation iteratively
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system control input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DARE {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
	typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_feedback_t;

	DARE(){}


	/*! compute the discrete-time steady state Riccati-Matrix
	 * this method iterates over the time-varying discrete-time Riccati Equation to compute the steady-state solution.
	 * @param Q state weight
	 * @param R control weight
	 * @param A discrete-time linear system matrix A
	 * @param B discrete-time linear system matrix B
	 * @param verbose print additional information
	 * @param eps treshold to stop iterating
	 * @return steady state riccati matrix P
	 */
	state_matrix_t computeSteadyStateRiccatiMatrix(
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const state_matrix_t& A,
			const control_gain_matrix_t& B,
			control_feedback_t& K,
			bool verbose = false,
			const SCALAR eps = 1e-6,
			size_t maxIter = 100)
	{
		state_matrix_t P, P_prev;
		size_t numIter = 0;

		SCALAR diff = 1;

		while (diff >= eps && numIter < maxIter)
		{
			state_matrix_t P_prev = P;
			dynamicRDE_.iterateRobust(Q, R, A, B, P, K);
			diff = (P-P_prev).cwiseAbs().maxCoeff();
			numIter++;
		}

		if(verbose)
			std::cout << "DARE : converged after " << numIter << " iterations out of a maximum of "<< maxIter << std::endl;

		return P;
	}


private:
	DynamicRiccatiEquation<STATE_DIM, CONTROL_DIM> dynamicRDE_;

};

}	// namespace optcon
}	// namespace ct

#endif /* CT_OPTCON_INCLUDE_CT_OPTCON_LQR_RICCATI_DARE_HPP_ */
