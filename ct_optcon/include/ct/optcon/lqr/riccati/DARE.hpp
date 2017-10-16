/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
 * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************/

#pragma once

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

	DARE();


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
			size_t maxIter = 100);


private:
	DynamicRiccatiEquation<STATE_DIM, CONTROL_DIM> dynamicRDE_;

};

}	// namespace optcon
}	// namespace ct
