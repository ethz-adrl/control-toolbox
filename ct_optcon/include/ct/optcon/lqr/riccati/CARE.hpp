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

#include <iostream>

// Schur reordering from Lapack
#ifdef CT_USE_LAPACK
extern "C" void dtrsen_(const char* JOB,
	const char* COMPQ,
	const int* SELECT,
	const int* N,
	const double* T,
	const int* LDT,
	const double* Q,
	const int* LDQ,
	double* WR,
	double* WI,
	int* M,
	double* S,
	double* SEP,
	double* WORK,
	const int* LWORK,
	int* IWORK,
	const int* LIWORK,
	int* INFO);
#endif

namespace ct {
namespace optcon {

/*!
 * \ingroup LQR
 *+
 * \brief Continuous-Time Algebraic Riccati Equation
 *
 * solves the continuous-time Infinite-Horizon Algebraic Riccati Equation
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system control input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class CARE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
	typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
	typedef Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> control_feedback_t;

	typedef Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> schur_matrix_t;
	typedef Eigen::Matrix<double, 2 * STATE_DIM, STATE_DIM> factor_matrix_t;

	CARE();

	// Implementation using the Schur-Method
	// This is numerically more stable and should be preferred over the naive implementation
	bool solve(const state_matrix_t& Q,
		const control_matrix_t& R,
		const state_matrix_t& A,
		const control_gain_matrix_t& B,
		state_matrix_t& P,
		bool RisDiagonal,
		control_matrix_t& R_inverse,
		bool useIterativeSolver = false);

	state_matrix_t computeSteadyStateRiccatiMatrix(const state_matrix_t& Q,
		const control_matrix_t& R,
		const state_matrix_t& A,
		const control_gain_matrix_t& B,
		const bool RisDiagonal = false,
		const bool useIterativeSolver = false);

private:
	bool solveSchurIterative(const schur_matrix_t& M,
		state_matrix_t& P,
		double epsilon = 1e-6,
		int maxIterations = 100000);

	bool solveSchurDirect(const schur_matrix_t& M, state_matrix_t& P);

	Eigen::RealSchur<schur_matrix_t> schur_;
	Eigen::FullPivLU<factor_matrix_t> FullPivLU_;

	// Lapack
	int LWORK_;
	int LIWORK_;

	Eigen::VectorXd WORK_;
	Eigen::VectorXi IWORK_;
};
}
}
