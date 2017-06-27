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

#ifndef CT_OPTCON_FHDTLQR_HPP_
#define CT_OPTCON_FHDTLQR_HPP_

#include <ct/optcon/lqr/riccati/DynamicRiccatiEquation.hpp>
#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>
#include <ct/core/core.h>

namespace ct {
namespace optcon {



/*! \defgroup LQR LQR
 *
 * \brief Linear Quadratic Regulator Module
 * This module holds two verified LQR implementations in C++.
 */

/*!
 * \ingroup LQR
 *
 * \brief Finite-Horizon Discrete Time LQR
 *
 * compute the finite-horizon discrete time LQR solution
 * (Example: stabilize a linear time-varying system about a trajectory).
 * The user can either provide the linearized system matrices, or alternatively a pointer to a derivatives instance
 *
 * The feedback law has form
 * \f[
 * u_{fb} = -K \cdot (x - x_{ref})
 * \f]
 *
 * @tparam STATE_DIM system state dimension
 * @tparam CONTROL_DIM system input dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class FHDTLQR
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
	typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
	typedef Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> control_gain_matrix_t;
	typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_feedback_t;

	typedef core::StateVectorArray<STATE_DIM, SCALAR> state_vector_array_t;
	typedef core::ControlVectorArray<CONTROL_DIM, SCALAR> control_vector_array_t;
	typedef ct::core::StateMatrixArray<STATE_DIM, SCALAR> state_matrix_array_t;

	typedef ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> control_feedback_array_t;
	typedef ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> control_gain_matrix_array_t;


	//! Constructor
	/*!
	 * @param costFunction the cost function to be used for designing the TVLQR
	 */
	FHDTLQR(
		std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> > costFunction
	) :
		costFunction_(costFunction)
	{}

	~FHDTLQR() {};


	//! design a time-varying feedback trajectory using user-provided matrices A and B
	/*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param A
	 * 	trajectory of system matrices A = df/dx
	 * @param B
	 *  trajectory of system matrices B = df/du
	 * @param dt
	 *  sampling time
	 * @param K
	 *  trajectory of resulting control feedback arrays
	 * @param performNumericalChecks
	 * (optional) perform some numerical checks while solving for K
	 */
	void designController(
		const state_vector_array_t& x_trajectory,
		const control_vector_array_t& u_trajectory,
		const state_matrix_array_t& A,
		const control_gain_matrix_array_t& B,
		SCALAR dt,
		control_feedback_array_t& K,
		bool performNumericalChecks = true
	)
	{
		size_t N = x_trajectory.size()-1;
		solve(x_trajectory, u_trajectory, A, B, N, dt, K, performNumericalChecks);
	}


	//! design a time-varying feedback trajectory using a user-provided derivative-pointer
	/*!
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param derivatives
	 *  shared_ptr to a LinearSystem, allowing to compute A and B
	 * @param dt
	 *  sampling time
	 * @param K
	 *  trajectory of resulting control feedback arrays
	 * @param performNumericalChecks
	 * (optional) perform some numerical checks while solving for K
	 */
	void designController(
			const state_vector_array_t& x_trajectory,
			const control_vector_array_t& u_trajectory,
			std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> > derivatives,
			SCALAR dt,
			control_feedback_array_t& K,
			bool performNumericalChecks = true
		)
	{
		size_t N = x_trajectory.size()-1;

		state_matrix_array_t A;
		control_gain_matrix_array_t B;

		linearizeModel(x_trajectory, u_trajectory, N, dt, derivatives, A, B);

		solve(x_trajectory, u_trajectory, A, B, N, dt, K, performNumericalChecks);
	}


private:

	//! compute trajectories of A and B matrices along the given reference trajectory using the user-provided derivative instance
	/*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param N
	 *  number of discrete sampling points
	 * @param dt
	 *  sampling time dt
	 * @param derivatives
	 *  user-provided derivatives
	 * @param A
	 *  resulting linear state matrices A
	 * @param B
	 *  resulting linear state-input matrices B
	 */
	void linearizeModel(
			const state_vector_array_t& x_trajectory,
			const control_vector_array_t& u_trajectory,
			size_t N,
			SCALAR dt,
			std::shared_ptr<core::LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR> >& derivatives,
			state_matrix_array_t& A,
			control_gain_matrix_array_t& B
	)
	{
		A.resize(N);
		B.resize(N);

		for (int i=0; i<N; i++)
		{
			A[i] = state_matrix_t::Identity() + dt * derivatives->getDerivativeState(x_trajectory[i], u_trajectory[i]);
			B[i] = dt * derivatives->getDerivativeControl(x_trajectory[i], u_trajectory[i]);
		}
	}


	//! solve for the LQR feedback gains
	/*!
	 *
	 * @param x_trajectory
	 * 	state reference trajectory
	 * @param u_trajectory
	 *  control reference trajectory
	 * @param A
	 *  given linear state matrices A
	 * @param B
	 *  given linear state-input matrices B
	 * @param N
	 *  number of discrete sampling points
	 * @param dt
	 *  sampling time
	 * @param K
	 *  the resulting array of state-feedback matrices
	 * @param performNumericalChecks
	 *  (optional) perform some numerical checks while solving
	 */
	void solve(
		const state_vector_array_t& x_trajectory,
		const control_vector_array_t& u_trajectory,
		const state_matrix_array_t& A,
		const control_gain_matrix_array_t& B,
		size_t N,
		SCALAR dt,
		control_feedback_array_t& K,
		bool performNumericalChecks = true
	)
	{
		if (x_trajectory.size() != N+1)
		{
			throw std::runtime_error("State trajectory does not have correct length. State trajectory length: "
					+ std::to_string(x_trajectory.size()) + ", should be: " + std::to_string(N+1));
		}
		if (u_trajectory.size() != N)
		{
			throw std::runtime_error("Input trajectory does not have correct length. Input trajectory length: "
					+ std::to_string(u_trajectory.size()) + ", should be: " + std::to_string(N));
		}
		if (A.size() != N)
		{
			throw std::runtime_error("Linearization A does not have correct length. Linearization length: "
					+ std::to_string(A.size()) + ", should be: " + std::to_string(N));
		}
		if (B.size() != N)
		{
			throw std::runtime_error("Linearization B does not have correct length. Linearization length: "
					+ std::to_string(B.size()) + ", should be: " + std::to_string(N));
		}

		K.resize(N);

		// initialize cost-to-go
		costFunction_->setCurrentStateAndControl(x_trajectory[N], control_vector_t::Zero(), dt*N);
		state_matrix_t P = costFunction_->stateSecondDerivativeTerminal();

		if (performNumericalChecks)
		{
			Eigen::Matrix<SCALAR, -1, 1> realEigVals = P.eigenvalues().real();
			if (realEigVals.minCoeff() < 0.0) { std::cout << "P: " << std::endl << P; throw std::runtime_error("[LQR] Q is not positive semi-definite."); }
		}

		for (int i=N-1; i>=0; i--)
		{
			costFunction_->setCurrentStateAndControl(x_trajectory[i], u_trajectory[i], i*dt);

			state_matrix_t Q = costFunction_->stateSecondDerivativeIntermediate() * dt;

			control_matrix_t R = costFunction_->controlSecondDerivativeIntermediate() * dt;

			if (performNumericalChecks)
			{
				if (Q.minCoeff() < 0.0) { std::cout << "Q: " << std::endl << Q; throw std::runtime_error("[LQR] Q contains negative entries."); }
				if (R.minCoeff() < 0.0) { std::cout << "R: " << std::endl << R; throw std::runtime_error("[LQR] R contains negative entries."); }
				if (R.diagonal().minCoeff() <= 0.0) { std::cout << "R: " << std::endl << R; throw std::runtime_error("[LQR] R contains zero entries on the diagonal."); }
				if (!Q.isDiagonal()) { std::cout << "[LQR] Warning: Q is not diagonal."<<std::endl; }
				if (!R.isDiagonal()) { std::cout << "[LQR] Warning: R is not diagonal."<<std::endl; }
			}

			//ricattiEq_.iterateNaive(Q, R, A, B, P, K[i]);
			ricattiEq_.iterateRobust(Q, R, A[i], B[i], P, K[i]);
		}
	}

	std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR> > costFunction_;	//! a quadratic costfunction for solving the optimal control problem
	DynamicRiccatiEquation<STATE_DIM, CONTROL_DIM, SCALAR> ricattiEq_;	//! the Riccati Equations
};

} // namespace optcon
} // namespace ct

#endif /* FHDTLQR_HPP_ */
