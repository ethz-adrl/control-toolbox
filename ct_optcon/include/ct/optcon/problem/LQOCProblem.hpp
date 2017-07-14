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

#ifndef INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_
#define INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_

namespace ct {
namespace optcon {

/*!
 *
 * This class defines a Linear Quadratic Optimal Control Problem, consisting of
 * - affine systen dynamics
 * - reference trajectories (arrays!) for state and control
 * - LQ approximation of the cost function
 *
 */
template <int STATE_DIM, int CONTROL_DIM, typename SCALAR = double>
class LQOCProblem
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! constructor
	LQOCProblem(int N = 0)
	{
		changeNumStages(N);
	}

	//! returns the number of discrete time steps in the LOCP, including terminal stage
	int getNumberOfStages()
	{
		return K_;
	}

	//! change the number of discrete time steps in the LOCP
	void changeNumStages(int N)
	{
		K_ = N;

		A_.resize(N);
		B_.resize(N);
		b_.resize(N+1);

		x_.resize(N+1);
		u_.resize(N);

		P_.resize(N);
		q_.resize(N+1);
		qv_.resize(N+1);
		Q_.resize(N+1);

		rv_.resize(N);
		R_.resize(N);
	}


	//! affine, time-varying system dynamics in discrete time
	ct::core::StateMatrixArray<STATE_DIM, SCALAR> A_;
	ct::core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM, SCALAR> B_;
	ct::core::StateVectorArray<STATE_DIM, SCALAR> b_;

	//! reference state trajectory
	ct::core::StateVectorArray<STATE_DIM, SCALAR> x_;

	//! reference control trajectory
	ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> u_;

	//! constant term of in the LQ approximation of the cost function
	ct::core::ScalarArray<SCALAR> q_;

	//! LQ approximation of the pure state penalty, including terminal state penalty
	ct::core::StateVectorArray<STATE_DIM, SCALAR> qv_;
	ct::core::StateMatrixArray<STATE_DIM, SCALAR> Q_;

	//! LQ approximation of the pure control penalty
	ct::core::ControlVectorArray<CONTROL_DIM, SCALAR> rv_;
	ct::core::ControlMatrixArray<CONTROL_DIM, SCALAR> R_;

	//! LQ approximation of the cross terms of the cost function
	ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR> P_;


	void setFromTimeInvariantLinearQuadraticProblem(
		ct::core::StateVector<STATE_DIM>& x0,
		ct::core::ControlVector<CONTROL_DIM>& u0,
		ct::core::DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>& linearSystem,
		ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>& costFunction,
		ct::core::StateVector<STATE_DIM>& stateOffset,
		double dt
	)
	{
		core::StateMatrix<STATE_DIM, SCALAR> A;
		core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> B;
		linearSystem.getAandB(x0, 0, u0, A, B);

		A_ = core::StateMatrixArray<STATE_DIM>(K_, A);
		B_ = core::StateControlMatrixArray<STATE_DIM, CONTROL_DIM>(K_, B);
		b_ = core::StateVectorArray<STATE_DIM>(K_+1, stateOffset);


		// feed current state and control to cost function
		costFunction.setCurrentStateAndControl(x0, u0, 0);

		// derivative of cost with respect to state
		qv_ = core::StateVectorArray<STATE_DIM, SCALAR>(K_+1, costFunction.stateDerivativeIntermediate()*dt);
		Q_ = core::StateMatrixArray<STATE_DIM, SCALAR>(K_+1, costFunction.stateSecondDerivativeIntermediate()*dt);

		// derivative of cost with respect to control and state
		P_ = core::FeedbackArray<STATE_DIM, CONTROL_DIM, SCALAR>(K_, costFunction.stateControlDerivativeIntermediate()*dt);

		// derivative of cost with respect to control
		rv_ = core::ControlVectorArray<CONTROL_DIM, SCALAR>(K_, costFunction.controlDerivativeIntermediate()*dt);

		R_ = core::ControlMatrixArray<CONTROL_DIM, SCALAR>(K_, costFunction.controlSecondDerivativeIntermediate()*dt);

		Q_[K_] = costFunction.stateSecondDerivativeTerminal();
		qv_[K_] = costFunction.stateDerivativeTerminal();

		x_ = core::StateVectorArray<STATE_DIM, SCALAR>(K_+1, x0);
		u_ = core::ControlVectorArray<CONTROL_DIM, SCALAR>(K_, u0);
	}


private:

	//! the number of discrete time steps in the LOCP, including terminal stage
	int K_;

};

} 	//! optcon
}	//! ct



#endif /* INCLUDE_CT_OPTCON_PROBLEM_LQOCPROBLEM_H_ */
