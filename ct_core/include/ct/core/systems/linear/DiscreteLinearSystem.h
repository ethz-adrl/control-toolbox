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

#ifndef INCLUDE_CT_CORE_SYSTEMS_LINEAR_DISCRETELINEARSYSTEM_H_
#define INCLUDE_CT_CORE_SYSTEMS_LINEAR_DISCRETELINEARSYSTEM_H_

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! interface class for a general discrete linear system or linearized discrete system
/*!
 * Defines the interface for a discrete linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteLinearSystem : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t; //!< state Jacobian type
	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t; //!< input Jacobian type

	//! default constructor
	/*!
	 * @param type system type
	 */
	DiscreteLinearSystem(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL):
		ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type)
		{}

	//! destructor
	virtual ~DiscreteLinearSystem(){};

	//! deep cloning
	virtual DiscreteLinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

	//! compute the system dynamics
	/*!
	 * This computes the system dynamics
	 * \f[
	 *  x_{n+1} = Ax_n + Bu_n
	 * \f]
	 * @param state current state
	 * @param n current time index
	 * @param control control input
	 * @param stateNext propagated state
	 */
	virtual void propagateControlledDynamics(
				const StateVector<STATE_DIM, SCALAR>& state,
				const int& n,
				const ControlVector<CONTROL_DIM, SCALAR>& control,
				StateVector<STATE_DIM, SCALAR>& stateNext
		) override
	{
		state_matrix_t A;
		state_control_matrix_t B;
		this->getAandB(state, n, control, A, B);
		stateNext = A * state + B * control;
	}

	virtual void getAandB(
			const StateVector<STATE_DIM, SCALAR>& x,
			const int n,
			const ControlVector<CONTROL_DIM, SCALAR>& u,
			state_matrix_t& A,
			state_control_matrix_t& B)
	= 0;

};

}
}



#endif /* INCLUDE_CT_CORE_SYSTEMS_LINEAR_DISCRETELINEARSYSTEM_H_ */
