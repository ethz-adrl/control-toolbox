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

#ifndef CT_LINEARSYSTEM_H_
#define CT_LINEARSYSTEM_H_

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! interface class for a general linear system or linearized system
/*!
 * Defines the interface for a linear system
 *
 * \tparam STATE_DIM size of state vector
 * \tparam CONTROL_DIM size of input vector
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class LinearSystem : public ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>{

public:

	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t; //!< state Jacobian type
	typedef typename Eigen::Matrix<SCALAR, STATE_DIM, CONTROL_DIM> state_control_matrix_t; //!< input Jacobian type

	//! default constructor
	/*!
	 * @param type system type
	 */
	LinearSystem(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL):
		ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(type)
		{}

	//! destructor
	virtual ~LinearSystem(){};

	//! deep cloning
	virtual LinearSystem<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const = 0;

	//! compute the system dynamics
	/*!
	 * This computes the system dynamics
	 * \f[
	 *  \dot{x} = Ax + Bu
	 * \f]
	 * @param state current state
	 * @param t current time
	 * @param control control input
	 * @param derivative state derivative
	 */
	virtual void computeControlledDynamics(
				const StateVector<STATE_DIM, SCALAR>& state,
				const SCALAR& t,
				const ControlVector<CONTROL_DIM, SCALAR>& control,
				StateVector<STATE_DIM, SCALAR>& derivative
		) override
	{
		// x_dot(t) = A(x,u,t) * x(t) + B(x,u,t) * u(t)

		derivative = getDerivativeState(state, control, t) * state
				+ getDerivativeControl(state, control, t) * control;
	}

	//! get the A matrix of a linear system
	/*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return A matrix
	 */
	virtual const state_matrix_t& getDerivativeState(const StateVector<STATE_DIM, SCALAR>& x, const ControlVector<CONTROL_DIM, SCALAR>& u, const SCALAR t = 0.0) = 0;

	//! get the B matrix of a linear system
	/*!
	 * @param x state vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param u input vector (required for linearizing non-linear systems, ignored for pure linear system)
	 * @param t current time
	 * @return B matrix
	 */
	virtual const state_control_matrix_t& getDerivativeControl(const StateVector<STATE_DIM, SCALAR>& x, const ControlVector<CONTROL_DIM, SCALAR>& u, const SCALAR t = 0.0) = 0;

};

}
}

#endif



