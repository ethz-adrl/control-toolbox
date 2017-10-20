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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#pragma once

#include <ct/core/control/Controller.h>
#include <ct/core/types/FeedbackMatrix.h>

namespace ct {
namespace core {

//! A constant state feedback controller
/*!
 * Implements a state feedback controller which works on a constant setpoint, i.e. it features
 * - one constant pure feedforward term
 * - one constant state reference
 * - one constant Feedback matrix
 *
 * and takes the form
 *
 * \f[
 *
 * u(x) = u_{ff} + K (x - x_{ref})
 *
 * \f]
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstantStateFeedbackController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Default constructor
	/*!
	 * Sets the control signal to zero
	 */
	ConstantStateFeedbackController()
	{
		u_ff_.setZero();
		x_ref_.setZero();
		K_.setZero();
	}


	//! Constructor
	/*!
	 * Initializes the control to a fixed value
	 * @param u The fixed feedforward control signal
	 * @param x The fixed reference state
	 * @param K the fixed state feedback gain
	 */
	ConstantStateFeedbackController(const ControlVector<CONTROL_DIM, SCALAR>& uff,
		const StateVector<STATE_DIM, SCALAR>& x,
		const FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& K)
		: u_ff_(uff), x_ref_(x), K_(K)
	{
	}


	//! Copy constructor
	ConstantStateFeedbackController(const ConstantStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>& other)
		: Controller<STATE_DIM, CONTROL_DIM, SCALAR>(other), u_ff_(other.u_ff_), x_ref_(other.x_ref_), K_(other.K_)
	{
	}


	//! Destructor
	~ConstantStateFeedbackController() {}
	//! Clone operator
	/*!
	 * Clones the controller. Used for cloning ControlledSystem's
	 * @return pointer to cloned controller
	 */
	ConstantStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new ConstantStateFeedbackController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}


	//! Computes current control
	/*!
	 * Returns the fixed control signal. Therefore, the return value is invariant
	 * to the parameters.
	 * @param state The current state of the system (ignored)
	 * @param t The time of the system (ignored)
	 * @param controlAction The (fixed) control action
	 */
	void computeControl(const StateVector<STATE_DIM, SCALAR>& x,
		const SCALAR& t,
		ControlVector<CONTROL_DIM, SCALAR>& controlAction) override
	{
		controlAction = u_ff_ + K_ * (x - x_ref_);
	}


	//! update the control law with a new set of parameters
	/*!
	 * @param u The fixed feedforward control signal
	 * @param x The fixed reference state
	 * @param K the fixed state feedback gain
	 */
	void updateControlLaw(const ControlVector<CONTROL_DIM, SCALAR>& uff,
		const StateVector<STATE_DIM, SCALAR>& x,
		const FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>& K =
			FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR>::Zero())
	{
		u_ff_ = uff;
		x_ref_ = x;
		K_ = K;
	}


private:
	//! feedforward control input
	ControlVector<CONTROL_DIM, SCALAR> u_ff_;

	//! reference state
	StateVector<STATE_DIM, SCALAR> x_ref_;

	//! state feedback matrix
	FeedbackMatrix<STATE_DIM, CONTROL_DIM, SCALAR> K_;
};

}  // namespace core
}  // namespace ct
