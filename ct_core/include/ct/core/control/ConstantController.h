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

#ifndef CT_CORE_CONSTANT_CONTROLLER_
#define CT_CORE_CONSTANT_CONTROLLER_

#include "ct/core/control/Controller.h"

namespace ct {
namespace core {

//! A constant controller
/*!
 * Implements a controller that is time and state invariant, i.e. fully constant.
 * This class is useful to integrate a ControlledSystem forward subject to a
 * constant control input.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ConstantController : public Controller<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Default constructor
	/*!
	 * Sets the control signal to zero
	 */
	ConstantController()
	{
		u_.setZero();
	}

	//! Constructor
	/*!
	 * Initializes the control to a fixed value
	 * @param u The fixed control signal
	 */
	ConstantController(ControlVector<CONTROL_DIM, SCALAR>& u): u_(u) {}

	//! Copy constructor
	ConstantController(const ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>& other) :
		Controller<STATE_DIM, CONTROL_DIM, SCALAR>(other),
		u_(other.u_)
	{}

	//! Destructor
	~ConstantController() {}

	//! Clone operator
	/*!
	 * Clones the controller. Used for cloning ControlledSystem's
	 * @return pointer to cloned controller
	 */
	ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>* clone() const override
	{
		return new ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>(*this);
	}

	//! Computes current control
	/*!
	 * Returns the fixed control signal. Therefore, the return value is invariant
	 * to the parameters.
	 * @param state The current state of the system (ignored)
	 * @param t The time of the system (ignored)
	 * @param controlAction The (fixed) control action
	 */
	void computeControl(
			const StateVector<STATE_DIM, SCALAR>& state,
			const SCALAR& t,
			ControlVector<CONTROL_DIM, SCALAR>& controlAction) override
	{
		controlAction = u_;
	}

	//! Sets the control signal
	/*!
	 *
	 * @param u The fixed control signal
	 */
	void setControl(const ControlVector<CONTROL_DIM, SCALAR>& u)
	{
		u_ = u;
	}

	//! Get the fixed control signal
	/*!
	 *
	 * @param u The control input to write the signal to.
	 */
	void getControl(ControlVector<CONTROL_DIM, SCALAR>& u) const
	{
		u = u_;
	}


private:
	ControlVector<CONTROL_DIM, SCALAR> u_;
};

}
}

#endif /* CT_CORE_CONSTANT_CONTROLLER_ */
