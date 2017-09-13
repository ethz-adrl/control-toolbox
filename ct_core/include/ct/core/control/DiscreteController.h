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

#ifndef INCLUDE_CT_CORE_CONTROL_DISCRETECONTROLLER_H_
#define INCLUDE_CT_CORE_CONTROL_DISCRETECONTROLLER_H_

#include <ct/core/types/StateVector.h>
#include <ct/core/types/ControlVector.h>


namespace ct {
namespace core {

//! Interface class for all controllers
/*!
 * This is a pure interface class for Controllers that can be fed to any
 * ControlledSystem. Any custom controller should derive from this class
 * to ensure it is compatible with ControlledSystem and the Integrator.
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class DiscreteController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//! Default constructor
	DiscreteController() {};

	//! Copy constructor
	DiscreteController(const DiscreteController& other) {};

	//! Destructor
	virtual ~DiscreteController() {};

	//! Deep cloning
	/*!
	 * Has to be implemented by any custom controller.
	 */
	virtual DiscreteController* clone() const = 0;

	//! Compute control signal
	/*!
	 * Evaluate the given controller for a given state and time index
	 * returns the computed control action.
	 *
	 * This function has to be implemented by any custom controller
	 *
	 * @param state current state of the system
	 * @param n current time index of the system
	 * @param controlAction the corresponding control action
	 */
	virtual void computeControl(
			const StateVector<STATE_DIM, SCALAR>& state,
			const int& n,
			ControlVector<CONTROL_DIM, SCALAR>& controlAction) = 0;
};

}  // namespace core
}  // namespace ct



#endif /* INCLUDE_CT_CORE_CONTROL_DISCRETECONTROLLER_H_ */
