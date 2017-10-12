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

#include "../Controller.h"

namespace ct {
namespace core {

//! An array of SISO controllers
/*!
 * By default CT assumes that a ControllerBase maps an N-dimensional state
 * to an M-dimensional control input. This class wraps SISOControllers (which
 * in fact is more a MISO controller) to an array of M controllers. This allows
 * for using SISOControllers as Controllers for ControlledSystems.
 *
 * \warning Assumes that the i-th state of the state vector is the state for the
 * i-th SISO controller
 *
 * @tparam STATE_DIM Defines the dimensionality of the state vector
 * @tparam CONTROL_DIM Defines the dimensionality of the control vector
 * @tparam SISOController Any SISO controller. Needs to have subclasses setpoint_t and parameters_t
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class SISOController>
class SISOArrayController : public Controller<STATE_DIM, CONTROL_DIM>
{
public:

	static_assert ( STATE_DIM >= CONTROL_DIM , "STATE_DIM must be greater or equal to CONTROL_DIM" );

	//! default constructor
	/*!
	 * Assigns the same parameters and setpoints to all controllers
	 * @param parameters the parameters for the SISO controller
	 * @param setpoint the setpoints for the SISO controller
	 */
	SISOArrayController(
			typename SISOController::parameters_t parameters = typename SISOController::parameters_t(),
			typename SISOController::setpoint_t setpoint = typename SISOController::setpoint_t()):
		controllers_(CONTROL_DIM, parameters, setpoint)
	{}

	//! constructor
	/*!
	 * Constructor with different sets of parameters and setpoints for all controllers
	 *
	 * \warning Throws an exception if length of parameters or setpoints is incorrect
	 *
	 * @param parameters vector of parameters
	 * @param setpoints vector of setpoints
	 */
	SISOArrayController(
			std::vector<typename SISOController::parameters_t> parameters,
			std::vector<typename SISOController::setpoint_t> setpoints)
	{
		if (parameters.size() != CONTROL_DIM) {
			throw std::runtime_error("Length of parameters for SISO controllers in SISOArrayController does not match control dimension.");
		}

		if (setpoints.size() != CONTROL_DIM) {
					throw std::runtime_error("Length of setponits for SISO controllers in SISOArrayController does not match control dimension.");
		}

		for (size_t i=0; i<CONTROL_DIM; i++)
		{
			controllers_.push_back(SISOController(parameters[i], setpoints[i]));
		}
	}

	//! copy constructor
	/*!
	 * @param arg the object to copy
	 */
	SISOArrayController(const SISOArrayController& arg)
	{
		for (size_t i=0; i<CONTROL_DIM; i++)
		{
			controllers_.push_back(SISOController(arg.controllers_[i]));
		}
	}

	//! destructor
	~SISOArrayController() {}


	//! deep cloning, required for cloning ControlledSystems
	SISOArrayController<STATE_DIM, CONTROL_DIM, SISOController>* clone() const
	{
		return new SISOArrayController(*this);
	}


	//! computes the control signal
	/*!
	 * Computes the control signal given a current state and time
	 *
	 * \warning The i-th state entry is fed to the i-th SISO controller which
	 * generates the i-th control output
	 *
	 * @param state the current state
	 * @param t the current time
	 * @param controlAction the resulting control action
	 */
	void computeControl(
			const StateVector<STATE_DIM>& state,
			const Time& t,
			ControlVector<CONTROL_DIM>& controlAction) override
	{
		for (size_t i=0; i<CONTROL_DIM; i++)
		{
			controlAction(i) = controllers_[i].computeControl(state(i), t);
		}
	}


private:
	std::vector<SISOController> controllers_; //! array of controllers
};

} // core
} // ct

