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

#include <ct/core/integration/EventHandler.h>

namespace ct{
namespace core{

//! Event handler to kill a (variable step) Integrator after doing too many steps
/*!
 * Checks the number of steps that a variable step Integrator has taken and kills it if exceeded
 *
 * @tparam STATE_DIM state vector size
 */
template <size_t STATE_DIM>
class MaxStepsEventHandler : public ct::core::EventHandler<STATE_DIM>
{
public:
	typedef ct::core::StateVector<STATE_DIM> State_T;

	//! default constructor
	/*!
	 * @param maxStepsPerSec allowed number of steps
	 */
	MaxStepsEventHandler(const size_t& maxStepsPerSec = std::numeric_limits<size_t>::max()) :
		maxNumSteps_(maxStepsPerSec),
		stepsTaken_(0)
	{}

	//! destructor
	virtual ~MaxStepsEventHandler() {}

	virtual bool callOnSubsteps() override { return false; }

	//! resets the number of steps taken
	virtual void reset() override {stepsTaken_ = 0; };

	//! checks if number of steps is exceeded
	/*!
	 *
	 * @param state current state (gets ignored)
	 * @param t current time (gets ignored)
	 * @return true if number of steps higher than maximum allowed number
	 */
	virtual bool checkEvent(const State_T& state, const double& t)  override {
		stepsTaken_++;
		return (stepsTaken_ > maxNumSteps_);	// todo: fix this
	}

	//! throws a std::runtime_error to terminate the integration
	virtual void handleEvent(const State_T& state, const double& t) override {
		throw std::runtime_error("integration terminated: max number of steps reached.\n");
	}

	//! set maximum number of steps
	/*!
	 * @param maxNumSteps maximum number of steps allowed
	 */
	void setMaxNumSteps(size_t maxNumSteps) {
		maxNumSteps_ = maxNumSteps;
	}

private:
	size_t maxNumSteps_; //! maximum number of steps allowed
	size_t stepsTaken_; //! counter how many steps have passed
};

} // namespace core
} // namespace ct

