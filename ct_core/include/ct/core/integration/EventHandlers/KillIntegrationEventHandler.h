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

//! Event handler to kill integration
/*!
 * This event handler kills the integration if the kill flag is set. This is useful
 * for multi-threaded applications where an external kill signal needs to interrupt
 * an ongoing integration
 *
 * @tparam STATE_DIM size of the state vector
 */
template <size_t STATE_DIM>
class KillIntegrationEventHandler : public EventHandler<STATE_DIM>
{
public:
	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	//! default constructor
	/*!
	 * sets kill event to false
	 */
	KillIntegrationEventHandler():
		killIntegration_(false)
	{}

	//! default destructor
	virtual ~KillIntegrationEventHandler() {}

	virtual bool callOnSubsteps() override { return false; }

	//! checks the kill flag
	bool checkEvent(const State_T& state, const double& t) override {
		return killIntegration_;
	}

	//! interrupts integration
	/*!
	 * interrupts the integration by throwing a std::runtime_error
	 * @param state current state (ignored)
	 * @param t current time (ignored)
	 */
	void handleEvent(const State_T& state, const double& t) override {

		/* throw an exception which stops the integration */
		throw std::runtime_error("Integration terminated due to external event specified by user.");
	}

	//! enables killing at next call
	void setEvent() {
		killIntegration_ = true;
	}

	//! disable killing at next call
	void resetEvent() {
		killIntegration_ = false;
	}

	//! resets kill flag to false
	virtual void reset() override { resetEvent();};


private:
	bool killIntegration_; //! kill flag
};

} // namespace core
} // namespace ct

