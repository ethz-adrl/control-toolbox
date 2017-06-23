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


#ifndef CT_CORE_EVENTHANDLER_H_
#define CT_CORE_EVENTHANDLER_H_

#include <ct/core/types/StateVector.h>

namespace ct {
namespace core {

//! Interface for an event handler for an Integrator
/*!
 * An Integrator can call an EventHandler after each integration step to check or log
 * current states etc. This is useful for terminating integrators or checking for events
 * such as system dynamic switches in hybrid systems.
 *
 * Derive from this class to implement your custom event handler.
 *
 * @tparam STATE_DIM dimensionality of the state vector
 */
template <size_t STATE_DIM, typename SCALAR = double>
class EventHandler
{
public:
	//! Default constructor
	EventHandler() {}

	//! destructor
	virtual ~EventHandler() {}

	//! reset event handler
	virtual void reset() = 0;

	//! check if an event has happened
	/*!
	 * checks if an event has happened and whether handleEvent() needs to be called
	 * @param state current state of the system
	 * @param t current time
	 * @return true if an event has happened
	 */
	virtual bool checkEvent(const StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t) = 0;

	//! handle the event
	/*!
	 * does something with an event that just occurred
	 * @param state current state of the system
	 * @param t current time
	 */
	virtual void handleEvent(const StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t) = 0;

private:
};

}
}

#endif /* EVENTHANDLER_H_ */
