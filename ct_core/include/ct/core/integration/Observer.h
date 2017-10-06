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

#ifndef CT_CORE_OBSERVER_H_
#define CT_CORE_OBSERVER_H_

#include "EventHandler.h"

#include <ct/core/types/arrays/TimeArray.h>
#include <ct/core/types/arrays/MatrixArrays.h>

namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
class Integrator;

//! Observer for Integrator
/*!
 * Implements a general observer as required by boost::odeint. This wraps all event handlers and calls them.
 * Furthermore, this class records state and time trajectories during integration.
 *
 * @tparam STATE_DIM The size of the state vector
 */
template <size_t STATE_DIM, typename SCALAR = double>
class Observer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	friend class Integrator<STATE_DIM, SCALAR>;

	typedef std::vector<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>, Eigen::aligned_allocator<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>>> EventHandlerPtrVector;

	//! default constructor
	/*!
	 * @param eventHandlers vector of event handlers
	 */
	Observer(const EventHandlerPtrVector& eventHandlers);

	//! reset the observer
	void reset();

	void observe(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

	void log(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

	void observeInternal(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t);

private:
	//! Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
	std::function<void (const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)> observeWrap;
	std::function<void (const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)> observeWrapWithLogging;

	ct::core::StateVectorArray<STATE_DIM, SCALAR> states_; //!< container for logging the state
	ct::core::tpl::TimeArray<SCALAR> times_;  //!< container for logging the time

	
	EventHandlerPtrVector eventHandlers_; //! list of event handlers


};

}
}


#endif /* OBSERVER_H_ */
