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

#include <ct/core/types/trajectories/TimeArray.h>
#include <ct/core/types/trajectories/StateVectorArray.h>

namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
class IntegratorBase;

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

	friend class IntegratorBase<STATE_DIM, SCALAR>;

	typedef std::vector<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>, Eigen::aligned_allocator<std::shared_ptr<EventHandler<STATE_DIM, SCALAR>>>> EventHandlerPtrVector;

	//! default constructor
	/*!
	 * @param eventHandlers vector of event handlers
	 */
	Observer(const EventHandlerPtrVector& eventHandlers) :
		observeWrap([this](const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t){this->observe(x,t); })
	{
		// fixme: somehow works if using assignment operator, but not if using constructing
		eventHandlers_ = eventHandlers;
	}

	//! reset the observer
	void reset() {
		stateTrajectory_.clear();
		timeTrajectory_.clear();

		for(size_t i = 0; i<eventHandlers_.size(); i++)
			eventHandlers_[i]->reset();
	}

	//! call observer
	/*!
	 * Calls the observer with a given state and time. Records state and time.
	 * @param x current state
	 * @param t current time
	 */
	void observe(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)
	{
		if (timeTrajectory_.size() > 0 && t <= timeTrajectory_.back())
		{
			std::cout << "Integrator Observer: Trying to add time " << t<<", which is smaller or equal than last one "<<timeTrajectory_.back() << std::endl;
			throw std::runtime_error("observations out of order");
		}

		stateTrajectory_.push_back(x);
		timeTrajectory_.push_back(t);

		for(size_t i = 0; i< eventHandlers_.size(); i++){
			if(eventHandlers_[i]->checkEvent(x, t))
				eventHandlers_[i]->handleEvent(x, t);
		}
	}

	//! Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
	std::function<void (const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)> observeWrap;

private:
	EventHandlerPtrVector eventHandlers_; //! list of event handlers

	StateVectorArray<STATE_DIM, SCALAR> stateTrajectory_; //! state trajectory for recording
	tpl::TimeArray<SCALAR> timeTrajectory_; //! time trajectory for recording

};

}
}


#endif /* OBSERVER_H_ */
