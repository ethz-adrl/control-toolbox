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

namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
Observer<STATE_DIM, SCALAR>::Observer(const EventHandlerPtrVector& eventHandlers) :
		observeWrap([this](const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t){this->observe(x,t); }),
		observeWrapWithLogging([this](const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t){ this->log(x,t); this->observe(x,t); })
	{
		// fixme: somehow works if using assignment operator, but not if using constructing
		eventHandlers_ = eventHandlers;
	}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::reset()
{
		for(size_t i = 0; i<eventHandlers_.size(); i++)
			eventHandlers_[i]->reset();

		states_.clear();
		times_.clear();
	}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::observe(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)
	{
		for(size_t i = 0; i< eventHandlers_.size(); i++){
			if(!eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
				eventHandlers_[i]->handleEvent(x, t);
		}
	}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::log(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)
	{
		states_.push_back(x);
		times_.push_back(t);
	}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::observeInternal(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)
	{
		for(size_t i = 0; i< eventHandlers_.size(); i++){
			if(eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
				eventHandlers_[i]->handleEvent(x, t);
		}
	}

}
}


