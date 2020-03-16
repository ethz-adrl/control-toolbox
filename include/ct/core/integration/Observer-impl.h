/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
Observer<STATE_DIM, SCALAR>::Observer(const EventHandlerPtrVector& eventHandlers)
    : observeWrap([this](const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t) { this->observe(x, t); }),
      observeWrapWithLogging([this](const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t) {
          this->log(x, t);
          this->observe(x, t);
      })
{
    // fixme: somehow works if using assignment operator, but not if using constructing
    eventHandlers_ = eventHandlers;
}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::reset()
{
    for (size_t i = 0; i < eventHandlers_.size(); i++)
        eventHandlers_[i]->reset();

    states_.clear();
    times_.clear();
}

template <size_t STATE_DIM, typename SCALAR>
void Observer<STATE_DIM, SCALAR>::observe(const StateVector<STATE_DIM, SCALAR>& x, const SCALAR& t)
{
    for (size_t i = 0; i < eventHandlers_.size(); i++)
    {
        if (!eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
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
    for (size_t i = 0; i < eventHandlers_.size(); i++)
    {
        if (eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
            eventHandlers_[i]->handleEvent(x, t);
    }
}
}
}
