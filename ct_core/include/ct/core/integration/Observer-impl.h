/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD, typename SCALAR>
Observer<MANIFOLD, SCALAR>::Observer(const EventHandlerPtrVector& eventHandlers)
    : observeWrap([this](const MANIFOLD& x, const SCALAR& t) { this->observe(x, t); }),
      observeWrapWithLogging([this](const MANIFOLD& x, const SCALAR& t) {
          this->log(x, t);
          this->observe(x, t);
      })
{
    // fixme: somehow works if using assignment operator, but not if using constructing
    eventHandlers_ = eventHandlers;
}

template <typename MANIFOLD, typename SCALAR>
void Observer<MANIFOLD, SCALAR>::reset()
{
    for (size_t i = 0; i < eventHandlers_.size(); i++)
        eventHandlers_[i]->reset();

    states_.clear();
    times_.clear();
}

template <typename MANIFOLD, typename SCALAR>
void Observer<MANIFOLD, SCALAR>::observe(const MANIFOLD& x, const SCALAR& t)
{
    for (size_t i = 0; i < eventHandlers_.size(); i++)
    {
        if (!eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
            eventHandlers_[i]->handleEvent(x, t);
    }
}

template <typename MANIFOLD, typename SCALAR>
void Observer<MANIFOLD, SCALAR>::log(const MANIFOLD& x, const SCALAR& t)
{
    states_.push_back(x);
    times_.push_back(t);
}

template <typename MANIFOLD, typename SCALAR>
void Observer<MANIFOLD, SCALAR>::observeInternal(const MANIFOLD& x, const SCALAR& t)
{
    for (size_t i = 0; i < eventHandlers_.size(); i++)
    {
        if (eventHandlers_[i]->callOnSubsteps() && eventHandlers_[i]->checkEvent(x, t))
            eventHandlers_[i]->handleEvent(x, t);
    }
}
}
}
