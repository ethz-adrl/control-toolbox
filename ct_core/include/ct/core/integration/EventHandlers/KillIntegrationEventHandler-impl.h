/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD>
KillIntegrationEventHandler<MANIFOLD>::KillIntegrationEventHandler() : killIntegration_(false)
{
}

template <typename MANIFOLD>
KillIntegrationEventHandler<MANIFOLD>::~KillIntegrationEventHandler()
{
}

template <typename MANIFOLD>
bool KillIntegrationEventHandler<MANIFOLD>::callOnSubsteps()
{
    return false;
}

template <typename MANIFOLD>
bool KillIntegrationEventHandler<MANIFOLD>::checkEvent(const MANIFOLD& state, const double& t)
{
    return killIntegration_;
}

template <typename MANIFOLD>
void KillIntegrationEventHandler<MANIFOLD>::handleEvent(const MANIFOLD& state, const double& t)
{
    /* throw an exception which stops the integration */
    throw std::runtime_error("Integration terminated due to external event specified by user.");
}

template <typename MANIFOLD>
void KillIntegrationEventHandler<MANIFOLD>::setEvent()
{
    killIntegration_ = true;
}

template <typename MANIFOLD>
void KillIntegrationEventHandler<MANIFOLD>::resetEvent()
{
    killIntegration_ = false;
}

template <typename MANIFOLD>
void KillIntegrationEventHandler<MANIFOLD>::reset()
{
    resetEvent();
};

}  // namespace core
}  // namespace ct
