/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {


template <typename MANIFOLD>
MaxStepsEventHandler<MANIFOLD>::MaxStepsEventHandler(const size_t& maxStepsPerSec)
    : maxNumSteps_(maxStepsPerSec), stepsTaken_(0)
{
}

template <typename MANIFOLD>
MaxStepsEventHandler<MANIFOLD>::~MaxStepsEventHandler()
{
}

template <typename MANIFOLD>
bool MaxStepsEventHandler<MANIFOLD>::callOnSubsteps()
{
    return false;
}

template <typename MANIFOLD>
void MaxStepsEventHandler<MANIFOLD>::reset()
{
    stepsTaken_ = 0;
};

template <typename MANIFOLD>
bool MaxStepsEventHandler<MANIFOLD>::checkEvent(const MANIFOLD& state, const double& t)
{
    stepsTaken_++;
    return (stepsTaken_ > maxNumSteps_);  // todo: fix this
}

template <typename MANIFOLD>
void MaxStepsEventHandler<MANIFOLD>::handleEvent(const MANIFOLD& state, const double& t)
{
    throw std::runtime_error("integration terminated: max number of steps reached.\n");
}

template <typename MANIFOLD>
void MaxStepsEventHandler<MANIFOLD>::setMaxNumSteps(size_t maxNumSteps)
{
    maxNumSteps_ = maxNumSteps;
}

}  // namespace core
}  // namespace ct
