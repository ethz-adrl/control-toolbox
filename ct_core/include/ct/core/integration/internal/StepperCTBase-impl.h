/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace internal {

template <typename MANIFOLD, typename SCALAR>
StepperCTBase<MANIFOLD, SCALAR>::StepperCTBase()
{
}

template <typename MANIFOLD, typename SCALAR>
StepperCTBase<MANIFOLD, SCALAR>::~StepperCTBase()
{
}

template <typename MANIFOLD, typename SCALAR>
void StepperCTBase<MANIFOLD, SCALAR>::integrate_n_steps(
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    SCALAR time = startTime;
    for (size_t i = 0; i < numSteps; ++i)
    {
        do_step(rhs, state, time, dt);
        time += dt;
    }
}

template <typename MANIFOLD, typename SCALAR>
void StepperCTBase<MANIFOLD, SCALAR>::integrate_n_steps(std::function<void(const MANIFOLD& x, const SCALAR& t)> observe,
    const std::function<void(const MANIFOLD&, Tangent&, SCALAR)>& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    SCALAR time = startTime;

    for (size_t i = 0; i < numSteps; ++i)
    {
        do_step(rhs, state, time, dt);
        time += dt;
        observe(state, time);
    }
}

}  // namespace internal
}  // namespace core
}  // namespace ct
