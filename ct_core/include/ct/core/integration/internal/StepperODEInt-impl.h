/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace internal {

template <class STEPPER, typename MANIFOLD>
StepperODEInt<STEPPER, MANIFOLD>::StepperODEInt()
{
}

template <class STEPPER, typename MANIFOLD>
StepperODEInt<STEPPER, MANIFOLD>::~StepperODEInt()
{
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_n_steps(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_n_steps(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    using namespace ct::core::internal;
    boost::numeric::odeint::integrate_n_steps(stepper_, rhs, state, startTime, dt, numSteps, observer);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_const(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt)
{
    boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_const(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt)
{
    boost::numeric::odeint::integrate_const(stepper_, rhs, state, startTime, finalTime, dt, observer);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_adaptive(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_adaptive(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_adaptive(stepper_, rhs, state, startTime, finalTime, dtInitial, observer);
}

template <class STEPPER, typename MANIFOLD>
void StepperODEInt<STEPPER, MANIFOLD>::integrate_times(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    SCALAR dtInitial)
{
    boost::numeric::odeint::integrate_times(
        stepper_, rhs, state, &timeTrajectory.front(), &timeTrajectory.back() + 1, dtInitial, observer);
}

}  // namespace internal
}  // namespace core
}  // namespace ct
