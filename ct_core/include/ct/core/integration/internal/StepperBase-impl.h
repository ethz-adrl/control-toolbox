/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {
namespace internal {

template <typename MANIFOLD>
StepperBase<MANIFOLD>::StepperBase() : absErrTol_(SCALAR(1e-8)), relErrTol_(SCALAR(1e-8))
{
}

template <typename MANIFOLD>
StepperBase<MANIFOLD>::~StepperBase()
{
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_n_steps(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_n_steps(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    throw std::runtime_error("Integrate_n_steps not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_const(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt)
{
    throw std::runtime_error("integrate_const not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_const(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt)
{
    throw std::runtime_error("integrate_const not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_adaptive(const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    throw std::runtime_error("integrate_adaptive not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_adaptive(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    const SCALAR dtInitial)
{
    throw std::runtime_error("integrate_adaptive not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::integrate_times(ObserverFunction_t observer,
    const SystemFunction_t& rhs,
    MANIFOLD& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    SCALAR dtInitial)
{
    throw std::runtime_error("integrate_times not implemented for the stepper type");
}

template <typename MANIFOLD>
void StepperBase<MANIFOLD>::setAdaptiveErrorTolerances(const SCALAR absErrTol, const SCALAR& relErrTol)
{
    absErrTol_ = absErrTol;
    relErrTol_ = relErrTol;
}

}  // namespace internal
}  // namespace core
}  // namespace ct
