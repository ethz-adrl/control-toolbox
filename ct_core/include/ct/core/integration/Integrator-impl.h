/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename MANIFOLD>
Integrator<MANIFOLD>::Integrator(const SystemPtr_t& system,
    const IntegrationType& intType,
    const EventHandlerPtrVector& eventHandlers)
    : system_(system), observer_(eventHandlers)
{
    changeIntegrationType(intType);
    setupSystem();
}

template <typename MANIFOLD>
Integrator<MANIFOLD>::Integrator(const SystemPtr_t& system,
    const IntegrationType& intType,
    const EventHandlerPtr& eventHandler)
    : system_(system), observer_(EventHandlerPtrVector(1, eventHandler))
{
    changeIntegrationType(intType);
    setupSystem();
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::changeIntegrationType(const IntegrationType& intType)
{
    initializeCTSteppers(intType);
    initializeAdaptiveSteppers(intType);
    initializeODEIntSteppers(intType);
    if (!integratorStepper_)
        throw std::runtime_error("Unknown integration type");
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::setApadativeErrorTolerances(const SCALAR absErrTol, const SCALAR& relErrTol)
{
    integratorStepper_->setAdaptiveErrorTolerances(absErrTol, relErrTol);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_n_steps(MANIFOLD& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt,
    DiscreteArray<MANIFOLD>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    reset();
    integratorStepper_->integrate_n_steps(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, numSteps, dt);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_n_steps(MANIFOLD& state, const SCALAR& startTime, size_t numSteps, SCALAR dt)
{
    reset();
    integratorStepper_->integrate_n_steps(observer_.observeWrap, systemFunction_, state, startTime, numSteps, dt);
}


template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_const(MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt,
    DiscreteArray<MANIFOLD>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    reset();
    integratorStepper_->integrate_const(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, finalTime, dt);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_const(MANIFOLD& state, const SCALAR& startTime, const SCALAR& finalTime, SCALAR dt)
{
    reset();
    integratorStepper_->integrate_const(observer_.observeWrap, systemFunction_, state, startTime, finalTime, dt);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_adaptive(MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    DiscreteArray<MANIFOLD>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory,
    const SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_adaptive(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, finalTime, dtInitial);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
    state = stateTrajectory.back();
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_adaptive(MANIFOLD& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_adaptive(
        observer_.observeWrap, systemFunction_, state, startTime, finalTime, dtInitial);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::integrate_times(MANIFOLD& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    DiscreteArray<MANIFOLD>& stateTrajectory,
    SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_times(
        observer_.observeWrapWithLogging, systemFunction_, state, timeTrajectory, dtInitial);
    retrieveStateVectorArrayFromObserver(stateTrajectory);
}


template <typename MANIFOLD>
void Integrator<MANIFOLD>::initializeCTSteppers(const IntegrationType& intType)
{
    switch (intType)
    {
        case EULERCT:
        {
            integratorStepper_ = std::shared_ptr<internal::StepperEulerCT<MANIFOLD>>(
                new internal::StepperEulerCT<MANIFOLD>());
            break;
        }

        case RK4CT:
        {
            integratorStepper_ = std::shared_ptr<internal::StepperRK4CT<MANIFOLD>>(
                new internal::StepperRK4CT<MANIFOLD>());
            break;
        }

        default:
            break;
    }
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::reset()
{
    observer_.reset();
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::retrieveTrajectoriesFromObserver(DiscreteArray<MANIFOLD>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    stateTrajectory.swap(observer_.states_);
    timeTrajectory.swap(observer_.times_);
}

template <typename MANIFOLD>
void Integrator<MANIFOLD>::retrieveStateVectorArrayFromObserver(DiscreteArray<MANIFOLD>& stateTrajectory)
{
    stateTrajectory.swap(observer_.states_);
}


template <typename MANIFOLD>
void Integrator<MANIFOLD>::setupSystem()
{
    systemFunction_ = [this](const MANIFOLD& x, TANGENT& dxdt, SCALAR t) {
        const MANIFOLD& xState(static_cast<const MANIFOLD&>(x));

        TANGENT& dxdtState(static_cast<TANGENT&>(dxdt));

        system_->computeDynamics(xState, t, dxdtState);

        observer_.observeInternal(xState, t);
    };

    reset();
}
}  // namespace core
}  // namespace ct
