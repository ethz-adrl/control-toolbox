/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


namespace ct {
namespace core {

template <size_t STATE_DIM, typename SCALAR>
Integrator<STATE_DIM, SCALAR>::Integrator(const std::shared_ptr<System<STATE_DIM, SCALAR>>& system,
    const IntegrationType& intType,
    const EventHandlerPtrVector& eventHandlers)
    : system_(system), observer_(eventHandlers)
{
    changeIntegrationType(intType);
    setupSystem();
}

template <size_t STATE_DIM, typename SCALAR>
Integrator<STATE_DIM, SCALAR>::Integrator(const std::shared_ptr<System<STATE_DIM, SCALAR>>& system,
    const IntegrationType& intType,
    const EventHandlerPtr& eventHandler)
    : system_(system), observer_(EventHandlerPtrVector(1, eventHandler))
{
    changeIntegrationType(intType);
    setupSystem();
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::changeIntegrationType(const IntegrationType& intType)
{
    initializeCTSteppers(intType);
    initializeAdaptiveSteppers(intType);
    initializeODEIntSteppers(intType);
    if (!integratorStepper_)
        throw std::runtime_error("Unknown integration type");
}


template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::setApadativeErrorTolerances(const SCALAR absErrTol, const SCALAR& relErrTol)
{
    integratorStepper_->setAdaptiveErrorTolerances(absErrTol, relErrTol);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_n_steps(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt,
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    reset();
    integratorStepper_->integrate_n_steps(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, numSteps, dt);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_n_steps(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    reset();
    integratorStepper_->integrate_n_steps(observer_.observeWrap, systemFunction_, state, startTime, numSteps, dt);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_const(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt,
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    reset();
    integratorStepper_->integrate_const(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, finalTime, dt);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_const(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dt)
{
    reset();
    integratorStepper_->integrate_const(observer_.observeWrap, systemFunction_, state, startTime, finalTime, dt);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_adaptive(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory,
    const SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_adaptive(
        observer_.observeWrapWithLogging, systemFunction_, state, startTime, finalTime, dtInitial);
    retrieveTrajectoriesFromObserver(stateTrajectory, timeTrajectory);
    state = stateTrajectory.back();
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_adaptive(StateVector<STATE_DIM, SCALAR>& state,
    const SCALAR& startTime,
    const SCALAR& finalTime,
    SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_adaptive(
        observer_.observeWrap, systemFunction_, state, startTime, finalTime, dtInitial);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::integrate_times(StateVector<STATE_DIM, SCALAR>& state,
    const tpl::TimeArray<SCALAR>& timeTrajectory,
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
    SCALAR dtInitial)
{
    reset();
    integratorStepper_->integrate_times(
        observer_.observeWrapWithLogging, systemFunction_, state, timeTrajectory, dtInitial);
    retrieveStateVectorArrayFromObserver(stateTrajectory);
}


template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::initializeCTSteppers(const IntegrationType& intType)
{
    switch (intType)
    {
        case EULERCT:
        {
            integratorStepper_ = std::shared_ptr<internal::StepperEulerCT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
                new internal::StepperEulerCT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
            break;
        }

        case RK4CT:
        {
            integratorStepper_ = std::shared_ptr<internal::StepperRK4CT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>>(
                new internal::StepperRK4CT<Eigen::Matrix<SCALAR, STATE_DIM, 1>, SCALAR>());
            break;
        }

        default:
            break;
    }
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::reset()
{
    observer_.reset();
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::retrieveTrajectoriesFromObserver(
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    stateTrajectory.swap(observer_.states_);
    timeTrajectory.swap(observer_.times_);
}

template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::retrieveStateVectorArrayFromObserver(
    StateVectorArray<STATE_DIM, SCALAR>& stateTrajectory)
{
    stateTrajectory.swap(observer_.states_);
}


template <size_t STATE_DIM, typename SCALAR>
void Integrator<STATE_DIM, SCALAR>::setupSystem()
{
    systemFunction_ = [this](
        const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x, Eigen::Matrix<SCALAR, STATE_DIM, 1>& dxdt, SCALAR t) {
        const StateVector<STATE_DIM, SCALAR>& xState(static_cast<const StateVector<STATE_DIM, SCALAR>&>(x));
        StateVector<STATE_DIM, SCALAR>& dxdtState(static_cast<StateVector<STATE_DIM, SCALAR>&>(dxdt));
        system_->computeDynamics(xState, t, dxdtState);
        observer_.observeInternal(xState, t);
    };

    reset();
}
}
}
