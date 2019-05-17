/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once


namespace ct {
namespace core {

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::IntegratorSymplectic(
    const std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> system,
    const EventHandlerPtrVector& eventHandlers)
    : systemSymplectic_(system), observer_(eventHandlers)
{
    setupSystem();
}

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::IntegratorSymplectic(
    const std::shared_ptr<SymplecticSystem<POS_DIM, VEL_DIM, CONTROL_DIM, SCALAR>> system,
    const EventHandlerPtr& eventHandler)
    : systemSymplectic_(system), observer_(EventHandlerPtrVector(1, eventHandler))
{
    setupSystem();
}


template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
void IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::integrate_n_steps(
    StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt,
    StateVectorArray<POS_DIM + VEL_DIM, SCALAR>& stateTrajectory,
    tpl::TimeArray<SCALAR>& timeTrajectory)
{
    pair_t xPair;
    xPair.first = state.head(POS_DIM);
    xPair.second = state.tail(VEL_DIM);
    SCALAR time = startTime;
    stateTrajectory.push_back(state);
    timeTrajectory.push_back(time);

    for (size_t i = 0; i < numSteps; ++i)
    {
        xCached_ = state;
        stepper_.do_step(std::make_pair(systemFunctionPosition_, systemFunctionVelocity_), xPair, time, dt);
        state.head(POS_DIM) = xPair.first;
        state.tail(VEL_DIM) = xPair.second;
        observer_.observeInternal(state, time);
        time += dt;
        stateTrajectory.push_back(state);
        timeTrajectory.push_back(time);
    }
}

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
void IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::integrate_n_steps(
    StateVector<POS_DIM + VEL_DIM, SCALAR>& state,
    const SCALAR& startTime,
    size_t numSteps,
    SCALAR dt)
{
    pair_t xPair;
    xPair.first = state.head(POS_DIM);
    xPair.second = state.tail(VEL_DIM);
    SCALAR time = startTime;
    for (size_t i = 0; i < numSteps; ++i)
    {
        xCached_ = state;
        stepper_.do_step(std::make_pair(systemFunctionPosition_, systemFunctionVelocity_), xPair, time, dt);
        state.head(POS_DIM) = xPair.first;
        state.tail(VEL_DIM) = xPair.second;
        observer_.observeInternal(state, time);
        time += dt;
    }
}

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
void IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::reset()
{
    observer_.reset();
}

template <size_t POS_DIM, size_t VEL_DIM, size_t CONTROL_DIM, class Stepper, typename SCALAR>
void IntegratorSymplectic<POS_DIM, VEL_DIM, CONTROL_DIM, Stepper, SCALAR>::setupSystem()
{
    systemFunctionPosition_ = [this](
        const Eigen::Matrix<SCALAR, VEL_DIM, 1>& v, Eigen::Matrix<SCALAR, POS_DIM, 1>& dxdt) {
        const StateVector<POS_DIM, SCALAR>& vState(static_cast<const StateVector<POS_DIM, SCALAR>&>(v));
        StateVector<POS_DIM, SCALAR>& dxdtState(static_cast<StateVector<POS_DIM, SCALAR>&>(dxdt));
        xCached_.template bottomRows<VEL_DIM>() = v;
        systemSymplectic_->computePdot(xCached_, vState, dxdtState);
    };

    systemFunctionVelocity_ = [this](
        const Eigen::Matrix<SCALAR, POS_DIM, 1>& x, Eigen::Matrix<SCALAR, VEL_DIM, 1>& dvdt) {
        const StateVector<VEL_DIM, SCALAR>& xState(static_cast<const StateVector<VEL_DIM, SCALAR>&>(x));
        StateVector<VEL_DIM, SCALAR>& dvdtState(static_cast<StateVector<VEL_DIM, SCALAR>&>(dvdt));
        xCached_.template topRows<POS_DIM>() = x;
        systemSymplectic_->computeVdot(xCached_, xState, dvdtState);
    };
}
}
}
