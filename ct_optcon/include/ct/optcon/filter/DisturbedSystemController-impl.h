/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::DisturbedSystemController(
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
    : controller_(controller)
{
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::DisturbedSystemController(
    const DisturbedSystemController& other)
    : controller_(other.controller_->clone())
{
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>*
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::clone() const
{
    return new DisturbedSystemController(*this);
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
void DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::computeControl(
    const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
    const SCALAR& t,
    ct::core::ControlVector<CONTROL_DIM, SCALAR>& controlAction)
{
    if (!controller_)
        throw std::runtime_error("Controller not set!");
    controller_->computeControl(state.head(STATE_DIM), t, controlAction);
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
ct::core::ControlMatrix<CONTROL_DIM, SCALAR>
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::getDerivativeU0(
    const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
    const SCALAR time)
{
    if (!controller_)
        throw std::runtime_error("Controller not set!");
    return controller_->getDerivativeU0(state.head(STATE_DIM), time);
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
ct::core::ControlMatrix<CONTROL_DIM, SCALAR>
DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::getDerivativeUf(
    const ct::core::StateVector<AUGMENTED_DIM, SCALAR>& state,
    const SCALAR time)
{
    if (!controller_)
        throw std::runtime_error("Controller not set!");
    return controller_->getDerivativeUf(state.head(STATE_DIM), time);
}

template <size_t STATE_DIM, size_t DIST_DIM, size_t CONTROL_DIM, typename SCALAR>
void DisturbedSystemController<STATE_DIM, DIST_DIM, CONTROL_DIM, SCALAR>::setController(
    std::shared_ptr<ct::core::Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller)
{
    controller_ = controller;
}

}  // optcon
}  // ct
