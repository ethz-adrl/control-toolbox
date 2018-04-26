/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::CTSystemModel(
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
    const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>& sensApprox,
    double dt,
    unsigned numSubsteps,
    const typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t& dFdv,
    const ct::core::IntegrationType& intType)
    : system_(system),
      sensApprox_(sensApprox),
      dt_(dt),
      dFdv_(dFdv),
      integrator_(system_, intType),
      numSubsteps_(numSubsteps)
{
    if (!system_ && !system_->getController())
        throw std::runtime_error("System or controller not initialized!");
    const double dtNormalized = dt_ / (numSubsteps_ + 1);
    sensApprox_.setTimeDiscretization(dtNormalized);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t
CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDynamics(
    const typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t& state,
    const control_vector_t& controlPlaceholder,
    Time_t t)
{
    typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t x = state;
    integrator_.integrate_n_steps(x, t, numSubsteps_ + 1, dt_ / (numSubsteps_ + 1));
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeState(
    const typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t& state,
    const control_vector_t& controlPlaceholder,
    Time_t t)
{
    control_vector_t control;
    if (!system_->getController())
        throw std::runtime_error("Controller not initialized!");
    system_->getController()->computeControl(state, t, control);
    const typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t xNext =
        CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t::Zero();
    sensApprox_.getAandB(state, control, xNext, int(t / dt_ * (numSubsteps_ + 1) + 0.5), numSubsteps_ + 1, A_, B_);
    return A_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_matrix_t
CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeNoise(
    const typename CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::state_vector_t& state,
    const control_vector_t& control,
    Time_t t)
{
    return dFdv_;
}

}  // optcon
}  // ct
