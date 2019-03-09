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
    const state_matrix_t& dFdv,
    const ct::core::IntegrationType& intType)
    : system_(system->clone()),
      constantController_(new ct::core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>()),
      sensApprox_(sensApprox),
      dt_(dt),
      dFdv_(dFdv),
      integrator_(system_, intType),
      numSubsteps_(numSubsteps)
{
    if (!system_)
        throw std::runtime_error("System not initialized!");

    // hand over constant controller for dynamics evaluation with known control inputs to the system.
    system_->setController(constantController_);

    const double dtNormalized = dt_ / (numSubsteps_ + 1);
    sensApprox_.setTimeDiscretization(dtNormalized);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDynamics(const state_vector_t& state,
    const control_vector_t& u,
    Time_t t) -> state_vector_t
{
    constantController_->setControl(u);
    state_vector_t x = state;
    integrator_.integrate_n_steps(x, t, numSubsteps_ + 1, dt_ / (numSubsteps_ + 1));
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeState(const state_vector_t& state,
    const control_vector_t& u,
    Time_t t) -> state_matrix_t
{
    sensApprox_.getAandB(state, u, state, int(t / dt_ * (numSubsteps_ + 1) + 0.5), numSubsteps_ + 1, A_, B_);
    return A_;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeNoise(const state_vector_t& state,
    const control_vector_t& control,
    Time_t t) -> state_matrix_t
{
    return dFdv_;
}

}  // namespace optcon
}  // namespace ct
