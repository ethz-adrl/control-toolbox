/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::CTSystemModel(
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
    std::shared_ptr<SensitivityApprox_t> sensApprox,
    const state_matrix_t& dFdv,
    const ct::core::IntegrationType& intType)
    : system_(system),
      constantController_(new ct::core::ConstantController<STATE_DIM, CONTROL_DIM, SCALAR>()),
      sensApprox_(sensApprox),
      dFdv_(dFdv),
      integrator_(system_, intType)
{
    if (!system_)
        throw std::runtime_error("CTSystemModel: System not initialized!");

    // hand over constant controller for dynamics evaluation with known control inputs to the system.
    system_->setController(constantController_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDynamics(const state_vector_t& state,
    const control_vector_t& u,
    const Time_t dt,
    Time_t t) -> state_vector_t
{
    constantController_->setControl(u);
    state_vector_t x = state;
    integrator_.integrate_n_steps(x, t, 1, dt);
    return x;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeState(const state_vector_t& state,
    const control_vector_t& u,
    const Time_t dt,
    Time_t t) -> state_matrix_t
{
    // local vars
    state_matrix_t A;
    ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> Btemp;

    sensApprox_->setTimeDiscretization(dt);

    sensApprox_->getAandB(state, u, state, int(t / dt), 1, A, Btemp);
    return A;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
auto CTSystemModel<STATE_DIM, CONTROL_DIM, SCALAR>::computeDerivativeNoise(const state_vector_t& state,
    const control_vector_t& control,
    const Time_t dt,
    Time_t t) -> state_matrix_t
{
    return dFdv_;
}

}  // namespace optcon
}  // namespace ct
