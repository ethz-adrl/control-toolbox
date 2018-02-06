/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "SystemModelBase.h"

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class CTSystemModel : public SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = SystemModelBase<STATE_DIM, CONTROL_DIM, SCALAR>;
    using typename Base::state_vector_t;
    using typename Base::state_matrix_t;
    using typename Base::control_vector_t;
    using typename Base::Time_t;

    CTSystemModel(std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system,
        const ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR>&
            sensApprox,
        double dt,
        unsigned numSubsteps = 0u,
        const state_matrix_t& dFdv = state_matrix_t::Identity(),
        const ct::core::IntegrationType& intType = ct::core::IntegrationType::EULERCT)
        : system_(system),
          sensApprox_(sensApprox),
          dt_(dt),
          dFdv_(dFdv),
          integrator_(system_, intType),
          numSubsteps_(numSubsteps)
    {
        if (!system_ && !system_->getController()) throw std::runtime_error("System or controller not initialized!");
        const double dtNormalized = dt_ / (numSubsteps_ + 1);
        sensApprox_.setTimeDiscretization(dtNormalized);
    }
    state_vector_t computeDynamics(const state_vector_t& state,
        const control_vector_t& controlPlaceholder,
        Time_t t) override
    {
        state_vector_t x = state;
        integrator_.integrate_n_steps(x, t, numSubsteps_ + 1, dt_ / (numSubsteps_ + 1));
        return x;
    }

    state_matrix_t computeDerivativeState(
        const state_vector_t& state,
        const control_vector_t& controlPlaceholder,
        Time_t t) override
    {
        control_vector_t control;
        if (!system_->getController()) throw std::runtime_error("Controller not initialized!");
        system_->getController()->computeControl(state, t, control);
        const state_vector_t xNext = state_vector_t::Zero();
        sensApprox_.getAandB(state, control, xNext, int(t / dt_ * (numSubsteps_ + 1) + 0.5), numSubsteps_ + 1, A_, B_);
        return A_;
    }
    state_matrix_t computeDerivativeNoise(
        const state_vector_t& state,
        const control_vector_t& control,
        Time_t t) override
    {
        return dFdv_;
    }

protected:
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> system_;
    ct::core::SensitivityApproximation<STATE_DIM, CONTROL_DIM, STATE_DIM / 2, STATE_DIM / 2, SCALAR> sensApprox_;
    double dt_;
    state_matrix_t dFdv_;
    ct::core::Integrator<STATE_DIM, SCALAR> integrator_;
    unsigned numSubsteps_;
    state_matrix_t A_;  // dFdx.
    ct::core::StateControlMatrix<STATE_DIM, CONTROL_DIM, SCALAR> B_;
};

}  // optcon
}  // ct
