
/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
InputDisturbedSystem<STATE_DIM, CONTROL_DIM, SCALAR>::InputDisturbedSystem(
    std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> sys)
    : Base(sys->getController()), system_(sys)
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
InputDisturbedSystem<STATE_DIM, CONTROL_DIM, SCALAR>::InputDisturbedSystem(const InputDisturbedSystem& other)
    : Base(*this), system_(other.system_->clone())
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
InputDisturbedSystem<STATE_DIM, CONTROL_DIM, SCALAR>* InputDisturbedSystem<STATE_DIM, CONTROL_DIM, SCALAR>::clone()
    const
{
    return new InputDisturbedSystem(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
void InputDisturbedSystem<STATE_DIM, CONTROL_DIM, SCALAR>::computeControlledDynamics(
    const ct::core::StateVector<AUGMENTED_STATE_DIM, SCALAR>& state,
    const SCALAR& t,
    const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
    ct::core::StateVector<AUGMENTED_STATE_DIM, SCALAR>& derivative)
{
    // the derivative of the original, non-augmented system
    ct::core::StateVector<STATE_DIM, SCALAR> tempDerivative;

    // the control consists of actual commanded control plus the estimated input disturbance,
    // which is the augmented part of the state vector
    ct::core::ControlVector<CONTROL_DIM, SCALAR> disturbed_control = control + state.template tail<CONTROL_DIM>();

    // the dynamics of the augmented system consist of the original dynamics ...
    system_->computeControlledDynamics(state.template head<STATE_DIM>(), t, disturbed_control, tempDerivative);
    derivative.template head<STATE_DIM>() = tempDerivative;

    // and the disturbance dynamics, which is zero as the disturbance is assumed constant.
    derivative.template tail<CONTROL_DIM>().setZero();
}

}  // namespace optcon
}  // namespace ct