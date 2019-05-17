/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(double w_n, double zeta)
    : oscillators_(NJOINTS, ct::core::tpl::SecondOrderSystem<SCALAR>((SCALAR)w_n, (SCALAR)zeta, (SCALAR)(w_n * w_n)))
{
}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(double w_n, double zeta, double g_dc)
    : oscillators_(NJOINTS, ct::core::tpl::SecondOrderSystem<SCALAR>((SCALAR)w_n, (SCALAR)zeta, (SCALAR)g_dc))
{
}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(std::vector<double> w_n,
    std::vector<double> zeta)
{
    if ((w_n.size() != NJOINTS) || (zeta.size() != NJOINTS))
        throw std::runtime_error("SecondOrderActuatorDynamics: w_n or zeta is not correct size");

    oscillators_.resize(NJOINTS);
    for (size_t i = 0; i < NJOINTS; i++)
        oscillators_[i] =
            ct::core::tpl::SecondOrderSystem<SCALAR>((SCALAR)w_n[i], (SCALAR)zeta[i], (SCALAR)(w_n[i] * w_n[i]));
}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(std::vector<double> w_n,
    std::vector<double> zeta,
    std::vector<double> g_dc)
{
    if ((w_n.size() != NJOINTS) || (zeta.size() != NJOINTS) || (g_dc.size() != NJOINTS))
        throw std::runtime_error("SecondOrderActuatorDynamics: w_n, zeta or g_dc is not correct size");

    oscillators_.resize(NJOINTS);
    for (size_t i = 0; i < NJOINTS; i++)
        oscillators_[i] = ct::core::tpl::SecondOrderSystem<SCALAR>((SCALAR)w_n[i], (SCALAR)zeta[i], (SCALAR)g_dc[i]);
}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::~SecondOrderActuatorDynamics()
{
}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>* SecondOrderActuatorDynamics<NJOINTS, SCALAR>::clone() const
{
    return new SecondOrderActuatorDynamics(*this);
}

template <size_t NJOINTS, typename SCALAR>
void SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeActuatorDynamics(
    const JointState<NJOINTS, SCALAR>& robotJointState,
    const ct::core::StateVector<2 * NJOINTS, SCALAR>& state,
    const SCALAR& t,
    const ct::core::ControlVector<NJOINTS, SCALAR>& control,
    ct::core::StateVector<2 * NJOINTS, SCALAR>& derivative)
{
    // evaluate oscillator dynamics for each joint
    for (size_t i = 0; i < NJOINTS; i++)
    {
        core::StateVector<2, SCALAR> secondOrderState;
        core::StateVector<2, SCALAR> secondOrderStateDerivative;
        core::ControlVector<1, SCALAR> inputCtrl;
        inputCtrl(0) = control(i);

        secondOrderState << state(i), state(i + NJOINTS);

        oscillators_[i].computeControlledDynamics(secondOrderState, SCALAR(0.0), inputCtrl, secondOrderStateDerivative);

        derivative(i) = state(i + NJOINTS);
        derivative(i + NJOINTS) = secondOrderStateDerivative(1);
    }
}

template <size_t NJOINTS, typename SCALAR>
core::ControlVector<NJOINTS, SCALAR> SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeControlOutput(
    const JointState<NJOINTS, SCALAR>& robotJointState,
    const typename BASE::act_state_vector_t& actState)
{
    // for this simple actuator dynamics model, the controlOutput is just the "position" coordinates of the actuator state
    return actState.template topRows<NJOINTS>();
}

template <size_t NJOINTS, typename SCALAR>
ct::core::StateVector<2 * NJOINTS, SCALAR> SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeStateFromOutput(
    const JointState<NJOINTS, SCALAR>& refRobotJointState,
    const core::ControlVector<NJOINTS, SCALAR>& refControl)
{
    ct::core::StateVector<2 * NJOINTS, SCALAR> refState;
    refState.setZero();
    refState.template topRows<NJOINTS>() = refControl.toImplementation();
    return refState;
}

}  // namespace rbd
}  // namespace ct
