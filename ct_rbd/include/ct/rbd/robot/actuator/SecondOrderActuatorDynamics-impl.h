/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#pragma once

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(SCALAR w_n, SCALAR zeta, SCALAR g_dc)
    : oscillator_(w_n, zeta, g_dc)
{
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
void SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computePdot(const typename BASE::act_state_vector_t& x,
    const typename BASE::act_vel_vector_t& v,
    const ct::core::ControlVector<NJOINTS, SCALAR>& control,
    typename BASE::act_pos_vector_t& pDot)
{
    // as the oscillator is symplectic itself, we simply transcribe the velocity coordinates
    pDot = v;
}

template <size_t NJOINTS, typename SCALAR>
void SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeVdot(const typename BASE::act_state_vector_t& x,
    const typename BASE::act_pos_vector_t& p,
    const ct::core::ControlVector<NJOINTS, SCALAR>& control,
    typename BASE::act_vel_vector_t& vDot)
{
    // evaluate oscillator dynamics for each joint
    for (size_t i = 0; i < NJOINTS; i++)
    {
        core::StateVector<2, SCALAR> secondOrderState;
        core::StateVector<2, SCALAR> secondOrderStateDerivative;
        core::ControlVector<1, SCALAR> inputCtrl;
        inputCtrl(0) = control(i);

        secondOrderState << p(i), x(i + NJOINTS);

        oscillator_.computeControlledDynamics(secondOrderState, SCALAR(0.0), inputCtrl, secondOrderStateDerivative);

        vDot(i) = secondOrderStateDerivative(1);
    }
}

template <size_t NJOINTS, typename SCALAR>
core::ControlVector<NJOINTS, SCALAR> SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeControlOutput(
    const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
    const typename BASE::act_state_vector_t& actState)
{
    // for this simple actuator dynamics model, the controlOutput is just the "position" coordinates of the actuator state
    return actState.template topRows<NJOINTS>();
}
}
}
