/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
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
        oscillators_[i] = ct::core::tpl::SecondOrderSystem<SCALAR>((SCALAR)w_n[i], (SCALAR)zeta[i], (SCALAR)(w_n[i] * w_n[i]));
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

        oscillators_[i].computeControlledDynamics(secondOrderState, SCALAR(0.0), inputCtrl, secondOrderStateDerivative);

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
