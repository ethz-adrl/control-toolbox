/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

template <typename SCALAR>
PIDController<SCALAR>::PIDController(const parameters_t& parameters, const setpoint_t& setpoint)
    : statePrevious_(SCALAR(0.0)), I_(SCALAR(0.0)), parameters_(parameters), setpoint_(setpoint)
{
}

template <typename SCALAR>
PIDController<SCALAR>::PIDController(const PIDController& other)
    : statePrevious_(other.statePrevious_), I_(other.I_), parameters_(other.parameters_), setpoint_(other.setpoint_)
{
}

template <typename SCALAR>
PIDController<SCALAR>::~PIDController()
{
}

template <typename SCALAR>
PIDController<SCALAR>* PIDController<SCALAR>::clone() const
{
    return new PIDController<SCALAR>(*this);
}

template <typename SCALAR>
void PIDController<SCALAR>::setInitialState(const SCALAR& state)
{
    statePrevious_ = state;
}

template <typename SCALAR>
void PIDController<SCALAR>::setDesiredState(const SCALAR& state)
{
    setpoint_.stateDesired_ = state;
}

template <typename SCALAR>
void PIDController<SCALAR>::setDesiredState(const SCALAR& state, const SCALAR& stateDerivative)
{
    setpoint_.stateDesired_ = state;
    setpoint_.stateDerivativeDesired_ = stateDerivative;
}

template <typename SCALAR>
SCALAR PIDController<SCALAR>::computeControl(const SCALAR& state, const Time& t)
{
    SCALAR error = setpoint_.stateDesired_ - state;

    // ** P-part **
    // ============
    SCALAR P = parameters_.k_p * error;

    // ** I-part **
    // ============
    computeI(error);

    // ** D-Part **
    // ============
    SCALAR D = parameters_.k_d * (setpoint_.stateDerivativeDesired_ - (state - statePrevious_) / parameters_.dt);

    // ** Controller Output **
    // =======================
    SCALAR u = (P + I_ + D);
    saturateControl(u);

    statePrevious_ = state;

    return u;
}

template <typename SCALAR>
SCALAR PIDController<SCALAR>::computeControl(const SCALAR& state, const SCALAR& stateDerivative, const Time& t)
{
    SCALAR error = setpoint_.stateDesired_ - state;

    // ** P-part **
    // ============
    SCALAR P = parameters_.k_p * error;

    // ** I-part **
    // ============
    computeI(error);

    // ** D-Part **
    // ============
    SCALAR D = parameters_.k_d * (setpoint_.stateDerivativeDesired_ - stateDerivative);

    // ** Controller Output **
    // =======================
    SCALAR u = (P + I_ + D);

    saturateControl(u);

    statePrevious_ = state;

    return u;
}

template <typename SCALAR>
void PIDController<SCALAR>::saturateControl(SCALAR& u)
{
    if (u > parameters_.uMax)
        u = parameters_.uMax;
    if (u < parameters_.uMin)
        u = parameters_.uMin;
}

template <typename SCALAR>
void PIDController<SCALAR>::changeParameters(const typename PIDController<SCALAR>::parameters_t& parameters)
{
    parameters_ = parameters;
}

template <typename SCALAR>
typename PIDController<SCALAR>::parameters_t& PIDController<SCALAR>::Parameters()
{
    return parameters_;
}

template <typename SCALAR>
void PIDController<SCALAR>::reset()
{
    statePrevious_ = SCALAR(0.0);
    I_ = SCALAR(0.0);
}

template <typename SCALAR>
void PIDController<SCALAR>::computeI(const SCALAR& error)
{
    I_ += parameters_.k_i * error * parameters_.dt;

    // anti wind-up
    if (I_ > parameters_.Imax)
    {
        I_ = parameters_.Imax;
    }
    if (I_ < -parameters_.Imax)
    {
        I_ = -parameters_.Imax;
    }
}

}  // namespace core
}  // namespace ct
