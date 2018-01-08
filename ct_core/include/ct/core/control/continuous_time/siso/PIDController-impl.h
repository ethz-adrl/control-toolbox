/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace core {

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
SCALAR PIDController<SCALAR>::computeControl(const SCALAR& state,
    const SCALAR& stateDerivative,
    const Time& t)
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

template <typename SCALAR>
void PIDController<SCALAR>::reset()
{
    statePrevious_ = SCALAR(0.0);
    I_ = SCALAR(0.0);
}

}  // namespace core
}  // namespace ct
