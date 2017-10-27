/**********************************************************************************************************************
This file is part of the Control Toobox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <iostream>

#include <ct/core/control/siso/PIDController.h>

#include <ct/core/types/Time.h>

namespace ct {
namespace core {

double PIDController::computeControl(const double& state, const core::Time& t)
{
    double error = setpoint_.stateDesired_ - state;

    // ** P-part **
    // ============
    double P = parameters_.k_p * error;

    // ** I-part **
    // ============
    computeI(error);

    // ** D-Part **
    // ============
    double D = parameters_.k_d * (setpoint_.stateDerivativeDesired_ - (state - statePrevious_) / parameters_.dt);

    // ** Controller Output **
    // =======================
    double u = (P + I_ + D);
    saturateControl(u);

    statePrevious_ = state;

    return u;
}

double PIDController::computeControl(const double& state, const double& stateDerivative, const core::Time& t)
{
    double error = setpoint_.stateDesired_ - state;

    // ** P-part **
    // ============
    double P = parameters_.k_p * error;

    // ** I-part **
    // ============
    computeI(error);

    // ** D-Part **
    // ============
    double D = parameters_.k_d * (setpoint_.stateDerivativeDesired_ - stateDerivative);

    // ** Controller Output **
    // =======================
    double u = (P + I_ + D);

    saturateControl(u);

    statePrevious_ = state;

    return u;
}

void PIDController::saturateControl(double& u)
{
    if (u > parameters_.uMax)
        u = parameters_.uMax;
    if (u < parameters_.uMin)
        u = parameters_.uMin;
}

void PIDController::computeI(double error)
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

void PIDController::reset()
{
    statePrevious_ = 0.0;
    I_ = 0.0;
}

}  // namespace core
}  // namespace ct
