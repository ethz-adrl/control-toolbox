/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

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
	double D = parameters_.k_d * (setpoint_.stateDerivativeDesired_ - (state - statePrevious_)/parameters_.dt);

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

} // namespace core
} // namespace ct
