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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************/

#pragma once

/*!
 * A simple custom PD controller for an oscillator
 *
 * see \ref DampedOscillatorCustomController.cpp on how to use it.
 *
 * \example CustomController.h
 */
// our controller class that takes a 2-dimensional state and outputs a 1-dimensional control action
class CustomController : public ct::core::Controller<2, 1>
{
public:
	static const size_t state_dim = 2;    // two states
	static const size_t control_dim = 1;  // one control action

	// default constructor
	CustomController(const ct::core::ControlVector<control_dim>& uff,  // feedforward control
		const double& kp,                                              // P gain
		const double& kd                                               // D gain
		)
		: uff_(uff), kp_(kp), kd_(kd)
	{
	}

	// destructor
	~CustomController() {}
	// copy constructor
	CustomController(const CustomController& other) : uff_(other.uff_), kp_(other.kp_), kd_(other.kd_) {}
	// clone method, needs to be implemented, overrides ct::core::Controller::clone()
	CustomController* clone() const override
	{
		return new CustomController(*this);  // calls copy constructor
	}

	// override the compute control method
	void computeControl(const ct::core::StateVector<state_dim>& state,
		const double& t,
		ct::core::ControlVector<control_dim>& controlAction) override
	{
		controlAction = uff_;                                 // apply feedforward control
		controlAction(0) -= kp_ * state(0) + kd_ * state(1);  // add feedback control
	}

private:
	ct::core::ControlVector<control_dim> uff_;  // feedforward control action
	double kp_;                                 // P gain
	double kd_;                                 // D gain
};
