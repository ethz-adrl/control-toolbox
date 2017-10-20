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
/*!
 *
 * \example DampedOscillatorCustomController.cpp
 */


#include <ct/core/core.h>
#include "CustomController.h"

int main(int argc, char** argv)
{
	// a damped oscillator has two states, position and velocity
	const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;      // = 2
	const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;  // = 1

	// create a state
	ct::core::StateVector<state_dim> x;

	// we initialize it at 0
	x.setZero();

	// create our oscillator
	double w_n = 50;
	std::shared_ptr<ct::core::SecondOrderSystem> oscillator(new ct::core::SecondOrderSystem(w_n));

	// create our controller
	double kp = 10;
	double kd = 1;
	ct::core::ControlVector<control_dim> uff;
	uff << 2.0;
	std::shared_ptr<CustomController> controller(new CustomController(uff, kp, kd));

	// assign our controller
	oscillator->setController(controller);

	// create an integrator
	ct::core::Integrator<state_dim> integrator(oscillator, ct::core::IntegrationType::RK4);

	// simulate 1000 steps
	double dt = 0.001;
	ct::core::Time t0 = 0.0;
	size_t nSteps = 1000;
	integrator.integrate_n_steps(x, t0, nSteps, dt);

	// print the new state
	std::cout << "state after integration: " << x.transpose() << std::endl;

	return 1;
}
