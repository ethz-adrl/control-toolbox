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

#ifndef SECONDORDERACTUATORDYNAMICS_H_
#define SECONDORDERACTUATORDYNAMICS_H_

namespace ct {
namespace robot {
namespace actuator {

template <size_t NJOINTS>
class SecondOrderActuatorDynamics : public ActuatorDynamicsBase <2*NJOINTS, NJOINTS>
{
public:
	SecondOrderActuatorDynamics(double frequency, double damping = 1.0):
		system_(frequency, damping)
	{}

	void calculateStateDerivatives(
		const robot::actuator::ActuatorState<2*NJOINTS, NJOINTS>& state,
		const rigidBodyGroup::dynamics::InternalForces<NJOINTS>& control,
		const double& t,
		rigidBodyGroup::state::ActuatorState<2*NJOINTS, NJOINTS>& stateDerivative) override
	{
		for (size_t i=0; i<NJOINTS; i++)
		{
			Eigen::Vector2d secondOrderState;
			Eigen::Vector2d secondOrderStateDerivative;

			secondOrderState << state(i), state(i+NJOINTS);
			system_.computeDerivative(secondOrderState, control(i), secondOrderStateDerivative);

			stateDerivative(i) = secondOrderStateDerivative(0);
			stateDerivative(i+NJOINTS) = secondOrderStateDerivative(1);
		}
	}

private:
	SecondOrderSystem system_;
};

}
}
}


#endif /* SECONDORDERACTUATORDYNAMICS_H_ */
