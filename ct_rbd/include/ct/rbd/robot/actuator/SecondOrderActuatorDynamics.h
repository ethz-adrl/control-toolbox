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

#ifndef CT_SECONDORDERACTUATORDYNAMICS_H_
#define CT_SECONDORDERACTUATORDYNAMICS_H_

#include "ActuatorDynamics.h"

namespace ct {
namespace rbd {

/*!
 * Actuator Dynamics modelled as second order system, an oscillator with damping.
 */
template <size_t NJOINTS, typename SCALAR = double>
class SecondOrderActuatorDynamics : public ActuatorDynamics <NJOINTS, 2*NJOINTS, SCALAR>
{
public:

	typedef ActuatorDynamics <NJOINTS, 2*NJOINTS, SCALAR> BASE;

	//! constructor
	SecondOrderActuatorDynamics(SCALAR w_n, SCALAR zeta = SCALAR(1.0), SCALAR g_dc = SCALAR(1.0)):
		oscillator_(w_n, zeta, g_dc)
	{}

	//! destructor
	virtual ~SecondOrderActuatorDynamics(){}

	//! deep cloning
	virtual SecondOrderActuatorDynamics<NJOINTS, SCALAR>* clone() const override {return new SecondOrderActuatorDynamics(*this);}


	virtual void computePdot(
			const BASE::act_state_vector_t& x,
			const BASE::act_vel_vector_t& v,
			const ct::core::ControlVector<NJOINTS, SCALAR>& control,
			BASE::act_pos_vector_t& pDot
		) override
	{
		// as the oscillator is symplectic itself, we simply transcribe the velocity coordinates
		pDot = v;
	}


	virtual void computeVdot(
			const BASE::act_state_vector_t& x,
			const BASE::act_pos_vector_t& p,
			const BASE::ControlVector<NJOINTS, SCALAR>& control,
			BASE::act_vel_vector_t& vDot
		) override
	{
		// evaluate oscillator dynamics for each joint
		for (size_t i=0; i<NJOINTS; i++)
		{
			Eigen::Vector2d secondOrderState;
			Eigen::Vector2d secondOrderStateDerivative;

			secondOrderState << p(i), x(i+NJOINTS);

			oscillator_.computeControlledDynamics(secondOrderState, 0.0, control(i), secondOrderStateDerivative);

			vDot(i) = secondOrderStateDerivative(1);
		}
	}


	virtual void computeControlOutput(
			const ct::rbd::RBDState<NJOINTS, SCALAR>& robotState,
			const BASE::act_state_vector_t& actState,
			core::ControlVector<NJOINTS, SCALAR>& controlOutput) override
	{
		// for this simple actuator dynamics model, the controlOutput is just the "position" coordinates of the actuator state
		controlOutput =  actState.topRows<NJOINTS>();
	}


private:

	ct::core::SecondOrderSystem oscillator_;
};

}
}


#endif /* CT_SECONDORDERACTUATORDYNAMICS_H_ */
