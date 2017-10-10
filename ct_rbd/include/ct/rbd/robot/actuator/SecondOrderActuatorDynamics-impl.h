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

#ifndef CT_SECONDORDERACTUATORDYNAMICS_IMPL_H_
#define CT_SECONDORDERACTUATORDYNAMICS_IMPL_H_

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::SecondOrderActuatorDynamics(
		SCALAR w_n,
		SCALAR zeta,
		SCALAR g_dc):
oscillator_(w_n, zeta, g_dc)
{}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>::~SecondOrderActuatorDynamics()
{}

template <size_t NJOINTS, typename SCALAR>
SecondOrderActuatorDynamics<NJOINTS, SCALAR>* SecondOrderActuatorDynamics<NJOINTS, SCALAR>::clone() const
{
	return new SecondOrderActuatorDynamics(*this);
}

template <size_t NJOINTS, typename SCALAR>
void SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computePdot(
		const typename BASE::act_state_vector_t& x,
		const typename BASE::act_vel_vector_t& v,
		const ct::core::ControlVector<NJOINTS, SCALAR>& control,
		typename BASE::act_pos_vector_t& pDot)
{
	// as the oscillator is symplectic itself, we simply transcribe the velocity coordinates
	pDot = v;
}

template <size_t NJOINTS, typename SCALAR>
void SecondOrderActuatorDynamics<NJOINTS, SCALAR>::computeVdot(
		const typename BASE::act_state_vector_t& x,
		const typename BASE::act_pos_vector_t& p,
		const ct::core::ControlVector<NJOINTS, SCALAR>& control,
		typename BASE::act_vel_vector_t& vDot)
{
	// evaluate oscillator dynamics for each joint
	for (size_t i=0; i<NJOINTS; i++)
	{
		core::StateVector<2, SCALAR> secondOrderState;
		core::StateVector<2, SCALAR> secondOrderStateDerivative;
		core::ControlVector<1, SCALAR> inputCtrl;
		inputCtrl(0) = control(i);

		secondOrderState << p(i), x(i+NJOINTS);

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

#endif /* CT_SECONDORDERACTUATORDYNAMICS_IMPL_H_ */
