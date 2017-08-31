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

#ifndef CT_ACTUATORDYNAMICS_H_
#define CT_ACTUATORDYNAMICS_H_

#include <ct/core/core.h>
#include <ct/rbd/state/JointState.h>

namespace ct {
namespace rbd {


/*!
 * This class covers the actuator dynamics for a robot, i.e. not the dynamics of a single actuator, but
 * the dynamics of the collection of all actuators in the system
 * The actuators are assumed to form a symplectic system.
 */
template <size_t NJOINTS, size_t ACT_STATE_DIM = 2*NJOINTS, typename SCALAR = double>
class ActuatorDynamics : public core::SymplecticSystem<ACT_STATE_DIM/2, ACT_STATE_DIM/2, NJOINTS, SCALAR>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const size_t ACT_POS_DIM = ACT_STATE_DIM/2;
	static const size_t ACT_VEL_DIM = ACT_STATE_DIM/2;

	typedef ct::core::StateVector<ACT_STATE_DIM, SCALAR> act_state_vector_t;
	typedef ct::core::StateVector<ACT_POS_DIM, SCALAR> act_pos_vector_t;
	typedef ct::core::StateVector<ACT_VEL_DIM, SCALAR> act_vel_vector_t;


	ActuatorDynamics() {};

	virtual ~ActuatorDynamics() {};

	virtual ActuatorDynamics<NJOINTS, ACT_STATE_DIM, SCALAR>* clone() const override = 0;

	virtual void computePdot(
			const act_state_vector_t& x,
			const act_vel_vector_t& v,
			const core::ControlVector<NJOINTS, SCALAR>& control,
			act_pos_vector_t& pDot
		) override = 0;


	virtual void computeVdot(
			const act_state_vector_t& x,
			const act_pos_vector_t& p,
			const core::ControlVector<NJOINTS, SCALAR>& control,
			act_vel_vector_t& vDot
		) override = 0;


	//! output equation of the actuator
	/**
	 * Algebraic output equation for the actuator system, translating the current actuator state and the current robot state into
	 * an output control vector.
	 * Example: in an RBD setting, the controlOutput may be the actual joint torques.
	 * @param robotState current RBD state of the robot
	 * @param actState current state of the actuators
	 * @param controlOutput control output (output side of actuator)
	 */
	virtual core::ControlVector<NJOINTS, SCALAR> computeControlOutput(
			const ct::rbd::tpl::JointState<NJOINTS, SCALAR>& robotJointState,
			const act_state_vector_t& actState) = 0;
};


} // namespace rbd
} // namespace ct


#endif /* CT_ACTUATORDYNAMICS_H_ */
