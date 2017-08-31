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

#ifndef INCLUDE_CT_RBD_ROBOT_SYSTEMS_FIXBASEFDSYSTEM_H_
#define INCLUDE_CT_RBD_ROBOT_SYSTEMS_FIXBASEFDSYSTEM_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>

#include "RBDSystem.h"

namespace ct{
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end effector forces expressed in the world.
 */
template <class RBDDynamics, size_t ACTUATOR_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystem :
		public RBDSystem<RBDDynamics, false>,
		public core::SymplecticSystem<
			RBDDynamics::NJOINTS+ACTUATOR_STATE_DIM/2, 	// position dimension of combined system
			RBDDynamics::NJOINTS+ACTUATOR_STATE_DIM/2,  // velocity dimension of combined system
			RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3,  // input dimension of combined system
			typename RBDDynamics::SCALAR
			>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Dynamics = RBDDynamics;

	typedef typename RBDDynamics::SCALAR SCALAR;

	const static size_t N_EE = RBDDynamics::N_EE;

	// the combined system consists of the RBD states and the actuator states
	static const size_t STATE_DIM = RBDDynamics::NSTATE + ACTUATOR_STATE_DIM;

	// the control inputs are the inputs to the actuators / joint torques + EE forces, if applicable
	static const size_t CONTROL_DIM = RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3;

	typedef core::SymplecticSystem<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM, SCALAR> Base;


	//! constructor
	FixBaseFDSystem() :
			Base(core::SYSTEM_TYPE::SECOND_ORDER),
			actuatorDynamics_(nullptr)
	{
		basePose_.setIdentity();
	}


	//! copy constructor
	/*!
	 * take care of explicitly cloning actuatorDynamics, if existent
	 * @param arg instance of FixBaseFDSystem to be copied.
	 */
	FixBaseFDSystem(const FixBaseFDSystem& arg) :
		Base(arg),
		basePose_(arg.basePose_),
		dynamics_(RBDDynamics())
	{
		if (arg.actuatorDynamics_)
		{
			actuatorDynamics_ = std::shared_ptr<ActuatorDynamics<RBDDynamics::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>> (arg.actuatorDynamics_->clone());
		}
	}

	//! destructor
	virtual ~FixBaseFDSystem(){}

	//! get dynamics
	virtual RBDDynamics& dynamics() override { return dynamics_; }

	//! get dynamics (const)
	virtual const RBDDynamics& dynamics() const override { return dynamics_; }


	//! compute position derivatives, for both RBD system and actuator dynamics, if applicable
	virtual void computePdot(
			const core::StateVector<STATE_DIM, SCALAR>& x,
			const core::StateVector<STATE_DIM/2, SCALAR>& v,
			const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
			core::StateVector<STATE_DIM/2, SCALAR>& pDot) override
	{
		// the top rows hold the RBD velocities ...
		pDot.template topRows<RBDDynamics::NJOINTS>() = v.template topRows<RBDDynamics::NJOINTS>();

		if(actuatorDynamics_)
		{
#if ACTUATOR_STATE_DIM > 0 // todo find clean solution
			// ... the bottom rows hold the actuator dynamics
			Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actPdot = pDot.template bottomRows<ACTUATOR_STATE_DIM/2>();

			// get references to the current actuator position and velocity states
			const Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actPos = x.segment<ACTUATOR_STATE_DIM/2>(RBDDynamics::NJOINTS);
			const Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actVel = x.template bottomRows<ACTUATOR_STATE_DIM/2>();

			// assemble temporary actuator state
			core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actState;
			actState << actPos, actVel;

			// the controls get remapped to the actuator input
			actuatorDynamics_->computePdot(actState, actVel, controlIn.template topRows<RBDDynamics::NJOINTS>(), actPdot);
#endif
		}
	}


	//! compute velocity derivatives, for both RBD system and actuator dynamics
	virtual void computeVdot(
			const core::StateVector<STATE_DIM, SCALAR>& x,
			const core::StateVector<STATE_DIM/2, SCALAR>& p,
			const core::ControlVector<CONTROL_DIM, SCALAR>& controlIn,
			core::StateVector<STATE_DIM/2, SCALAR>& vDot) override
	{
		// temporary variable for the control (will get modified by the actuator dynamics, if applicable)
		core::ControlVector<CONTROL_DIM, SCALAR> control = controlIn;

		// extract the current RBD joint state from the state vector
		typename RBDDynamics::JointState_t jState; jState.setZero();
		jState.getPositions() = p.template topRows<RBDDynamics::NJOINTS>();
		jState.getVelocities() = x.template segment<RBDDynamics::NJOINTS>(STATE_DIM/2);


		if(actuatorDynamics_)
		{
#if ACTUATOR_STATE_DIM > 0 // todo find clean solution
			// ... the bottom rows hold the actuator dynamics
			Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actVdot = vDot.template bottomRows<ACTUATOR_STATE_DIM/2>();

			// get references to the current actuator position and velocity states
			const Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actPos = x.segment<ACTUATOR_STATE_DIM/2>(RBDDynamics::NJOINTS);
			const Eigen::Ref<core::StateVector<ACTUATOR_STATE_DIM/2, SCALAR>> actVel = x.template bottomRows<ACTUATOR_STATE_DIM/2>();

			// assemble temporary actuator state
			core::StateVector<ACTUATOR_STATE_DIM, SCALAR> actState;
			actState << actPos, actVel;

			// the input controls get remapped to the actuator input
			actuatorDynamics_->computeVdot(actState, actPos, controlIn.template topRows<RBDDynamics::NJOINTS>(), actVdot);

			// overwrite control with actuator control output as a function of current robot and actuator states
			control = actuatorDynamics_->computeControlOutput(jState, actState);
#endif
		}


		// Cache updated rbd state
		typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

		// add end effector forces as control inputs (if applicable)
		if(EE_ARE_CONTROL_INPUTS == true)
		{
			for (size_t i=0; i<RBDDynamics::N_EE; i++)
			{
				auto endEffector = dynamics_.kinematics().getEndEffector(i);
				size_t linkId = endEffector.getLinkId();
				linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
						dynamics_.kinematics().mapForceFromWorldToLink3d(
								control.template segment<3>(RBDDynamics::NJOINTS + i*3),
								basePose_, jState.getPositions(), i);
			}
		}

		typename RBDDynamics::JointAcceleration_t jAcc;

		dynamics_.FixBaseForwardDynamics(
				jState,
				control.template head<RBDDynamics::NJOINTS>(),
				linkForces,
				jAcc);

		vDot =jAcc.getAcceleration();
	}

	//! deep cloning
	virtual FixBaseFDSystem<RBDDynamics, EE_ARE_CONTROL_INPUTS>* clone() const override
	{
		return new FixBaseFDSystem<RBDDynamics, EE_ARE_CONTROL_INPUTS> (*this);
	}

	//! transform control systems state vector to a RBDState
	typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
	{
		typename RBDDynamics::RBDState_t x;
		x.setZero();
		x.basePose() = basePose_;
		x.joints() = state;
		return x;
	}

private:

	//! a "dummy" base pose, which is always identity/zero for the fix-base case
	tpl::RigidBodyPose<SCALAR> basePose_;

	RBDDynamics dynamics_;

	std::shared_ptr<ActuatorDynamics<RBDDynamics::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>> actuatorDynamics_;
};

} // namespace rbd
} // namespace ct

#endif /* INCLUDE_CT_RBD_ROBOT_SYSTEMS_FIXBASEFDSYSTEM_H_ */
