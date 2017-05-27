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

#ifndef INCLUDE_CT_RBD_ROBOT_SYSTEMS_FLOATINGBASEFDSYSTEM_H_
#define INCLUDE_CT_RBD_ROBOT_SYSTEMS_FLOATINGBASEFDSYSTEM_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/rbd/state/RigidBodyPose.h>
#include <ct/rbd/physics/EEContactModel.h>

#include "RBDSystem.h"

namespace ct{
namespace rbd {

/**
 * \brief A floating base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end-effector forces expressed in the world.
 */
template <class RBDDynamics, bool QUAT_INTEGRATION = false, bool EE_ARE_CONTROL_INPUTS = false>
class FloatingBaseFDSystem :
		public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
		public core::ControlledSystem<RBDDynamics::NSTATE+QUAT_INTEGRATION, RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3, typename RBDDynamics::SCALAR>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Dynamics = RBDDynamics;
	using Kinematics = typename RBDDynamics::Kinematics_t;

	typedef typename RBDDynamics::SCALAR SCALAR;

	const static size_t N_EE = RBDDynamics::N_EE;
	const static size_t STATE_DIM = RBDDynamics::NSTATE+QUAT_INTEGRATION;
	const static size_t CONTROL_DIM = RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*N_EE*3;

	typedef core::StateVector<STATE_DIM, SCALAR> StateVector;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;

	typedef core::ControlledSystem<RBDDynamics::NSTATE+QUAT_INTEGRATION, RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*N_EE*3, SCALAR> Base;

	using ContactModel = ct::rbd::EEContactModel<Kinematics>;

	FloatingBaseFDSystem() :
		Base(),
		dynamics_(),
		eeContactModel_(nullptr)
	{};

	FloatingBaseFDSystem(const FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>& other) :
		Base(other),
		dynamics_(other.dynamics_),
		eeContactModel_(other.eeContactModel_->clone())
	{};

	virtual ~FloatingBaseFDSystem() {};

	virtual FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>* clone() const override{
		return new FloatingBaseFDSystem<RBDDynamics, QUAT_INTEGRATION, EE_ARE_CONTROL_INPUTS>(*this);
	}

	virtual RBDDynamics& dynamics() override { return dynamics_; }
	virtual const RBDDynamics& dynamics() const override { return dynamics_; }

	void setContactModel(const std::shared_ptr<ContactModel>& contactModel) { eeContactModel_ = contactModel; }

	void computeControlledDynamics(
		const core::StateVector<STATE_DIM, SCALAR>& state,
		const SCALAR& t,
		const core::ControlVector<CONTROL_DIM, SCALAR>& control,
		core::StateVector<STATE_DIM, SCALAR>& derivative

	) override
	{
		typename RBDDynamics::RBDState_t x = RBDStateFromVector(state);

		typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

		std::array<typename Kinematics::EEForceLinear, N_EE> eeForcesW;
		eeForcesW.fill(Kinematics::EEForceLinear::Zero());

		if (eeContactModel_)
			eeForcesW = eeContactModel_->computeContactForces(x);

		if (EE_ARE_CONTROL_INPUTS)
		{
			for (size_t i=0; i<N_EE; i++)
			{
				eeForcesW[i] += control.template segment<3>(RBDDynamics::NJOINTS + i*3);
			}
		}

		mapEndeffectorForcesToLinkForces(x, eeForcesW, linkForces);

		typename RBDDynamics::RBDAcceleration_t xd;

		dynamics_.FloatingBaseForwardDynamics(
				x,
				control.template head<RBDDynamics::NJOINTS>(),
				linkForces,
				xd);

		derivative = toStateDerivative<QUAT_INTEGRATION>(xd, x);
	}


	/**
	 * Maps the end-effector forces expressed in the world to the link frame as required by robcogen.
	 * The link forces are transformed from world frame to the link frame. The according momentum is added.
	 * @param state robot state
	 * @param control end-effector forces expressed in the world
	 * @param linkForces forces acting on the link expressed in the link frame
	 */
	void mapEndeffectorForcesToLinkForces(
			const typename RBDDynamics::RBDState_t& state,
			const std::array<typename Kinematics::EEForceLinear, N_EE>& eeForcesW,
			typename RBDDynamics::ExtLinkForces_t& linkForces)
	{
		for (size_t i=0; i<N_EE; i++)
		{
			auto endEffector = dynamics_.kinematics().getEndEffector(i);
			size_t linkId = endEffector.getLinkId();
			linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
				dynamics_.kinematics().mapForceFromWorldToLink3d(
					eeForcesW[i],
					state.basePose(),
					state.jointPositions(),
					i);
		}
	}

	typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
	{
		return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state, typename std::enable_if<T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::QUAT);
		x.fromStateVectorQuaternion(state);
		return x;
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state, typename std::enable_if<!T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(tpl::RigidBodyPose<SCALAR>::EULER);
		x.fromStateVectorEulerXyz(state);
		return x;
	}

	template <bool T>
	core::StateVector<STATE_DIM, SCALAR> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<T, bool>::type = true)
	{
		return acceleration.toStateUpdateVectorQuaternion(state);
	}

	template <bool T>
	core::StateVector<STATE_DIM, SCALAR> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<!T, bool>::type = true)
	{
		return acceleration.toStateUpdateVectorEulerXyz(state);
	}

private:
	RBDDynamics dynamics_;

	std::shared_ptr<ContactModel> eeContactModel_;

};

} // namespace rbd
} // namespace ct

#endif /* INCLUDE_CT_RBD_ROBOT_SYSTEMS_FLOATINGBASEFDSYSTEM_H_ */
