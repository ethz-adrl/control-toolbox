/*
 * ProjectedFDSystem.hpp
 *
 *  Created on: Oct 3, 2016
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_RBD_ROBOT_SYSTEMS_PROJECTEDFDSYSTEM_H_
#define INCLUDE_CT_RBD_ROBOT_SYSTEMS_PROJECTEDFDSYSTEM_H_

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
template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class ProjectedFDSystem :
		public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
		public core::ControlledSystem<RBDDynamics::NSTATE+QUAT_INTEGRATION, RBDDynamics::NJOINTS>
{
public:
	using Dynamics = RBDDynamics;
	using Kinematics = typename RBDDynamics::Kinematics_t;

	const static size_t N_EE = RBDDynamics::N_EE;
	const static size_t STATE_DIM = RBDDynamics::NSTATE+QUAT_INTEGRATION;
	const static size_t CONTROL_DIM = RBDDynamics::NJOINTS;

	typedef core::ControlledSystem<RBDDynamics::RBDState_t::NSTATE+QUAT_INTEGRATION, RBDDynamics::NJOINTS> Base;

	ProjectedFDSystem(typename RBDDynamics::EE_in_contact_t eeInContact = true) :
		eeInContact_(eeInContact)
	{};

	virtual ~ProjectedFDSystem() {};

	virtual RBDDynamics& dynamics() override { return dynamics_; }
	virtual const RBDDynamics& dynamics() const override { return dynamics_; }

	void computeControlledDynamics(
		const core::StateVector<STATE_DIM>& state,
		const core::Time& t,
		const core::ControlVector<CONTROL_DIM>& control,
		core::StateVector<STATE_DIM>& derivative

	) override
	{
		typename RBDDynamics::RBDState_t x = RBDStateFromVector(state);

		typename RBDDynamics::RBDAcceleration_t xd;

		dynamics_.ProjectedForwardDynamics(eeInContact_, x, control, xd);

		derivative = toStateDerivative<QUAT_INTEGRATION>(xd, x);
	}

	void setEEInContact(typename RBDDynamics::EE_in_contact_t& eeInContact)
	{
		eeInContact_ = eeInContact;
	}

	typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM>& state)
	{
		return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM>& state, typename std::enable_if<T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(RigidBodyPose::QUAT);
		x.fromStateVectorQuaternion(state);
		return x;
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM>& state, typename std::enable_if<!T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(RigidBodyPose::EULER);
		x.fromStateVectorEulerXyz(state);
		return x;
	}

	template <bool T>
	core::StateVector<STATE_DIM> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<T, bool>::type = true)
	{
		return acceleration.toStateUpdateVectorQuaternion(state);
	}

	template <bool T>
	core::StateVector<STATE_DIM> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<!T, bool>::type = true)
	{
		return acceleration.toStateUpdateVectorEulerXyz(state);
	}


	virtual ProjectedFDSystem<RBDDynamics, QUAT_INTEGRATION>* clone() const override { throw std::runtime_error("clone not implemented"); }

private:
	typename RBDDynamics::EE_in_contact_t eeInContact_;

	RBDDynamics dynamics_;

};

} // namespace rbd
} // namespace ct

#endif /* INCLUDE_CT_RBD_ROBOT_SYSTEMS_PROJECTEDFDSYSTEM_H_ */
