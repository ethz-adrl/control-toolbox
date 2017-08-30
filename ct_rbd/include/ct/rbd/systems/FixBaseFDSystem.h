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


#include "RBDSystem.h"

namespace ct{
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics. The input vector
 * is assumed to consist of joint torques and end effector forces expressed in the world.
 */
template <class RBDDynamics, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystem :
		public RBDSystem<RBDDynamics, false>,
		public core::SymplecticSystem<RBDDynamics::NSTATE / 2, RBDDynamics::NSTATE / 2, RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3, typename RBDDynamics::SCALAR>
{
public:
	using Dynamics = RBDDynamics;

	typedef typename RBDDynamics::SCALAR SCALAR;

	const static size_t N_EE = RBDDynamics::N_EE;
	static const size_t STATE_DIM = RBDDynamics::NSTATE;
	static const size_t CONTROL_DIM = RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3;

	typedef core::StateVector<STATE_DIM, SCALAR> StateVector;
	typedef core::ControlVector<CONTROL_DIM, SCALAR> ControlVector;

	typedef core::SymplecticSystem<RBDDynamics::NSTATE / 2, RBDDynamics::NSTATE / 2, RBDDynamics::NJOINTS+EE_ARE_CONTROL_INPUTS*RBDDynamics::N_EE*3, SCALAR> Base;

	typedef Eigen::Matrix<SCALAR, RBDDynamics::NJOINTS, RBDDynamics::NJOINTS> DampingMatrix_t;

	FixBaseFDSystem() :
			Base(core::SYSTEM_TYPE::SECOND_ORDER)
	{
		basePose_.setIdentity();
	}

	/**
	 * The system type becomes GENERAL if we introduce artificial damping
	 * @param dampingMatrix
	 */
	FixBaseFDSystem(const DampingMatrix_t& dampingMatrix):
		Base(core::SYSTEM_TYPE::GENERAL)
	{
		basePose_.setIdentity();
		/*
		 * that is missing currently
		 * */
	}

	FixBaseFDSystem(const FixBaseFDSystem& arg) :
		Base(arg),
		basePose_(arg.basePose_),
		dynamics_(RBDDynamics())
	{}

	virtual ~FixBaseFDSystem(){}

	virtual RBDDynamics& dynamics() override { return dynamics_; }
	virtual const RBDDynamics& dynamics() const override { return dynamics_; }

	virtual void computePdot(
			const core::StateVector<STATE_DIM, SCALAR>& x,
			const core::StateVector<STATE_DIM / 2, SCALAR>& v,
			const core::ControlVector<CONTROL_DIM, SCALAR>& control,
			core::StateVector<STATE_DIM / 2, SCALAR>& pDot) override {
		pDot = v;
	}

	virtual void computeVdot(
			const StateVector& x,
			const core::StateVector<STATE_DIM / 2, SCALAR>& p,
			const ControlVector& control,
			core::StateVector<STATE_DIM / 2, SCALAR>& vDot
	) override {

		typename RBDDynamics::JointState_t jState = x;

		jState.getPositions() = p;

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


	virtual FixBaseFDSystem<RBDDynamics, EE_ARE_CONTROL_INPUTS>* clone() const override {
		return new FixBaseFDSystem<RBDDynamics, EE_ARE_CONTROL_INPUTS> (*this);
	}

	typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
	{
		typename RBDDynamics::RBDState_t x;
		x.setZero();
		x.basePose() = basePose_;
		x.joints() = state;
		return x;
	}

private:

	tpl::RigidBodyPose<SCALAR> basePose_;

	RBDDynamics dynamics_;
};

} // namespace rbd
} // namespace ct

#endif /* INCLUDE_CT_RBD_ROBOT_SYSTEMS_FIXBASEFDSYSTEM_H_ */
