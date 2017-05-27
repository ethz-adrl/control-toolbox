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

#ifndef INCLUDE_CT_RBD_PHYSICS_EECONTACTMODEL_H_
#define INCLUDE_CT_RBD_PHYSICS_EECONTACTMODEL_H_

#include <kindr/Core>

#include <ct/rbd/state/RBDState.h>

#include <iit/rbd/rbd.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/TraitSelector.h>

namespace ct {
namespace rbd {


/**
 * Soft contact model that only uses end-effector positions/velocities to compute the contact force
 */
template <class Kinematics>
class EEContactModel
{
public:
	static const size_t NUM_EE = Kinematics::NUM_EE;
	static const size_t NJOINTS = Kinematics::NJOINTS;

	typedef typename Kinematics::SCALAR SCALAR;

	typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait TRAIT;

	typedef std::array<bool, NUM_EE> ActiveMap;
	typedef typename Kinematics::EEForceLinear EEForceLinear;
	typedef std::array<EEForceLinear, NUM_EE> EEForcesLinear;

	typedef Eigen::Matrix<SCALAR, 3, 1> Vector3s;
	typedef kindr::Position<SCALAR, 3> Position3S;
	typedef kindr::Velocity<SCALAR, 3> Velocity3S;



	enum VELOCITY_SMOOTHING {
		NONE = 0,
		SIGMOID = 1,
		TANH = 2,
		ABS = 3
	};

	EEContactModel(
			const SCALAR& k = SCALAR(5000),
			const SCALAR& d = SCALAR(500.0),
			const SCALAR& alpha = SCALAR(100.0),
			const SCALAR& alpha_n = SCALAR(-1.0),
			const SCALAR& zOffset = SCALAR(0.0),
			const VELOCITY_SMOOTHING& smoothing = NONE,
			const std::shared_ptr<Kinematics>& kinematics = std::shared_ptr<Kinematics>(new Kinematics())) :
		kinematics_(kinematics),
		smoothing_(smoothing),
		k_(k),
		d_(d),
		alpha_(alpha),
		alpha_n_(alpha_n),
		zOffset_(zOffset)
	{
		for (size_t i=0; i<NUM_EE; i++)
			EEactive_[i] = true;
	}

	EEContactModel(const EEContactModel& other) :
		kinematics_(other.kinematics_->clone()),
		smoothing_(other.smoothing_),
		k_(other.k_),
		d_(other.d_),
		alpha_(other.alpha_),
		alpha_n_(other.alpha_n_),
		zOffset_(other.zOffset_),
		EEactive_(other.EEactive_)
	{
	}

	EEContactModel* clone() const {
		return new EEContactModel(*this);
	}


	/**
	 * Sets which end-effectors can have forces excerted on them
	 * @param activeMap flags of active end-effectors
	 */
	void setActiveEE(const ActiveMap& activeMap)
	{
		EEactive_ = activeMap;
	}

	/**
	 * Computes the contact forces given a state of the robot. Returns forces expressed in the world frame
	 * @param state The state of the robot
	 * @return End-effector forces expressed in the world frame
	 */
	EEForcesLinear computeContactForces(const tpl::RBDState<NJOINTS, SCALAR>& state)
	{
		EEForcesLinear eeForces;

		for (size_t i=0; i<NUM_EE; i++)
		{
			if (EEactive_[i])
			{
				Vector3s eePenetration = computePenetration(i, state.basePose(), state.jointPositions());

				if (eeInContact(eePenetration))
				{
					Velocity3S eeVelocity = kinematics_->getEEVelocityInWorld(i, state);
					eeForces[i] = computeEEForce(eePenetration, eeVelocity);
				} else
				{
					eeForces[i].setZero();
				}
			}
		}

		return eeForces;
	}

	SCALAR& alpha() { return alpha_; }
	SCALAR& alpha_n() { return alpha_n_; }
	SCALAR& k() { return k_; }
	SCALAR& d() { return d_; }
	SCALAR& zOffset() { return zOffset_; }

	VELOCITY_SMOOTHING& smoothing() { return smoothing_; }


private:
	/**
	 * Checks if end-effector is in contact. Currently assumes this is the case for negative z
	 * @param eePenetration The surface penetration of the end-effector
	 * @return flag if the end-effector is in contact
	 */
	bool eeInContact(const Vector3s& eePenetration)
	{
		if (smoothing_ == NONE && eePenetration(2) > 0.0)
			return false;
		else
			return true;
	}


	/**
	 * Computes the surface penetration. Currently assumes the surface is at height z = 0.
	 * @param eeId ID of the end-effector
	 * @param basePose Position of the robot base
	 * @param jointPosition Joint position of the robot
	 * @return Penetration in world coordinates
	 */
	Vector3s computePenetration(const size_t& eeId, const tpl::RigidBodyPose<SCALAR>& basePose, const typename tpl::JointState<NJOINTS,SCALAR>::Position& jointPosition)
	{
		Position3S pos = kinematics_->getEEPositionInWorld(eeId, basePose, jointPosition);

		// we currently assume flat ground at height zero penetration is only z height
		Vector3s penetration;
		penetration << SCALAR(0.0), SCALAR(0.0), pos.z();

		return penetration;
	}

	EEForceLinear computeEEForce(const Vector3s& eePenetration, const Velocity3S& eeVelocity)
	{
		EEForceLinear eeForce;

		computeDamperForce(eeForce, eePenetration, eeVelocity);

		smoothEEForce(eeForce, eePenetration);

		computeNormalSpring(eeForce, eePenetration(2) - zOffset_, eeVelocity.toImplementation()(2));

		return eeForce;
	}

	void smoothEEForce(EEForceLinear& eeForce, const Vector3s& eePenetration)
	{
		switch(smoothing_)
		{
		case NONE:
			return;
		case SIGMOID:
			eeForce *= 1./(1. + TRAIT::exp(eePenetration(2)*alpha_));
			return;
		case TANH:
			// same as sigmoid, maybe cheaper / more expensive to compute?
			eeForce *= 0.5*TRAIT::tanh(-0.5*eePenetration(2)*alpha_) + 0.5;
			return;
		case ABS:
			eeForce *= 0.5 * -eePenetration(2)*alpha_ / (1. + TRAIT::fabs(-eePenetration(2)*alpha_)) + 0.5;
			return;
		default:
			throw std::runtime_error("undefined smoothing function");
		}
	}

	void computeDamperForce(EEForceLinear& force, const Vector3s& eePenetration, const Velocity3S& eeVelocity)
	{
		force = -d_ * eeVelocity.toImplementation();
	}

	void computeNormalSpring(EEForceLinear& force, const SCALAR& p_N, const SCALAR& p_dot_N)
	{
		if (alpha_n_ > SCALAR(0))
		{
			force(2) += k_*TRAIT::exp(-alpha_n_*p_N);
		} else if (p_N <= SCALAR(0))
		{
			force(2) -= k_*p_N;
		}
	}


	std::shared_ptr<Kinematics> kinematics_;

	VELOCITY_SMOOTHING smoothing_;

	SCALAR k_;
	SCALAR d_;
	SCALAR alpha_;
	SCALAR alpha_n_;
	SCALAR zOffset_;

	ActiveMap EEactive_;
};


}
}

#endif /* INCLUDE_CT_RBD_PHYSICS_EECONTACTMODEL_H_ */
