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

#pragma once

#include <ct/rbd/common/SpatialForceVector.h>
#include <ct/rbd/state/RBDState.h>

#include "kinematics/EndEffector.h"
#include "kinematics/FloatingBaseTransforms.h"

namespace ct {
namespace rbd {

/**
 * \brief A general class for computing Kinematic properties
 *
 * This class implements useful Kinematic quantities. It wraps RobCoGen to
 * have access to efficient transforms and jacobians.
 */
template <class RBD, size_t N_EE>
class Kinematics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Kinematics(std::shared_ptr<RBD> rbdContainer = std::shared_ptr<RBD>(new RBD())) :
		rbdContainer_(rbdContainer),
		floatingBaseTransforms_(rbdContainer_)
	{
		initEndeffectors(endEffectors_);
	}

    Kinematics(const Kinematics<RBD, N_EE>& other) :
    	rbdContainer_(new RBD()),
    	endEffectors_(other.endEffectors_),
		floatingBaseTransforms_(rbdContainer_)
    {
    }

	virtual ~Kinematics() {};

	Kinematics<RBD,N_EE>* clone() const {
		return new Kinematics<RBD, N_EE>(*this);
	}

	static const size_t NUM_EE = N_EE;
	static const size_t NJOINTS = RBD::NJOINTS;
	static const size_t NLINKS = RBD::NLINKS;

	typedef std::shared_ptr<Kinematics<RBD, N_EE>> Ptr_t;

	typedef RBD ROBCOGEN;
	typedef typename ROBCOGEN::SCALAR SCALAR;

	typedef typename ROBCOGEN::HomogeneousTransform HomogeneousTransform;
	typedef typename ROBCOGEN::HomogeneousTransforms HomogeneousTransforms;
	typedef typename ROBCOGEN::ForceTransform ForceTransform;
	typedef typename ROBCOGEN::Jacobian Jacobian;
	typedef typename ROBCOGEN::Jacobians Jacobians;
	typedef Eigen::Matrix<SCALAR, 3, 1> Vector3Tpl; 
	typedef kindr::Position<SCALAR, 3> Position3Tpl;
	typedef kindr::Velocity<SCALAR, 3> Velocity3Tpl;
	typedef SpatialForceVector<SCALAR> EEForce;
	typedef Vector3Tpl EEForceLinear;

	//typedef kindr::Vector<kindr::PhysicalType::Position, SCALAR, 3> KindrPosTpl;


	void initEndeffectors(std::array<EndEffector<NJOINTS, SCALAR>, NUM_EE>& endeffectors)
	{
		for (size_t i=0; i<NUM_EE; i++)
		{
			endeffectors[i].setLinkId(ROBCOGEN::UTILS::eeIdToLinkId(i));
		}
	}

	/**
	 * \brief Get an end-effector
	 * @param id end-effector id
	 * @return
	 */
	EndEffector<NJOINTS, SCALAR>& getEndEffector(size_t id) { return endEffectors_[id];};

	/**
	 * \brief Set an end-effector
	 * @param id end-effector id
	 * @param ee end-effector
	 */
	void setEndEffector(size_t id, const EndEffector<NJOINTS, SCALAR>& ee) {};


	Jacobian getJacobianById(size_t linkId) { throw std::runtime_error("getJacobian not implemented"); return tpl::RigidBodyPose<SCALAR>(); };

	FloatingBaseTransforms<RBD>& floatingBaseTransforms() { throw std::runtime_error("floating base transforms not implemented"); return floatingBaseTransforms_; };
	FloatingBaseTransforms<RBD>& floatingBaseTransforms() const { throw std::runtime_error("floating base transforms not implemented"); return floatingBaseTransforms_; };

	const HomogeneousTransforms& transforms() const {return robcogen().homogeneousTransforms();}
	HomogeneousTransforms& transforms() {return robcogen().homogeneousTransforms();} 

	const Jacobians& jacobians() const {return robcogen().jacobians();}
	Jacobians& jacobians() {return robcogen().jacobians();}

//	RigidBodyPoseTpl<SCALAR> getLinkPoseInWorld(size_t linkId, const typename JointStateTpl<NJOINTS, SCALAR>::Position& jointPosition, const RigidBodyPoseTpl<SCALAR>& basePose) {
//		throw std::runtime_error("not implemented");
//		return getHomogeneousTransformBaseLinkById(linkId, jointPosition);
//	};

	Velocity3Tpl getEEVelocityInBase(
			size_t eeId,
			const tpl::RBDState<NJOINTS, SCALAR>& rbdState)
	{
		Velocity3Tpl eeVelocityBase;
		eeVelocityBase.toImplementation() = (robcogen().getJacobianBaseEEbyId(eeId, rbdState.jointPositions())*rbdState.jointVelocities()).template bottomRows<3>();

		// add translational velocity induced by linear base motion
		eeVelocityBase += rbdState.base().velocities().getTranslationalVelocity();

		// add translational velocity induced by angular base motion
		eeVelocityBase += rbdState.base().velocities().getRotationalVelocity().cross(getEEPositionInBase(eeId, rbdState.jointPositions()));

		return eeVelocityBase;
	}

	Velocity3Tpl getEEVelocityInWorld (
			size_t eeId,
			const tpl::RBDState<NJOINTS, SCALAR>& rbdState)
	{
		Velocity3Tpl eeVelocityBase = getEEVelocityInBase(eeId, rbdState);
		return rbdState.base().pose().rotateBaseToInertia(eeVelocityBase);
	}

	Position3Tpl getEEPositionInBase(size_t eeID, const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return robcogen().getEEPositionInBase(eeID, jointPosition);
	}


	Position3Tpl getEEPositionInWorld(size_t eeID, const tpl::RigidBodyPose<SCALAR>& basePose, const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		// vector from base to endeffector expressed in world frame
		Position3Tpl W_x_EE =  basePose.template rotateBaseToInertia(Position3Tpl(getEEPositionInBase(eeID, jointPosition)));

		// vector from origin to endeffector = vector from origin to base + vector from base to endeffector
		return basePose.position() + W_x_EE;
	}

	/**
	 * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is rigidly connected to.
	 * @param W_force Force expressed in world coordinates
	 * @param basePose Pose of the base (in the world)
	 * @param jointPosition Joint angles
	 * @param eeId ID of the end-effector
	 * @return
	 */
	EEForce mapForceFromWorldToLink3d(
			const Vector3Tpl& W_force,
			const tpl::RigidBodyPose<SCALAR>& basePose,
			const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition,
			size_t eeId)
	{
		EEForce eeForce(EEForce::Zero());
		eeForce.force() = W_force;

		return mapForceFromWorldToLink(eeForce, basePose, jointPosition, eeId);
	}

	/**
	 * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is rigidly connected to.
	 * @param W_force Force expressed in world coordinates
	 * @param basePose Pose of the base (in the world)
	 * @param jointPosition Joint angles
	 * @param eeId ID of the end-effector
	 * @return
	 */
	EEForce mapForceFromWorldToLink(
		const EEForce& W_force,
		const tpl::RigidBodyPose<SCALAR>& basePose,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition,
		size_t eeId)
	{
		// get endeffector position in world
		Position3Tpl B_x_EE = getEEPositionInBase(eeId, jointPosition);

		return mapForceFromWorldToLink(
				W_force,
				basePose,
				jointPosition,
				B_x_EE,
				eeId);
	}


	/**
	 * \brief Transforms a force applied at an end-effector and expressed in the world into the link frame the EE is rigidly connected to.
	 * @param W_force Force expressed in world coordinates
	 * @param basePose Pose of the base (in the world)
	 * @param jointPosition Joint angles
	 * @param B_x_EE Position of the end effector in the base
	 * @param eeId ID of the end-effector
	 * @return
	 */
	EEForce mapForceFromWorldToLink(
		const EEForce& W_force,
		const tpl::RigidBodyPose<SCALAR>& basePose,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition,
		const Position3Tpl& B_x_EE,
		size_t eeId)
	{
		// get the link that the EE is attached to
		size_t linkId = getEndEffector(eeId).getLinkId();

		// transform the force/torque to an equivalent force/torque in the base using a lever-arm for the torque
		EEForce B_force;

		B_force.force() = basePose.template rotateInertiaToBase<Vector3Tpl>(W_force.force());
		B_force.torque() = B_x_EE.toImplementation().cross(B_force.force()) + basePose.template rotateInertiaToBase<Vector3Tpl>(W_force.torque());

		// transform force to link on which endeffector sits on
		return EEForce(robcogen().getForceTransformLinkBaseById(linkId, jointPosition) * B_force);
	}


	/**
	 * \brief Transforms a force applied at an end-effector expressed in an arbitrary (end-effector)
	 * frame into the link frame the EE is rigidly connected to.
	 *
	 * This function does not assume any position or orientation of the end-effector. Therefore,
	 * the user must specify the orientation of the end-effector in the base. There are two common
	 * choices: use zero rotation to assume forces/torques are expressed in the base. Or use the
	 * base orientation to assume the end-effector forces/torques are expressed in the world. For
	 * this variant, also see mapForceFromWorldToLink().
	 *
	 * NOTE: Even if zero or base rotation is assumed, you have to pass the correct position of the
	 * end-effector expressed in the base as part of T_B_EE! Do NOT pass the base pose directly here!
	 *
	 * @param EE_force 6D torque/force vector expressed in the EE frame
	 * @param T_B_EE transform from end-effector to base
	 * @param jointPosition joint angles
	 * @param eeId ID of the end-effector
	 * @return
	 */
	EEForce mapForceFromEEToLink(
			const EEForce& EE_force,
			const tpl::RigidBodyPose<SCALAR>& T_B_EE,
			const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition,
			size_t eeId)
	{
		// get the link that the EE is attached to
		size_t linkId = getEndEffector(eeId).getLinkId();

		// transform the force/torque to an equivalent force/torque in the base using a lever-arm for the torque
		EEForce B_force;

		// convenience accessors
		Vector3Tpl B_x_EE = T_B_EE.position().toImplementation();

		// rotate from the end-effector frame to the inertia frame
		B_force.force() = T_B_EE.template rotateBaseToInertia<Vector3Tpl>(EE_force.force());

		// build the cross product and add the torque (which first is rotated from the end-effector to the base)
		B_force.torque() = B_x_EE.cross(B_force.force()) + T_B_EE.template rotateBaseToInertia<Vector3Tpl>(EE_force.torque());

		// transform force to link on which endeffector sits on
		return EEForce(robcogen().getForceTransformLinkBaseById(linkId, jointPosition) * B_force);
	};

	RBD& robcogen() { return *rbdContainer_; }

private:
	std::shared_ptr<RBD> rbdContainer_;
	std::array<EndEffector<NJOINTS, SCALAR>, N_EE> endEffectors_;
	FloatingBaseTransforms<RBD> floatingBaseTransforms_;
};


} /* namespace rbd */
} /* namespace ct */

