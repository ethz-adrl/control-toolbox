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

#ifndef INCLUDE_CT_RBD_ROBOT_RBDCONTAINER_H_
#define INCLUDE_CT_RBD_ROBOT_RBDCONTAINER_H_

#include <kindr/Core>
#include <ct/rbd/state/JointState.h>

namespace ct {
namespace rbd {

/**
 * \brief Container class containing all robcogen classes
 */
template <class RBDTrait, template <typename> class LinkDataMapT, class U>
class RobCoGenContainer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	RobCoGenContainer()
		: homogeneousTransforms_(),
		  motionTransforms_(),
		  forceTransforms_(),
		  jacobians_(),
		  inertiaProperties_(),
		  jSim_(inertiaProperties_, forceTransforms_),
		  forwardDynamics_(inertiaProperties_, motionTransforms_),
		  inverseDynamics_(inertiaProperties_, motionTransforms_){};

	typedef typename RBDTrait::S SCALAR;

	typedef RBDTrait TRAIT;

	typedef U UTILS;

	static const size_t NJOINTS = RBDTrait::joints_count;
	static const size_t NLINKS = RBDTrait::links_count;

	typedef RobCoGenContainer<RBDTrait, LinkDataMapT, UTILS> specialized_t;
	typedef std::shared_ptr<specialized_t> Ptr_t;

	typedef typename RBDTrait::HomogeneousTransforms HomogeneousTransforms;
	typedef typename RBDTrait::MotionTransforms MotionTransforms;
	typedef typename RBDTrait::ForceTransforms ForceTransforms;
	typedef typename RBDTrait::Jacobians Jacobians;
	typedef typename RBDTrait::InertiaProperties InertiaProperties;
	typedef typename RBDTrait::JSIM JSIM;
	typedef typename RBDTrait::FwdDynEngine ForwardDynamics;
	typedef typename RBDTrait::InvDynEngine InverseDynamics;
	typedef typename RBDTrait::LinkID LinkIdentifiers;

	typedef LinkDataMapT<Eigen::Matrix<SCALAR, 6, 1>> LinkForceMap;

	typedef Eigen::Matrix<SCALAR, 4, 4> HomogeneousTransform;
	typedef Eigen::Matrix<SCALAR, 6, 6> ForceTransform;
	typedef Eigen::Matrix<SCALAR, 6, NJOINTS> Jacobian;

	typedef kindr::Position<SCALAR, 3> Position3Tpl;
	typedef Eigen::Matrix<SCALAR, 3, 1> Vector3Tpl;


	HomogeneousTransforms& homogeneousTransforms() { return homogeneousTransforms_; };
	const HomogeneousTransforms& homogeneousTransforms() const { return homogeneousTransforms_; };
	MotionTransforms& motionTransforms() { return motionTransforms_; };
	const MotionTransforms& motionTransforms() const { return motionTransforms_; };
	ForceTransforms& forceTransforms() { return forceTransforms_; };
	const ForceTransforms& forceTransforms() const { return forceTransforms_; };
	Jacobians& jacobians() { return jacobians_; };
	const Jacobians& jacobians() const { return jacobians_; };
	InertiaProperties& inertiaProperties() { return inertiaProperties_; };
	const InertiaProperties& inertiaProperties() const { return inertiaProperties_; };
	JSIM& jSim() { return jSim_; }
	const JSIM& jSim() const { return jSim_; }
	ForwardDynamics& forwardDynamics() { return forwardDynamics_; };
	const ForwardDynamics& forwardDynamics() const { return forwardDynamics_; };
	InverseDynamics& inverseDynamics() { return inverseDynamics_; };
	const InverseDynamics& inverseDynamics() const { return inverseDynamics_; };
	/**
	 * \brief Get a homogeneous transformation from link to base provided a link id
	 *
	 * The homogeneous transform takes a quantity expressed in the link frame and transforms
	 * it into the base frame, i.e. \f$ {_Bx} = T_{BL} {_Lx} \f$
	 * where \f$ T_{BL}  \f$ is the transform you obtain from this function.
	 *
	 * @param linkId link id
	 * @param position joint positions (angles)
	 * @return Homogeneous transformation from link to base \f$ T_{BL}  \f$
	 */
	HomogeneousTransform getHomogeneousTransformBaseLinkById(size_t linkId,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return UTILS::getTransformBaseLinkById(homogeneousTransforms(), linkId, jointPosition);
	}

	/**
	 * \brief Get a force transformation from link to base provided a link id
	 *
	 * The force transform takes a torque-force vector expressed in the base frame and transforms
	 * it into the link frame, i.e. \f$ {_Lx} = T_{LB} {_Bx} \f$
	 * where \f$ T_{LB}  \f$ is the transform you obtain from this function.
	 *
	 * @param linkId link id
	 * @param position joint positions (angles)
	 * @return Force transformation from link to base \f$ T_{LB}  \f$
	 */
	ForceTransform getForceTransformLinkBaseById(size_t linkId,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return UTILS::getTransformLinkBaseById(forceTransforms(), linkId, jointPosition);
	}

	/**
	 * \brief Get a homogeneous transformation from base to an endeffector provided an endeffector id
	 *
	 * The homogeneous transform converts a quantity expressed in the base and converts it to the end-effector frame, i.e.
	 * \f$ {_EEx} = T_{EE-B} {_Bx}  \f$
	 * The endeffector frame corresponds to the convention used when creating the RobCoGen code.
	 *
	 * @param eeId End-effector ID
	 * @param jointPosition Joint angles/positions
	 * @return Homogeneous transformation from base to endeffector \f$ T_{EE-B}  \f$
	 */
	HomogeneousTransform getHomogeneousTransformBaseEEById(size_t eeId,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return UTILS::getTransformBaseEEById(homogeneousTransforms(), eeId, jointPosition);
	}


	/*!
	 * \brief Get the end-effector Jacobian expressed in the base frame
	 * @param eeId endeffector ID
	 * @param jointPosition current joint position
	 * @return Jacobian of the endeffector expressed in the base frame
	 */
	Jacobian getJacobianBaseEEbyId(size_t eeId,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return UTILS::getJacobianBaseEEbyId(jacobians(), eeId, jointPosition);
	}

	//	/**
	//	 * \brief Get a force transformation from base to an endeffector provided an endeffector id
	//	 *
	//	 * The force transform converts a torque-force vector expressed in the base and converts it to the end-effector frame, i.e.
	//	 * \f$ {_EEx} = T_{EE-B} {_Bx}  \f$
	//	 * The endeffector frame corresponds to the convention used when creating the RobCoGen code.
	//	 *
	//	 * @param eeId End-effector ID
	//	 * @param jointPosition Joint angles/positions
	//	 * @return Homogeneous transformation from base to endeffector \f$ T_{EE-B}  \f$
	//	 */
	// ForceTransform getForceTransformEEBaseById(size_t eeId, const typename JointState<NJOINTS>::Position& jointPosition);

	/*!
	 * \brief Get the endeffector position expressed in the base frame
	 *
	 * @param eeId endeffector ID
	 * @param jointPosition current joint position
	 * @return position of the endeffector expressed in the base frame
	 */
	Position3Tpl getEEPositionInBase(size_t eeId,
		const typename tpl::JointState<NJOINTS, SCALAR>::Position& jointPosition)
	{
		return Position3Tpl(getHomogeneousTransformBaseEEById(eeId, jointPosition).template topRightCorner<3, 1>());
	}


private:
	HomogeneousTransforms homogeneousTransforms_;
	MotionTransforms motionTransforms_;
	ForceTransforms forceTransforms_;
	Jacobians jacobians_;
	InertiaProperties inertiaProperties_;
	JSIM jSim_;
	ForwardDynamics forwardDynamics_;
	InverseDynamics inverseDynamics_;
};


}  // namespace rbd
}  // namespace ct


#endif /* INCLUDE_CT_RBD_ROBOT_RBDCONTAINER_H_ */
