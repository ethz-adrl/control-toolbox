/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include "kinematics/RBDDataMap.h"
#include "Kinematics.h"
#include "ProjectedDynamics.h"
#include "state/JointAcceleration.h"
#include "state/JointState.h"
#include "state/RBDAcceleration.h"
#include "state/RBDState.h"
#include "state/RigidBodyAcceleration.h"
#include "control/SelectionMatrix.h"

#define ENABLE_FIX_BASE    \
    template <bool B = FB> \
    typename std::enable_if<!B, void>::type
#define ENABLE_FIX_BASE_IMPL \
    template <bool B>        \
    typename std::enable_if<!B, void>::type
#define ENABLE_FLOAT_BASE  \
    template <bool B = FB> \
    typename std::enable_if<B, void>::type
#define ENABLE_FLOAT_BASE_IMPL \
    template <bool B>          \
    typename std::enable_if<B, void>::type


namespace ct {
namespace rbd {

/**
 * @brief This class implements the equations of motion of a Rigid Body System
 * @tparam RBD  The rbd container class
 * @tparam NEE  The number of endeffectors
 */
template <class RBD, size_t NEE>
class Dynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef RBD ROBCOGEN;
    typedef typename ROBCOGEN::SCALAR SCALAR;

    static const bool FB = ROBCOGEN::TRAIT::floating_base;

    static const size_t NJOINTS = RBD::NJOINTS;
    static const size_t NLINKS = RBD::NLINKS;


    // NSTATE either includes the base or is just the joint state
    static const size_t NSTATE = FB * RBDState<NJOINTS, SCALAR>::NSTATE + (1 - FB) * 2 * NJOINTS;
    static const size_t N_EE = NEE;

    //temporary definitions, ideally this should come from RBD
    typedef Eigen::Matrix<SCALAR, NJOINTS, 1> control_vector_t;
    typedef Eigen::Matrix<SCALAR, NSTATE, 1> state_vector_t;
    typedef Eigen::Matrix<SCALAR, 6, 1> Vector6d_t;
    typedef Vector6d_t ForceVector_t;

    typedef RBDState<NJOINTS, SCALAR> RBDState_t;
    typedef RBDAcceleration<NJOINTS, SCALAR> RBDAcceleration_t;
    typedef JointState<NJOINTS, SCALAR> JointState_t;
    typedef JointAcceleration<NJOINTS, SCALAR> JointAcceleration_t;
    typedef tpl::RigidBodyAcceleration<SCALAR> RigidBodyAcceleration_t;

    // currently assumes our input is the joint dimension and we only affect acceleration
    typedef SelectionMatrix<NJOINTS, NSTATE / 2, SCALAR> SelectionMatrix_t;

    // From the container
    typedef typename RBD::LinkForceMap ExtLinkForces_t;

    typedef RBDDataMap<bool, NEE> EE_in_contact_t;

    typedef Kinematics<RBD, NEE> Kinematics_t;

    /**
	 * @brief The constructor
	 * @param[in] 	kinematics	The kinematics of the RBD
	 */
    Dynamics(typename Kinematics_t::Ptr_t kinematics = typename Kinematics_t::Ptr_t(new Kinematics_t()))
        : S_(FB), kinematics_(kinematics), p_dynamics_(kinematics)
    {
    }

    Dynamics(const Dynamics& other) : S_(other.S_), kinematics_(other.kinematics_->clone()), p_dynamics_(kinematics_) {}
    virtual ~Dynamics(){};


    /**
	 * @brief Compute forward dynamics of a fixed-base RBD system under external
	 * forces
	 * @param[in] 	x		The JointState state
	 * @param[in] 	u		The control vector
	 * @param[in] 	force	The external forces vector
	 * @param[out]	qdd		The joints acceleration
	 */
    ENABLE_FIX_BASE FixBaseForwardDynamics(const JointState_t& x,
        const control_vector_t& u,
        ExtLinkForces_t& force,
        JointAcceleration_t& qdd);

    /**
	 * @brief Compute forward dynamics of a fixed-base RBD system,  NO contact forces
	 * forces
	 * @param[in] 	x	The RBD state
	 * @param[in] 	u	The control vector
	 * @param[out]	qdd	The Joints acceleration
	 */
    ENABLE_FIX_BASE FixBaseForwardDynamics(const JointState_t& x, const control_vector_t& u, JointAcceleration_t& qdd)
    {
        ExtLinkForces_t force(Eigen::Matrix<SCALAR, 6, 1>::Zero());
        FixBaseForwardDynamics(x, u, force, qdd);
    }

    /**
	 * @brief Computes Inverse dynamics of a fixed-base system under external
	 * forces.
	 * @param[in] 	x		the current state of the robot
	 * @param[in] 	qdd		the Joints acceleration
	 * @param[in] 	force	the external forces vector
	 * @param[out]	u		The control vector
	 */
    ENABLE_FIX_BASE FixBaseID(const JointState_t& x,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        control_vector_t& u);

    /**
	 * @brief Computes Inverse dynamics of a fixed-base system without external
	 * forces.
	 * @param[in] 	x		the current state of the robot
	 * @param[in] 	qdd		the Joints acceleration
	 * @param[out]	u		The control vector
	 */
    ENABLE_FIX_BASE FixBaseID(const JointState_t& x, const JointAcceleration_t& qdd, control_vector_t& u);

    /**
	 * @brief Compute forward dynamics for an floating-base RBD system under external
	 * forces
	 * @param[in] 	x           The RBD state
	 * @param[in] 	u           The control vector
	 * @param[in] 	link_forces The external forces vector
	 * @param[out]	xd          The RBD state derivative
	 */
    ENABLE_FLOAT_BASE FloatingBaseForwardDynamics(const RBDState_t& x,
        const control_vector_t& u,
        const ExtLinkForces_t& link_forces,
        RBDAcceleration_t& xd);

    /**
	 * @brief Computes Inverse dynamics of a floating-base system under external
	 * forces.
	 * @param[in] 	x		the RBDstate
	 * @param[in] 	qdd		the joints acceleration
	 * @param[in] 	force	the external forces vector
	 * @param[out]	u		The control vector
	 * @param[out]  base_a  The base state derivative
	 */
    ENABLE_FLOAT_BASE FloatingBaseID(const RBDState_t& x,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        control_vector_t& u,
        RigidBodyAcceleration_t& base_a);

    /**
	 * @brief Computes the inverse dynamics of a floating-base fully-actuated
	 * system
	 * @param[in]	x		The RBDState
	 * @param[in]	base_a	The base acceleration
	 * @param[in]	qdd		The joint acceleration
	 * @param[in]	force	The external forces vector
	 * @param[out]	base_w	The base wrench
	 * @param[out]	u		The control vector
	 */
    ENABLE_FLOAT_BASE FloatingBaseFullyActuatedID(const RBDState_t& x,
        const RigidBodyAcceleration_t& base_ac,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        ForceVector_t& base_w,
        control_vector_t& u);


    /**
	 * @brief Computes Projected Forward Dynamics of a floating-base system with contact constraints
	 * @param[in]	ee_contact	the EE contact configuration (EEDataMap<bool>)
	 * @param[in]   x			the RBD state
	 * @param[in]	u			the control vector
	 * @param[out]  rbd_a		the RBD acceleration
	 */
    ENABLE_FLOAT_BASE ProjectedForwardDynamics(const EE_in_contact_t& ee_contact,
        const RBDState_t& x,
        const control_vector_t& u,
        RBDAcceleration_t& rbd_a)
    {
        // optional, for caching
        ee_contact_ = ee_contact;
        p_dynamics_.setContactConfiguration(ee_contact);

        p_dynamics_.ProjectedForwardDynamics(x, u, rbd_a);
    }

    ENABLE_FLOAT_BASE ProjectedInverseDynamics(const EE_in_contact_t& ee_contact,
        const RBDState_t& x,
        const RBDAcceleration_t& rbd_a,
        control_vector_t& u)
    {
        // optional, for caching
        ee_contact_ = ee_contact;
        p_dynamics_.setContactConfiguration(ee_contact);
        p_dynamics_.ProjectedInverseDynamics(x, rbd_a, u);
    }

    ENABLE_FLOAT_BASE ProjectedInverseDynamicsNoGravity(const EE_in_contact_t& ee_contact,
        const RBDState_t& x,
        const RBDAcceleration_t& rbd_a,
        control_vector_t& u)
    {
        // optional, for caching
        ee_contact_ = ee_contact;
        p_dynamics_.setContactConfiguration(ee_contact);
        p_dynamics_.ProjectedInverseDynamicsNoGravity(x, rbd_a, u);
    }

    Kinematics_t& kinematics() { return *kinematics_; }
    const Kinematics_t& kinematics() const { return *kinematics_; }
    typename Kinematics_t::Ptr_t& kinematicsPtr() { return kinematics_; }
    const typename Kinematics_t::Ptr_t& kinematicsPtr() const { return kinematics_; }
    SelectionMatrix_t& S() { return S_; }
    const SelectionMatrix_t& S() const { return S_; }
private:
    SelectionMatrix_t S_;

    EE_in_contact_t ee_contact_ = false;

    typename Kinematics_t::Ptr_t kinematics_; /*!< The RBD kinematics */
    ProjectedDynamics<RBD, NEE> p_dynamics_;  /*!< The Projected Dynamics */

protected:
};


template <class RBD, size_t NEE>
ENABLE_FIX_BASE_IMPL Dynamics<RBD, NEE>::FixBaseForwardDynamics(const JointState_t& x,
    const control_vector_t& u,
    ExtLinkForces_t& force,
    JointAcceleration_t& qdd)
{
    kinematics_->robcogen().forwardDynamics().fd(qdd.getAcceleration(), x.getPositions(), x.getVelocities(), u, force);
}

template <class RBD, size_t NEE>
ENABLE_FIX_BASE_IMPL Dynamics<RBD, NEE>::FixBaseID(const JointState_t& x,
    const JointAcceleration_t& qdd,
    const ExtLinkForces_t& force,
    control_vector_t& u)
{
    kinematics_->robcogen().inverseDynamics().id(u, x.getPositions(), x.getVelocities(), qdd.getAcceleration(), force);
}

template <class RBD, size_t NEE>
ENABLE_FIX_BASE_IMPL Dynamics<RBD, NEE>::FixBaseID(const JointState_t& x,
    const JointAcceleration_t& qdd,
    control_vector_t& u)
{
    ExtLinkForces_t force(Eigen::Matrix<SCALAR, 6, 1>::Zero());
    kinematics_->robcogen().inverseDynamics().id(u, x.getPositions(), x.getVelocities(), qdd.getAcceleration(), force);
}

template <class RBD, size_t NEE>
ENABLE_FLOAT_BASE_IMPL Dynamics<RBD, NEE>::FloatingBaseForwardDynamics(const RBDState_t& x,
    const control_vector_t& u,
    const ExtLinkForces_t& l_forces,
    RBDAcceleration_t& xd)
{
    Vector6d_t b_accel;

    kinematics_->robcogen().forwardDynamics().fd(xd.joints().getAcceleration(), b_accel, x.baseVelocities().getVector(),
        x.basePose().computeGravityB6D(), x.joints().getPositions(), x.joints().getVelocities(), u, l_forces);

    xd.base().fromVector6d(b_accel);
}

template <class RBD, size_t NEE>
ENABLE_FLOAT_BASE_IMPL Dynamics<RBD, NEE>::FloatingBaseID(const RBDState_t& x,
    const JointAcceleration_t& qdd,
    const ExtLinkForces_t& l_forces,
    control_vector_t& u,
    RigidBodyAcceleration_t& base_a)
{
    Vector6d_t b_accel;

    kinematics_->robcogen().inverseDynamics().id(u,
        b_accel,  //Output
        x.basePose().computeGravityB6D(), x.baseVelocities().getVector(), x.joints().getPositions(),
        x.joints().getVelocities(), qdd.getAcceleration(), l_forces);

    base_a.fromVector6d(b_accel);
}

template <class RBD, size_t NEE>
ENABLE_FLOAT_BASE_IMPL Dynamics<RBD, NEE>::FloatingBaseFullyActuatedID(const RBDState_t& x,
    const RigidBodyAcceleration_t& base_a,
    const JointAcceleration_t& qdd,
    const ExtLinkForces_t& l_forces,
    ForceVector_t& base_w,
    control_vector_t& u)
{
    Vector6d_t b_accel;

    kinematics_->robcogen().inverseDynamics().id_fully_actuated(base_w, u, x.basePose().computeGravityB6D(),
        x.baseVelocities().getVector(), b_accel, x.joints().getPositions(), x.joints().getVelocities(),
        qdd.getAcceleration(), l_forces);
}

#undef ENABLE_FIX_BASE
#undef ENABLE_FIX_BASE_IMPL
#undef ENABLE_FLOAT_BASE
#undef ENABLE_FLOAT_BASE_IMPL

}  // namespace rbd
}  // namespace ct
