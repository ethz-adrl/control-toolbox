/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "JointState.h"
#include "RigidBodyState.h"

namespace ct {
namespace rbd {

/** \defgroup State State
  * \brief Different state types for Rigid Bodies, Robots etc.
  */


/*!
 *
 * \ingroup State
 *
 * @brief joint states and base states
 */

template <size_t NJOINTS, typename SCALAR = double>
class RBDState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t NDOF = NJOINTS + 6;
    static const size_t NSTATE = 2 * (NJOINTS + 6);
    static const size_t NSTATE_QUAT = NSTATE + 1;

    using RigidBodyState_t = tpl::RigidBodyState<SCALAR>;
    using RigidBodyPose_t = tpl::RigidBodyPose<SCALAR>;
    typedef ct::core::StateVector<NSTATE_QUAT, SCALAR> state_vector_quat_t;
    typedef ct::core::StateVector<NSTATE, SCALAR> state_vector_euler_t;
    typedef ct::core::StateVector<NDOF, SCALAR> coordinate_vector_t;

    RBDState(typename RigidBodyPose_t::STORAGE_TYPE storage = RigidBodyPose_t::EULER) : baseState_(storage)
    {
        baseState_.setIdentity();
        jointState_.setZero();
    }

    RBDState(const RBDState& other) : baseState_(other.baseState_), jointState_(other.jointState_) {}
    RBDState(const RigidBodyState_t& baseState, const JointState<NJOINTS, SCALAR>& jointState)
        : baseState_(baseState), jointState_(jointState)
    {
    }

    virtual ~RBDState() = default;

    bool operator!=(const RBDState& other) const { return (base() != other.base() || joints() != other.joints()); }
    bool isApprox(const RBDState& rhs, const SCALAR& tol = 1e-10)
    {
        return base().isApprox(rhs.base(), tol) && joints().isApprox(rhs.joints(), tol);
    }

    /// @brief get base states
    RigidBodyState_t& base() { return baseState_; }
    /// @brief get constant base states
    const RigidBodyState_t& base() const { return baseState_; }
    /// @brief get base pose
    RigidBodyPose_t& basePose() { return base().pose(); }
    /// @brief get constant base states
    const RigidBodyPose_t& basePose() const { return base().pose(); }
    /// @brief get base velocities
    tpl::RigidBodyVelocities<SCALAR>& baseVelocities() { return base().velocities(); }
    /// @brief get constant base velocities
    const tpl::RigidBodyVelocities<SCALAR>& baseVelocities() const { return base().velocities(); }
    /// @brief get base local angular velocity
    kindr::LocalAngularVelocity<SCALAR>& baseLocalAngularVelocity()
    {
        return base().velocities().getRotationalVelocity();
    }
    /// @brief get constant base local angular velocity
    const kindr::LocalAngularVelocity<SCALAR>& baseLocalAngularVelocity() const
    {
        return base().velocities().getRotationalVelocity();
    }

    /// @brief get base linear velocity
    kindr::Velocity<SCALAR, 3>& baseLinearVelocity() { return base().velocities().getTranslationalVelocity(); }
    /// @brief get constant base local angular velocity
    const kindr::Velocity<SCALAR, 3>& baseLinearVelocity() const
    {
        return base().velocities().getTranslationalVelocity();
    }

    /// @brief get joint states
    JointState<NJOINTS, SCALAR>& joints() { return jointState_; }
    /// @brief get constant joint states
    const JointState<NJOINTS, SCALAR>& joints() const { return jointState_; }
    //BLOCKS ARE ALREADY REFERENCES, DO NOT MAKE THESE RETURN REFERENCES TO BLOCKS
    /// @brief get joint positions
    typename JointState<NJOINTS, SCALAR>::JointPositionBlock jointPositions() { return joints().getPositions(); }
    /// @brief get constant joint positions
    const typename JointState<NJOINTS, SCALAR>::JointPositionBlockConst jointPositions() const
    {
        return joints().getPositions();
    }

    //BLOCKS ARE ALREADY REFERENCES, DO NOT MAKE THESE RETURN REFERENCES TO BLOCKS
    /// @brief get joint velocities
    typename JointState<NJOINTS, SCALAR>::JointPositionBlock jointVelocities() { return joints().getVelocities(); }
    /// @brief get constant joint velocities
    const typename JointState<NJOINTS, SCALAR>::JointPositionBlockConst jointVelocities() const
    {
        return joints().getVelocities();
    }

    state_vector_quat_t toStateVectorQuaternion() const
    {
        state_vector_quat_t state;

        state << base().pose().getRotationQuaternion().w(), base().pose().getRotationQuaternion().x(),
            base().pose().getRotationQuaternion().y(), base().pose().getRotationQuaternion().z(),
            base().pose().position().toImplementation(), joints().getPositions(),
            base().velocities().getRotationalVelocity().toImplementation(),
            base().velocities().getTranslationalVelocity().toImplementation(), joints().getVelocities();

        return state;
    }

    coordinate_vector_t toCoordinatePositionUnique() const
    {
        coordinate_vector_t q;
        q << base().pose().getEulerAnglesXyz().getUnique().toImplementation(),
            base().pose().position().toImplementation(), joints().getPositions();
        return q;
    }

    coordinate_vector_t toCoordinatePosition() const
    {
        coordinate_vector_t q;
        q << base().pose().getEulerAnglesXyz().toImplementation(), base().pose().position().toImplementation(),
            joints().getPositions();
        return q;
    }

    coordinate_vector_t toCoordinateVelocity() const
    {
        coordinate_vector_t dq;
        dq << base().velocities().getRotationalVelocity().toImplementation(),
            base().velocities().getTranslationalVelocity().toImplementation(), joints().getVelocities();
        return dq;
    }

    state_vector_euler_t toStateVectorEulerXyzUnique() const
    {
        state_vector_euler_t state;

        state << toCoordinatePositionUnique(), toCoordinateVelocity();

        return state;
    }

    state_vector_euler_t toStateVectorEulerXyz() const
    {
        state_vector_euler_t state;

        state << toCoordinatePosition(), toCoordinateVelocity();

        return state;
    }

    void fromStateVectorQuaternion(const state_vector_quat_t& state)
    {
        base().pose().setFromRotationQuaternion(kindr::RotationQuaternion<SCALAR>(state.template head<4>()));
        base().pose().position().toImplementation() = state.template segment<3>(4);
        joints().getPositions() = state.template segment<NJOINTS>(7);
        base().velocities().getRotationalVelocity().toImplementation() = state.template segment<3>(7 + NJOINTS);
        base().velocities().getTranslationalVelocity().toImplementation() = state.template segment<3>(10 + NJOINTS);
        joints().getVelocities() = state.template tail<NJOINTS>();
    }

    void fromStateVectorEulerXyz(const state_vector_euler_t& state)
    {
        base().pose().setFromEulerAnglesXyz(state.template head<3>());
        base().pose().position().toImplementation() = state.template segment<3>(3);
        joints().getPositions() = state.template segment<NJOINTS>(6);
        base().velocities().getRotationalVelocity().toImplementation() = state.template segment<3>(6 + NJOINTS);
        base().velocities().getTranslationalVelocity().toImplementation() = state.template segment<3>(9 + NJOINTS);
        joints().getVelocities() = state.template tail<NJOINTS>();
    }

    void fromStateVectorRaw(const state_vector_quat_t& state) { fromStateVectorQuaternion(state); }
    void fromStateVectorRaw(const state_vector_euler_t& state) { fromStateVectorEulerXyz(state); }
    virtual void setDefault()
    {
        baseState_.setIdentity();
        jointState_.setZero();
    }

    void setZero()
    {
        baseState_.setIdentity();
        jointState_.setZero();
    }

    void setRandom()
    {
        baseState_.setRandom();
        jointState_.setRandom();
    }


protected:
    RigidBodyState_t baseState_;
    JointState<NJOINTS, SCALAR> jointState_;
};

}  // namespace rbd
}  // namespace ct
