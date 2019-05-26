/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "JointAcceleration.h"
#include "RBDState.h"
#include "RigidBodyAcceleration.h"

namespace ct {
namespace rbd {

/**
 * @class RBDAcceleration
 *
 * \ingroup State
 *
 * @brief joint acceleration and base acceleration
 */
template <size_t NJOINTS, typename SCALAR = double>
class RBDAcceleration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum DIMS
    {
        NDOF = NJOINTS + 6,
        NSTATE = 2 * NDOF  ///< position/velocity of (joints + base)

    };

    typedef Eigen::Matrix<SCALAR, NDOF, 1> coordinate_vector_t;

    typedef tpl::RigidBodyAcceleration<SCALAR> RigidBodyAcceleration_t;


    RBDAcceleration() { setZero(); }
    RBDAcceleration(const RigidBodyAcceleration_t& baseStateDerivative,
        const JointAcceleration<NJOINTS, SCALAR>& jointStateDerivative)
        : baseStateDerivative_(baseStateDerivative), jointStateDerivative_(jointStateDerivative)
    {
    }

    /// @brief get base acceleration
    RigidBodyAcceleration_t& base() { return baseStateDerivative_; }
    /// @brief get constant base acceleration
    const RigidBodyAcceleration_t& base() const { return baseStateDerivative_; }
    /// @brief get joint acceleration
    JointAcceleration<NJOINTS, SCALAR>& joints() { return jointStateDerivative_; }
    /// @brief get constant joint acceleration
    const JointAcceleration<NJOINTS, SCALAR>& joints() const { return jointStateDerivative_; }
    typename RBDState<NJOINTS, SCALAR>::state_vector_quat_t toStateUpdateVectorQuaternion(
        const RBDState<NJOINTS, SCALAR>& state) const
    {
        typename RBDState<NJOINTS, SCALAR>::state_vector_quat_t stateDerivative;

        auto localDiffMat = ct::rbd::getLocalQuaternionDiffMatrix(state.basePose().getRotationQuaternion());

        Eigen::Quaternion<SCALAR> rotationQuaternionDiff(
            0.5 * (localDiffMat.transpose() * state.baseLocalAngularVelocity()));

        stateDerivative << rotationQuaternionDiff.w(), rotationQuaternionDiff.x(), rotationQuaternionDiff.y(),
            rotationQuaternionDiff.z(), state.base().computeTranslationalVelocityW(), state.joints().getVelocities(),
            base().getAngularAcceleration(), base().getTranslationalAcceleration(), joints().getAcceleration();

        return stateDerivative;
    }

    typename RBDState<NJOINTS, SCALAR>::state_vector_euler_t toStateUpdateVectorEulerXyz(
        const RBDState<NJOINTS, SCALAR>& state) const
    {
        typename RBDState<NJOINTS, SCALAR>::state_vector_euler_t stateDerivative;

        Eigen::Matrix<SCALAR, 3, 3> rotMatEulerDiff =
            state.basePose().getEulerAnglesXyz().getMappingFromLocalAngularVelocityToDiff();

        tpl::EulerAnglesXYZ<SCALAR> eulerAnglesXyzDiff = rotMatEulerDiff * state.baseLocalAngularVelocity();

        stateDerivative << eulerAnglesXyzDiff.angles(), state.base().computeTranslationalVelocityW(),
            state.joints().getVelocities(), base().getAngularAcceleration(), base().getTranslationalAcceleration(),
            joints().getAcceleration();

        return stateDerivative;
    }

    coordinate_vector_t toCoordinateAcceleration() const
    {
        coordinate_vector_t ddq;
        ddq << base().getAngularAcceleration(), base().getTranslationalAcceleration(), joints().getAcceleration();
        return ddq;
    }

    void setZero()
    {
        baseStateDerivative_.setZero();
        jointStateDerivative_.setZero();
    }

    static RBDAcceleration<NJOINTS, SCALAR> Zero()
    {
        return RBDAcceleration<NJOINTS, SCALAR>(
            RigidBodyAcceleration_t::Zero(), JointAcceleration<NJOINTS, SCALAR>::Zero());
    }

protected:
    RigidBodyAcceleration_t baseStateDerivative_;
    JointAcceleration<NJOINTS, SCALAR> jointStateDerivative_;
};

}  // namespace rbd
}  // namespace ct
