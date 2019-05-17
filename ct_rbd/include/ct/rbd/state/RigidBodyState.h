/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "RigidBodyPose.h"
#include "RigidBodyVelocities.h"

namespace ct {
namespace rbd {
namespace tpl {

/**
 * \brief State (pose and velocities) of a single rigid body
 *
 * \ingroup State
 *
 */
template <typename SCALAR = double>
class RigidBodyState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RigidBodyState(typename RigidBodyPose<SCALAR>::STORAGE_TYPE storage = RigidBodyPose<SCALAR>::EULER)
        : pose_(storage), velocities_()
    {
    }

    //! copy constructor
    RigidBodyState(const RigidBodyState<SCALAR>& arg) : pose_(arg.pose_), velocities_(arg.velocities_) {}
    //! destructor
    virtual ~RigidBodyState(){};

    //! return the rigid body pose
    RigidBodyPose<SCALAR>& pose() { return pose_; }
    //! return the rigid body pose (const)
    const RigidBodyPose<SCALAR>& pose() const { return pose_; }
    //! return the rigid body velocities
    RigidBodyVelocities<SCALAR>& velocities() { return velocities_; }
    //! return the rigid body velocities (const)
    const RigidBodyVelocities<SCALAR>& velocities() const { return velocities_; }
    //! numerically comparing two rigid body states
    bool isApprox(const RigidBodyState& rhs, const double& tol = 1e-10)
    {
        return pose().isNear(rhs.pose(), tol) && velocities().getVector().isApprox(rhs.velocities().getVector(), tol);
    }

    /// @brief get translational velocity
    const kindr::Velocity<SCALAR, 3> computeTranslationalVelocityW() const
    {
        return pose().rotateBaseToInertia(velocities().getTranslationalVelocity());
    }
    kindr::Velocity<SCALAR, 3> computeTranslationalVelocityW()
    {
        return pose().rotateBaseToInertia(velocities().getTranslationalVelocity());
    }

    void setIdentity()
    {
        pose().setIdentity();
        velocities().setZero();
    }

    void setRandom()
    {
        pose().setRandom();
        velocities().getTranslationalVelocity().toImplementation().setRandom();
        velocities().getRotationalVelocity().toImplementation().setRandom();
    }

private:
    RigidBodyPose<SCALAR> pose_;
    RigidBodyVelocities<SCALAR> velocities_;
};

}  // namespace tpl

// convenience typedef (required)
typedef tpl::RigidBodyState<double> RigidBodyState;

} /* namespace rbd */
} /* namespace ct */
