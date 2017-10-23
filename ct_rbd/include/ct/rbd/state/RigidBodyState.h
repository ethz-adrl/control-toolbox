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

#ifndef INCLUDE_CT_RBD_ROBOT_STATE_RIGIDBODYSTATE_H_
#define INCLUDE_CT_RBD_ROBOT_STATE_RIGIDBODYSTATE_H_

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

    RigidBodyState(typename RigidBodyPose<SCALAR>::STORAGE_TYPE storage = RigidBodyPose<SCALAR>::QUAT)
        : pose_(storage), velocities_()
    {
    }

    virtual ~RigidBodyState(){};

    RigidBodyPose<SCALAR>& pose() { return pose_; }
    const RigidBodyPose<SCALAR>& pose() const { return pose_; }
    RigidBodyVelocities<SCALAR>& velocities() { return velocities_; }
    const RigidBodyVelocities<SCALAR>& velocities() const { return velocities_; }
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

typedef tpl::RigidBodyState<double> RigidBodyState;

} /* namespace rbd */
} /* namespace ct */

#endif /* INCLUDE_CT_RBD_ROBOT_STATE_RIGIDBODYSTATE_H_ */
