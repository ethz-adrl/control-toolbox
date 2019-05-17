/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FixBaseSystemBase.h"

namespace ct {
namespace rbd {

/**
 * \brief A simple fix base robot system which is purely kinematic and actuated at VELOCITY level
 *  - the control input vector are the joint velocities
 *  - the state vector are the joint positions
 */
template <class RBDDynamics>
class FixBaseVelSystem : public FixBaseSystemBase<RBDDynamics, RBDDynamics::NJOINTS, RBDDynamics::NJOINTS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = FixBaseSystemBase<RBDDynamics, RBDDynamics::NJOINTS, RBDDynamics::NJOINTS>;
    using SCALAR = typename BASE::SCALAR;
    using state_vector_t = typename BASE::state_vector_t;
    using control_vector_t = typename BASE::control_vector_t;
    using JointAcceleration_t = typename BASE::JointAcceleration_t;
    using RigidBodyPose_t = typename BASE::RigidBodyPose_t;

    //! constructor
    FixBaseVelSystem(const RigidBodyPose_t& basePose = RigidBodyPose_t()) : BASE(basePose) {}
    //! copy constructor
    FixBaseVelSystem(const FixBaseVelSystem& arg) : BASE(arg) {}
    //! destructor
    virtual ~FixBaseVelSystem() = default;

    //! compute the controlled dynamics of the fixed base robotic system
    void computeControlledDynamics(const state_vector_t& state,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& derivative) override
    {
        derivative.template head<BASE::NJOINTS>() = controlIn.template head<BASE::NJOINTS>();
    }

    //! deep cloning
    virtual FixBaseVelSystem<RBDDynamics>* clone() const override { return new FixBaseVelSystem<RBDDynamics>(*this); }
};

}  // namespace rbd
}  // namespace ct
