/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FixBaseSystemBase.h"

namespace ct {
namespace rbd {

/**
 * \brief A simple fix base robot system which is purely kinematic and actuated at ACCELERATION level
 *  - the control input vector are the joint accelerations
 *  - the state vector are the joint positions and velocities
 */
template <class RBDDynamics>
class FixBaseAccSystem : public FixBaseSystemBase<RBDDynamics, 2 * RBDDynamics::NJOINTS, RBDDynamics::NJOINTS>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = FixBaseSystemBase<RBDDynamics, 2 * RBDDynamics::NJOINTS, RBDDynamics::NJOINTS>;
    using SCALAR = typename BASE::SCALAR;
    using state_vector_t = typename BASE::state_vector_t;
    using control_vector_t = typename BASE::control_vector_t;
    using JointAcceleration_t = typename BASE::JointAcceleration_t;
    using RigidBodyPose_t = typename BASE::RigidBodyPose_t;

    //! constructor
    FixBaseAccSystem(const RigidBodyPose_t& basePose = RigidBodyPose_t()) : BASE(basePose) {}
    //! copy constructor
    FixBaseAccSystem(const FixBaseAccSystem& arg) : BASE(arg) {}
    //! destructor
    virtual ~FixBaseAccSystem() = default;

    //! compute the controlled dynamics of the fixed base robotic system
    void computeControlledDynamics(const state_vector_t& state,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& derivative) override
    {
        // transcribe joint velocities
        derivative.template head<BASE::NJOINTS>() = state.template tail<BASE::NJOINTS>();
        // transcribe joint accelerations
        derivative.template tail<BASE::NJOINTS>() = controlIn.template head<BASE::NJOINTS>();
    }

    //! deep cloning
    virtual FixBaseAccSystem<RBDDynamics>* clone() const override { return new FixBaseAccSystem<RBDDynamics>(*this); }
};

}  // namespace rbd
}  // namespace ct
