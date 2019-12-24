/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/RigidBodyPose.h>

#include "RBDSystem.h"

namespace ct {
namespace rbd {

/**
 * \brief Base class for fix-base robot systems
 */
template <class RBDDynamics, size_t STATE_D, size_t CONTROL_D>
class FixBaseSystemBase : public RBDSystem<RBDDynamics, false>,
                          public core::ControlledSystem<STATE_D, CONTROL_D, typename RBDDynamics::SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! number of end-effectors
    static const size_t N_EE = RBDDynamics::N_EE;
    //! number of joints
    static const size_t NJOINTS = RBDDynamics::NJOINTS;

    using Dynamics = RBDDynamics;
    using SCALAR = typename RBDDynamics::SCALAR;
    using Base = core::ControlledSystem<STATE_D, CONTROL_D, SCALAR>;
    using RigidBodyPose_t = tpl::RigidBodyPose<SCALAR>;

    // typedefs state and controls
    using state_vector_t = ct::core::StateVector<STATE_D, SCALAR>;
    using control_vector_t = core::ControlVector<CONTROL_D, SCALAR>;
    using JointAcceleration_t = JointAcceleration<NJOINTS, SCALAR>;

    //! constructor
    FixBaseSystemBase(const RigidBodyPose_t& basePose = RigidBodyPose_t())
        : Base(), basePose_(basePose), dynamics_(RBDDynamics())
    {
    }

    /*!
     * @brief copy constructor
	 */
    FixBaseSystemBase(const FixBaseSystemBase& arg) : Base(arg), basePose_(arg.basePose_), dynamics_(RBDDynamics()) {}
    //! destructor
    virtual ~FixBaseSystemBase() = default;

    //! get dynamics
    virtual RBDDynamics& dynamics() override { return dynamics_; }
    //! get dynamics (const)
    virtual const RBDDynamics& dynamics() const override { return dynamics_; }
    //! compute the controlled dynamics of the fixed base robotic system
    virtual void computeControlledDynamics(const ct::core::StateVector<STATE_D, SCALAR>& state,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_D, SCALAR>& controlIn,
        ct::core::StateVector<STATE_D, SCALAR>& derivative) override = 0;

    //! deep cloning
    virtual FixBaseSystemBase<RBDDynamics, STATE_D, CONTROL_D>* clone() const override = 0;

    //! compute inverse dynamics torques
    ct::core::ControlVector<NJOINTS> computeIDTorques(const JointState<NJOINTS, SCALAR>& jState,
        const JointAcceleration_t& jAcc = JointAcceleration_t(Eigen::Matrix<SCALAR, NJOINTS, 1>::Zero()))
    {
        ct::core::ControlVector<NJOINTS> torque;
        dynamics_.FixBaseID(jState, jAcc, torque);
        return torque;
    }

protected:
    //! a "dummy" base pose which sets the robot's "fixed" position in the world
    tpl::RigidBodyPose<SCALAR> basePose_;

    //! rigid body dynamics container
    RBDDynamics dynamics_;
};

}  // namespace rbd
}  // namespace ct
