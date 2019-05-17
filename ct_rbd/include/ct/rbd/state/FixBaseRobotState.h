/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "RBDState.h"

namespace ct {
namespace rbd {

/*!
 *
 * \ingroup State
 *
 * @brief whole fix base robot state, i.e. Joint state, Actuator Dynamics (and fix-base pose)
 *
 * \note a base pose is not included on purpose, since that would rather be part of the "output equation" than of the state
 *
 * @param NJOINTS the number of robot joints
 * @param ACT_STATE_DIM the state dimension of the actuator
 * @param SCALAR the scalar type
 */
template <size_t NJOINTS, size_t ACT_STATE_DIM = 0, typename SCALAR = double>
class FixBaseRobotState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using RigidBodyPose_t = tpl::RigidBodyPose<SCALAR>;
    using JointState_t = JointState<NJOINTS, SCALAR>;
    using RBDState_t = RBDState<NJOINTS, SCALAR>;

    static const size_t NDOF = NJOINTS;
    static const size_t NSTATE = 2 * NJOINTS + ACT_STATE_DIM;

    using actuator_state_vector_t = ct::core::StateVector<ACT_STATE_DIM, SCALAR>;
    using state_vector_t = ct::core::StateVector<NSTATE, SCALAR>;

    //! constructor
    FixBaseRobotState() : joints_(), act_state_(actuator_state_vector_t::Zero()) {}
    //! constructor
    FixBaseRobotState(const JointState_t& joints,
        const actuator_state_vector_t& actState = actuator_state_vector_t::Zero())
        : joints_(joints), act_state_(actState)
    {
    }

    //! constructor
    FixBaseRobotState(const state_vector_t& robotState) : joints_(), act_state_() { fromStateVector(robotState); }
    //! constructor
    FixBaseRobotState(const FixBaseRobotState& other) : joints_(other.joints_), act_state_(other.act_state_) {}
    //! destructor
    ~FixBaseRobotState() {}
    //! full state vector to JointState (static)
    static const JointState_t jointStateFromVector(const state_vector_t& robotState)
    {
        JointState_t jointState;
        jointState.getPositions() = robotState.template head<NJOINTS>();
        jointState.getVelocities() = robotState.template segment<NJOINTS>(NJOINTS);
        return jointState;
    }

    //! transform full robot state to RBDState (static)
    static const RBDState_t rbdStateFromVector(const state_vector_t& robotState,
        const RigidBodyPose_t& basePose = RigidBodyPose_t())
    {
        RBDState_t rbdState;
        rbdState.setZero();
        rbdState.basePose() = basePose;
        rbdState.joints() = jointStateFromVector(robotState);
        return rbdState;
    }

    //! transform full robot state to a actuator state (static)
    static const actuator_state_vector_t actuatorStateFromVector(const state_vector_t& robotState)
    {
        actuator_state_vector_t actState = robotState.template tail<ACT_STATE_DIM>();
        return actState;
    }

    //! transform from joint and actuator state to full state
    static const state_vector_t toFullState(const ct::core::StateVector<2 * NJOINTS, SCALAR>& jointState,
        const actuator_state_vector_t& x_act = actuator_state_vector_t::Zero())
    {
        state_vector_t robotState;
        robotState.template head<2 * NJOINTS>() = jointState.template head<2 * NJOINTS>();
        robotState.template tail<ACT_STATE_DIM>() = x_act;
        return robotState;
    }

    //! accessor to actuator state
    actuator_state_vector_t& actuatorState() { return act_state_; }
    //! accessor to actuator state
    const actuator_state_vector_t& actuatorState() const { return act_state_; }
    //! accessor to single element of actuator state
    SCALAR& actuatorState(size_t i) { return act_state_(i); }
    //! accessor to single element of actuator state
    const SCALAR& actuatorState(size_t i) const { return act_state_(i); }
    //! accessor to joint state
    JointState_t& joints() { return joints_; }
    //! accessor to joint state
    const JointState_t& joints() const { return joints_; }
    void setZero()
    {
        joints_.setZero();
        act_state_.setZero();
    }

    //! conversion from dense state vector to a RobotState
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type fromStateVector(const state_vector_t& robotState)
    {
        joints_.getPositions() = robotState.template head<NJOINTS>();
        joints_.getVelocities() = robotState.template segment<NJOINTS>(NJOINTS);
    }

    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type fromStateVector(const state_vector_t& robotState)
    {
        joints_.getPositions() = robotState.template head<NJOINTS>();
        joints_.getVelocities() = robotState.template segment<NJOINTS>(NJOINTS);
        act_state_ = robotState.template tail<ACT_STATE_DIM>();
    }

    //! conversion from RobotState to dense state vector
    template <typename T = state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type toStateVector() const
    {
        state_vector_t robotState;
        robotState.template head<NJOINTS>() = joints_.getPositions();
        robotState.template segment<NJOINTS>(NJOINTS) = joints_.getVelocities();
        return robotState;
    }
    template <typename T = state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type toStateVector() const
    {
        state_vector_t robotState;
        robotState.template head<NJOINTS>() = joints_.getPositions();
        robotState.template segment<NJOINTS>(NJOINTS) = joints_.getVelocities();
        robotState.template tail<ACT_STATE_DIM>() = act_state_;
        return robotState;
    }

    //! transform full robot state to RBDState
    RBDState_t toRBDState(const RigidBodyPose_t& basePose = RigidBodyPose_t())
    {
        RBDState_t rbdState;
        rbdState.setZero();
        rbdState.basePose() = basePose;
        rbdState.joints() = joints_;
        return rbdState;
    }

protected:
    //! the joint state
    JointState_t joints_;
    //! the actuator state vector
    actuator_state_vector_t act_state_;
};

}  // namespace rbd
}  // namespace ct
