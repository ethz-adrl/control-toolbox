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
 * @brief whole robot state, i.e. RBDState and Actuator Dynamics
 *
 * @param NJOINTS the number of robot joints
 * @param ACT_STATE_DIM the state dimension of the actuator
 * @param SCALAR the scalar type
 */
template <size_t NJOINTS, size_t ACT_STATE_DIM = 0, typename SCALAR = double>
class FloatingBaseRobotState : public RBDState<NJOINTS, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base_t = RBDState<NJOINTS, SCALAR>;
    using RBDState_t = RBDState<NJOINTS, SCALAR>;

    using RigidBodyPose_t = tpl::RigidBodyPose<SCALAR>;
    using RigidBodyState_t = tpl::RigidBodyState<SCALAR>;
    using JointState_t = JointState<NJOINTS, SCALAR>;
    using STORAGE_TYPE = typename RigidBodyPose_t::STORAGE_TYPE;  //! Euler or quat

    using actuator_state_vector_t = ct::core::StateVector<ACT_STATE_DIM, SCALAR>;
    static const size_t NDOF = Base_t::NDOF;
    static const size_t NSTATE = Base_t::NSTATE + ACT_STATE_DIM;
    static const size_t NSTATE_QUAT = NSTATE + 1;

    using state_vector_quat_t = ct::core::StateVector<NSTATE_QUAT, SCALAR>;
    using state_vector_euler_t = ct::core::StateVector<NSTATE, SCALAR>;


    //! default constructor
    FloatingBaseRobotState(const STORAGE_TYPE storage = STORAGE_TYPE::EULER)
        : Base_t(storage), act_state_(actuator_state_vector_t::Zero())
    {
    }

    //! constructor with actuator state as parameter
    FloatingBaseRobotState(const RBDState_t& rbdState,
        const actuator_state_vector_t& act_state = actuator_state_vector_t::Zero())
        : Base_t(rbdState), act_state_(act_state)
    {
    }

    //! constructor with base, jonit and actuator state as parameters
    FloatingBaseRobotState(const RigidBodyState_t& baseState,
        const JointState_t& jointState,
        const actuator_state_vector_t& act_state = actuator_state_vector_t::Zero())
        : Base_t(baseState, jointState), act_state_(act_state)
    {
    }

    //! copy constructor
    FloatingBaseRobotState(const FloatingBaseRobotState& other) : Base_t(other), act_state_(other.act_state_) {}
    //! destructor
    ~FloatingBaseRobotState() {}
    //! accessor to base class RBDState
    RBDState_t& rbdState() { return *this; }
    //! accessor to base class RBDState
    const RBDState_t& rbdState() const { return *this; }
    //! accessor to actuator state
    actuator_state_vector_t& actuatorState() { return act_state_; }
    //! accessor to actuator state
    const actuator_state_vector_t& actuatorState() const { return act_state_; }
    //! compute state vector including with quaternion representation
    template <typename T = state_vector_quat_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type toStateVectorQuaternion() const
    {
        // assemble overall state from RBDState and actuator state
        state_vector_quat_t state;
        state << Base_t::toStateVectorQuaternion(), act_state_;
        return state;
    }
    template <typename T = state_vector_quat_t>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type toStateVectorQuaternion() const
    {
        // transcribe RBDState only
        return Base_t::toStateVectorQuaternion();
    }

    //! get unique state vector with euler coordinates
    template <typename T = state_vector_euler_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type toStateVectorEulerXyzUnique() const
    {
        // assemble overall state from RBDState and actuator state
        state_vector_euler_t state;
        state << Base_t::toStateVectorEulerXyzUnique(), act_state_;
        return state;
    }
    template <typename T = state_vector_euler_t>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type toStateVectorEulerXyzUnique() const
    {
        // transcribe RBDState only
        return Base_t::toStateVectorEulerXyzUnique();
    }

    //! get state vector in euler coordinates
    template <typename T = state_vector_euler_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type toStateVectorEulerXyz() const
    {
        // the overall state consists of RBDState and actuator state
        state_vector_euler_t state;
        state << Base_t::toStateVectorEulerXyz(), act_state_;
        return state;
    }
    template <typename T = state_vector_euler_t>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type toStateVectorEulerXyz() const
    {
        // transcribe RBDState only
        return Base_t::toStateVectorEulerXyz();
    }

    //! reconstruct robot state from an Eigen state-vector with quaternion representation
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type fromStateVectorQuaternion(const state_vector_quat_t& state)
    {
        // transcribe the first part of the state in to the RBDState and the second part into the actuator state
        Eigen::VectorBlock<const Eigen::Matrix<SCALAR, NSTATE_QUAT, 1>, Base_t::NSTATE_QUAT> stateBase =
            state.template head<Base_t::NSTATE_QUAT>();
        Base_t::fromStateVectorQuaternion(stateBase);
        act_state_ = state.template tail<ACT_STATE_DIM>();
    }
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type fromStateVectorQuaternion(const state_vector_quat_t& state)
    {
        // transcribe RBDState only
        Base_t::fromStateVectorQuaternion(state);
    }

    //! reconstruct robot state from an Eigen state-vector with euler representation
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type fromStateVectorEulerXyz(const state_vector_euler_t& state)
    {
        // transcribe the first part of the state in to the RBDState and the second part into the actuator state
        Eigen::VectorBlock<const Eigen::Matrix<SCALAR, NSTATE, 1>, Base_t::NSTATE> stateBase =
            state.template head<Base_t::NSTATE>();
        Base_t::fromStateVectorEulerXyz(stateBase);
        act_state_ = state.template tail<ACT_STATE_DIM>();
    }
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type fromStateVectorEulerXyz(const state_vector_euler_t& state)
    {
        // transcribe RBDState only
        Base_t::state_vector_euler_t(state);
    }

    //! a convenience method
    void fromStateVectorRaw(const state_vector_quat_t& state) { fromStateVectorQuaternion(state); }
    //! a convenience method
    void fromStateVectorRaw(const state_vector_euler_t& state) { fromStateVectorEulerXyz(state); }
    //! set all zero
    void setZero()
    {
        act_state_.setZero();
        Base_t::setZero();
    }

protected:
    actuator_state_vector_t act_state_;

private:
    /*the following methods are not to be called from the Robot state. They can be accessed via rbdState(). */
    using Base_t::toCoordinatePositionUnique;
    using Base_t::toCoordinatePosition;
    using Base_t::toCoordinateVelocity;
};

}  // namespace rbd
}  // namespace ct
