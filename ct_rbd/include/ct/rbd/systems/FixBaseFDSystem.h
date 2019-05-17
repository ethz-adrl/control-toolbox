/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include "FixBaseSystemBase.h"
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>
#include <ct/rbd/state/FixBaseRobotState.h>

namespace ct {
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics.
 *
 * A fix base rigid body system that uses forward dynamics. The control input vector is assumed to consist of
 *  - joint torques and
 *  - end effector forces expressed in the world frame
 *
 * The overall state vector is arranged in the order
 * - joint positions
 * - joint velocities
 * - actuator state
 *
 * \warning when modelled with RobCoGen, the base pose must be rotated "against gravity" (RobCoGen modeling assumption)
 */
template <class RBDDynamics, size_t ACT_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class FixBaseFDSystem : public FixBaseSystemBase<RBDDynamics,
                            RBDDynamics::NSTATE + ACT_STATE_DIM,                                   // state dim
                            RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3>  // control dim
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t N_EE = RBDDynamics::N_EE;                         //! number of end-effectors
    static const size_t STATE_DIM = RBDDynamics::NSTATE + ACT_STATE_DIM;  // combined state dim
    static const size_t CONTROL_DIM = RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * N_EE * 3;  // combined control dim
    static const size_t ACTUATOR_STATE_DIM = ACT_STATE_DIM;

    using BASE = FixBaseSystemBase<RBDDynamics, STATE_DIM, CONTROL_DIM>;
    using SCALAR = typename BASE::SCALAR;
    using state_vector_t = typename BASE::state_vector_t;
    using control_vector_t = typename BASE::control_vector_t;
    using JointAcceleration_t = typename BASE::JointAcceleration_t;
    using RigidBodyPose_t = typename BASE::RigidBodyPose_t;

    using ActuatorDynamics_t = ActuatorDynamics<ACTUATOR_STATE_DIM, RBDDynamics::NJOINTS, SCALAR>;
    using FixBaseRobotState_t = FixBaseRobotState<BASE::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>;
    using actuator_state_vector_t = typename FixBaseRobotState_t::actuator_state_vector_t;


    /*!
     * @brief constructor
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    FixBaseFDSystem(const RigidBodyPose_t& basePose = RigidBodyPose_t()) : BASE(basePose), actuatorDynamics_(nullptr) {}
    /*!
     * @brief constructor including actuator dynamics
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    FixBaseFDSystem(std::shared_ptr<ActuatorDynamics_t> actuatorDynamics,
        const RigidBodyPose_t& basePose = RigidBodyPose_t())
        : BASE(basePose), actuatorDynamics_(actuatorDynamics)
    {
    }

    /*!
     * @brief copy constructor
	 *
	 * @param arg instance of FixBaseFDSystem to be copied.
	 *
	 * \note takes care of explicitly cloning actuatorDynamics, if existent
	 */
    FixBaseFDSystem(const FixBaseFDSystem& arg) : BASE(arg)
    {
        if (arg.actuatorDynamics_)
        {
            actuatorDynamics_ = std::shared_ptr<ActuatorDynamics_t>(arg.actuatorDynamics_->clone());
        }
    }

    //! destructor
    virtual ~FixBaseFDSystem() = default;

    //! compute the controlled dynamics of the fixed base robotic system
    void computeControlledDynamics(const state_vector_t& state,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& derivative) override
    {
        FixBaseRobotState_t robotState(state);

        // map the joint velocities (unchanged, no damping)
        derivative.template topRows<BASE::NJOINTS>() = robotState.joints().getVelocities();

        // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
        control_vector_t control = controlIn;

        // compute actuator dynamics and their control output
        computeActuatorDynamics(robotState, t, controlIn, derivative, control);

        // Cache updated rbd state
        typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

        // add end effector forces as control inputs (if applicable)
        if (EE_ARE_CONTROL_INPUTS == true)
        {
            for (size_t i = 0; i < RBDDynamics::N_EE; i++)
            {
                auto endEffector = this->dynamics_.kinematics().getEndEffector(i);
                size_t linkId = endEffector.getLinkId();
                linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
                    this->dynamics_.kinematics().mapForceFromWorldToLink3d(
                        control.template segment<3>(BASE::NJOINTS + i * 3), this->basePose_,
                        robotState.joints().getPositions(), i);
            }
        }

        typename RBDDynamics::JointAcceleration_t jAcc;

        this->dynamics_.FixBaseForwardDynamics(
            robotState.joints(), control.template head<BASE::NJOINTS>(), linkForces, jAcc);

        derivative.template segment<BASE::NJOINTS>(BASE::NJOINTS) = jAcc.getAcceleration();
    }

    //! evaluate the actuator dynamics if actuators are enabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeActuatorDynamics(const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
        // get references to the current actuator position, velocity and input
        const Eigen::Ref<const typename control_vector_t::Base> actControlIn =
            controlIn.template topRows<BASE::NJOINTS>();

        actuator_state_vector_t actStateDerivative;  // todo use vector block for this?
        actuatorDynamics_->computeActuatorDynamics(
            robotState.joints(), robotState.actuatorState(), t, actControlIn, actStateDerivative);

        stateDerivative.template tail<ACTUATOR_STATE_DIM>() = actStateDerivative;

        // overwrite control with actuator control output as a function of current robot and actuator states
        controlOut = actuatorDynamics_->computeControlOutput(robotState.joints(), robotState.actuatorState());
    }

    //! do nothing if actuators disabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type computeActuatorDynamics(
        const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
    }

    //! deep cloning
    virtual FixBaseFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new FixBaseFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>(*this);
    }

    //! get pointer to actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> getActuatorDynamics() { return actuatorDynamics_; }
    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeConsistentActuatorState(
        const JointState<BASE::NJOINTS, SCALAR>& jStateRef,
        const ct::core::ControlVector<BASE::NJOINTS>& torqueRef)
    {
        return actuatorDynamics_->computeStateFromOutput(jStateRef, torqueRef);
    }

    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeConsistentActuatorState(
        const JointState<BASE::NJOINTS, SCALAR>& jStateRef)
    {
        const ct::core::ControlVector<BASE::NJOINTS> torqueRef = computeIDTorques(jStateRef);
        return computeConsistentActuatorState(jStateRef, torqueRef);
    }


private:
    //! pointer to the actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> actuatorDynamics_;
};

}  // namespace rbd
}  // namespace ct
